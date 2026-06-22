// Copyright (c) 2026, voshch
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright notice,
//      this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above copyright notice,
//      this list of conditions and the following disclaimer in the documentation
//      and/or other materials provided with the distribution.
//    * Neither the name of the copyright holder nor the names of its contributors
//      may be used to endorse or promote products derived from this software
//      without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Ported from ros-visualization/rviz_animated_view_controller (BSD-3-Clause),
// adapting its smooth eye/focus/up interpolation to the rviz_common (ROS 2) API.

#include "rviz_viewport_control/viewport_view_controller.hpp"

#include <QEvent>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>

#include <OgreCamera.h>
#include <OgreSceneNode.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/tf_frame_property.hpp>
#include <rviz_common/properties/vector_property.hpp>
#include <rviz_common/viewport_mouse_event.hpp>

namespace rviz_viewport_control
{

namespace
{
// Camera body convention shared with the gazebo backend: forward +X, up +Z.
constexpr double kLookEps = 1e-9;

Ogre::Vector3 toOgre(const geometry_msgs::msg::Point & p)
{
  return Ogre::Vector3(
    static_cast<Ogre::Real>(p.x),
    static_cast<Ogre::Real>(p.y),
    static_cast<Ogre::Real>(p.z));
}

Ogre::Quaternion toOgre(const geometry_msgs::msg::Quaternion & q)
{
  return Ogre::Quaternion(
    static_cast<Ogre::Real>(q.w),
    static_cast<Ogre::Real>(q.x),
    static_cast<Ogre::Real>(q.y),
    static_cast<Ogre::Real>(q.z));
}

// Zero-roll look-at orientation in the body convention (forward +X, up +Z), so a
// streamed pose and an eye -> target placement agree on what "looking" means.
Ogre::Quaternion lookAtOrientation(const Ogre::Vector3 & eye, const Ogre::Vector3 & target)
{
  Ogre::Vector3 dir = target - eye;
  if (dir.length() < kLookEps) {
    return Ogre::Quaternion::IDENTITY;
  }
  dir.normalise();
  const double yaw = std::atan2(dir.y, dir.x);
  const double pitch = -std::asin(std::clamp<double>(dir.z, -1.0, 1.0));
  Ogre::Quaternion q;
  q.FromAngleAxis(Ogre::Radian(static_cast<Ogre::Real>(yaw)), Ogre::Vector3::UNIT_Z);
  Ogre::Quaternion p;
  p.FromAngleAxis(Ogre::Radian(static_cast<Ogre::Real>(pitch)), Ogre::Vector3::UNIT_Y);
  return q * p;
}

}  // namespace

ViewportViewController::ViewportViewController() = default;

ViewportViewController::~ViewportViewController() = default;

void ViewportViewController::onInitialize()
{
  rviz_common::FramePositionTrackingViewController::onInitialize();

  camera_->setProjectionType(Ogre::PT_PERSPECTIVE);

  distance_property_ = new rviz_common::properties::FloatProperty(
    "Distance", 10.0f, "Initial camera distance from the focal point.", this);
  distance_property_->setMin(0.01f);

  focal_point_property_ = new rviz_common::properties::VectorProperty(
    "Focal Point", Ogre::Vector3::ZERO,
    "Initial point the camera looks at, in the target frame.", this);

  pitch_property_ = new rviz_common::properties::FloatProperty(
    "Pitch", 0.785f, "Initial camera elevation over the focal point, in radians.", this);

  yaw_property_ = new rviz_common::properties::FloatProperty(
    "Yaw", 0.0f, "Initial camera azimuth about the focal point, in radians.", this);

  publish_period_property_ = new rviz_common::properties::FloatProperty(
    "Pose Publish Period", 0.1f,
    "Seconds between camera_pose publications, 0 disables publishing.", this);
  publish_period_property_->setMin(0.0f);

  stream_timeout_property_ = new rviz_common::properties::FloatProperty(
    "Stream Timeout", 0.4f,
    "Release the camera to manual control this long after the last cmd_view.", this);
  stream_timeout_property_->setMin(0.0f);

  setupRos();
}

void ViewportViewController::reset()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    one_shot_ = false;
    streaming_ = false;
    view_buffer_.clear();
  }
  needs_initial_placement_ = true;
}

void ViewportViewController::setupRos()
{
  const auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction) {
    return;
  }
  node_ = ros_node_abstraction->get_raw_node();

  set_view_srv_ = node_->create_service<SetView>(
    "viewport/set_view",
    [this](const std::shared_ptr<SetView::Request> req, std::shared_ptr<SetView::Response> res) {
      const Ogre::Vector3 eye = toOgre(req->eye);
      const Ogre::Vector3 target = toOgre(req->target);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        one_shot_position_ = eye;
        one_shot_orientation_ = lookAtOrientation(eye, target);
        one_shot_ = true;
        streaming_ = false;
        if (req->fov > 0.0) {
          pending_fov_ = req->fov;
        }
      }
      res->success = true;
      res->message = "ok";
    });

  set_reference_frame_srv_ = node_->create_service<SetReferenceFrame>(
    "viewport/set_reference_frame",
    [this](
      const std::shared_ptr<SetReferenceFrame::Request> req,
      std::shared_ptr<SetReferenceFrame::Response> res) {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_mode_ = req->mode;
      if (!req->entity.empty()) {
        pending_frame_ = req->entity;
        res->message = "tracking " + req->entity;
      } else {
        // Empty entity selects the fixed/world frame, the has_pose / latch nuance of
        // the gazebo backend has no rviz analogue beyond the fixed frame.
        pending_frame_ = std::string();
        res->message = req->has_pose ? "constant reference unsupported, using fixed frame" :
        "using fixed frame";
      }
      res->success = true;
    });

  set_projection_srv_ = node_->create_service<SetProjection>(
    "viewport/set_projection",
    [this](
      const std::shared_ptr<SetProjection::Request> req,
      std::shared_ptr<SetProjection::Response> res) {
      if (req->projection == "perspective") {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_orthographic_ = false;
      } else if (req->projection == "orthographic") {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_orthographic_ = true;
      } else {
        res->success = false;
        res->message = "projection must be 'perspective' or 'orthographic'";
        return;
      }
      res->success = true;
      res->message = "ok";
    });

  // Deep queue: every keyframe must reach the buffer, not just the latest one.
  view_sub_ = node_->create_subscription<ViewportView>(
    "viewport/cmd_view",
    rclcpp::QoS(rclcpp::KeepLast(64)).best_effort(),
    [this](const ViewportView::SharedPtr msg) {
      Keyframe kf;
      // target_time is wall time from the cam, so tag and compare on the system clock.
      kf.time = rclcpp::Time(msg->target_time, RCL_SYSTEM_TIME);
      kf.position = toOgre(msg->pose.position);
      kf.orientation = toOgre(msg->pose.orientation);
      kf.world_orientation = msg->world_orientation;
      kf.fov = msg->fov;
      std::lock_guard<std::mutex> lock(mutex_);
      view_buffer_.push_back(kf);
      streaming_ = true;
      last_view_ = stream_clock_.now();
    });

  capture_srv_ = node_->create_service<Capture>(
    "viewport/capture",
    [](const std::shared_ptr<Capture::Request>, std::shared_ptr<Capture::Response> res) {
      res->success = false;
      res->message = "capture not yet supported";
    });

  pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "viewport/camera_pose", rclcpp::QoS(10));
}

Ogre::Quaternion ViewportViewController::reduceReference(
  const Ogre::Quaternion & orientation, uint8_t mode)
{
  switch (mode) {
    case SetReferenceFrame::Request::YAW_ONLY: {
        Ogre::Quaternion yaw;
        yaw.FromAngleAxis(orientation.getYaw(), Ogre::Vector3::UNIT_Z);
        return yaw;
      }
    case SetReferenceFrame::Request::POSITION_ONLY:
      return Ogre::Quaternion::IDENTITY;
    default:  // FULL
      return orientation;
  }
}

// A keyframe leading now hides publish jitter, mirroring the gazebo SampleBuffer.
bool ViewportViewController::sampleBuffer(
  const rclcpp::Time & now, Ogre::Vector3 & position, Ogre::Quaternion & orientation,
  bool & world_orientation, double & fov) const
{
  const std::size_t n = view_buffer_.size();
  if (n == 0) {
    return false;
  }
  if (now <= view_buffer_.front().time) {
    const Keyframe & kf = view_buffer_.front();
    position = kf.position;
    orientation = kf.orientation;
    world_orientation = kf.world_orientation;
    fov = kf.fov;
    return true;
  }
  if (now >= view_buffer_.back().time) {
    const Keyframe & kf = view_buffer_.back();
    position = kf.position;
    orientation = kf.orientation;
    world_orientation = kf.world_orientation;
    fov = kf.fov;
    return true;
  }
  std::size_t i = 1;
  while (i < n && view_buffer_[i].time < now) {
    ++i;
  }
  const Keyframe & a = view_buffer_[i - 1];
  const Keyframe & b = view_buffer_[i];
  const double span = (b.time - a.time).seconds();
  const double alpha = span > kLookEps ? (now - a.time).seconds() / span : 1.0;
  position = a.position + (b.position - a.position) * static_cast<Ogre::Real>(alpha);
  orientation = Ogre::Quaternion::Slerp(
    static_cast<Ogre::Real>(alpha), a.orientation, b.orientation, true);
  world_orientation = b.world_orientation;
  fov = b.fov > 0.0 ? b.fov : a.fov;
  return true;
}

void ViewportViewController::update(std::chrono::nanoseconds dt, std::chrono::nanoseconds ros_dt)
{
  // Base resolves the target frame onto target_scene_node_ (reference_position_ and
  // reference_orientation_), so the camera scene node lives in reference-local coords.
  rviz_common::FramePositionTrackingViewController::update(dt, ros_dt);

  std::optional<std::string> requested_frame;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::swap(requested_frame, pending_frame_);
    if (pending_mode_) {
      reference_mode_ = *pending_mode_;
      pending_mode_.reset();
    }
  }
  if (requested_frame) {
    const QString frame = requested_frame->empty() ?
      rviz_common::properties::TfFrameProperty::FIXED_FRAME_STRING :
      QString::fromStdString(*requested_frame);
    target_frame_property_->setValue(frame);
  }

  // YAW_ONLY / POSITION_ONLY drop reference rotation channels, so the local offset
  // stays level or world-aligned even while the tracked frame rolls or pitches.
  if (reference_mode_ != SetReferenceFrame::Request::FULL && target_scene_node_) {
    target_scene_node_->setOrientation(
      reduceReference(reference_orientation_, reference_mode_));
  }

  if (camera_) {
    applyProjection();
    applyToCamera(std::chrono::duration<float>(ros_dt).count());
  }
}

void ViewportViewController::applyProjection()
{
  std::optional<bool> orthographic;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::swap(orthographic, pending_orthographic_);
  }
  if (orthographic) {
    camera_->setProjectionType(*orthographic ? Ogre::PT_ORTHOGRAPHIC : Ogre::PT_PERSPECTIVE);
  }
}

void ViewportViewController::applyInitialPlacement()
{
  const double distance = distance_property_->getFloat();
  const double yaw = yaw_property_->getFloat();
  const double pitch = pitch_property_->getFloat();
  const Ogre::Vector3 focus = focal_point_property_->getVector();
  // Spherical offset in the target frame, matching the rviz Orbit convention: yaw is
  // azimuth in the XY plane, pitch lifts the eye above the focal point.
  const Ogre::Vector3 offset(
    static_cast<Ogre::Real>(distance * std::cos(yaw) * std::cos(pitch)),
    static_cast<Ogre::Real>(distance * std::sin(yaw) * std::cos(pitch)),
    static_cast<Ogre::Real>(distance * std::sin(pitch)));
  const Ogre::Vector3 eye = focus + offset;
  const Ogre::Quaternion orientation = lookAtOrientation(eye, focus);
  orbit_distance_ = distance;
  placeCamera(eye, orientation, false);
  applied_position_ = eye;
  applied_orientation_ = orientation;
  placed_ = true;
}

void ViewportViewController::placeCamera(
  const Ogre::Vector3 & eye, const Ogre::Quaternion & orientation, bool world_orientation)
{
  Ogre::SceneNode * camera_parent = camera_->getParentSceneNode();
  if (!camera_parent) {
    return;
  }
  // Look direction and up taken in the body convention (forward +X, up +Z), the same
  // basis lookAtOrientation produces, so setDirection / setFixedYawAxis map them onto
  // the Ogre camera (-Z forward) consistently for both set_view and the stream. The
  // position stays parent-local (reference * local), world_orientation aims the camera
  // in world space so the look stays stable while the reference rotates under it.
  const Ogre::Vector3 direction = orientation * Ogre::Vector3::UNIT_X;
  const Ogre::Vector3 up = orientation * Ogre::Vector3::UNIT_Z;
  const Ogre::Node::TransformSpace space =
    world_orientation ? Ogre::Node::TS_WORLD : Ogre::Node::TS_PARENT;
  camera_parent->setPosition(eye);
  camera_parent->setFixedYawAxis(true, up);
  camera_parent->setDirection(direction, space);
}

void ViewportViewController::applyToCamera(float ros_dt)
{
  if (!node_) {
    return;
  }
  const rclcpp::Time now = stream_clock_.now();
  const double timeout = stream_timeout_property_->getFloat();

  bool one_shot = false;
  bool streaming = false;
  Ogre::Vector3 position{Ogre::Vector3::ZERO};
  Ogre::Quaternion orientation{Ogre::Quaternion::IDENTITY};
  bool world_orientation = false;
  double sampled_fov = 0.0;
  std::optional<double> pending_fov;
  bool have_local = false;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    one_shot = one_shot_;
    one_shot_ = false;

    // Drop keyframes fully in the past, keeping the one bracketing now.
    while (view_buffer_.size() > 1 && view_buffer_[1].time <= now) {
      view_buffer_.pop_front();
    }
    streaming = streaming_;
    if (streaming && (now - last_view_).seconds() > timeout) {
      streaming_ = false;  // stream went quiet, release to manual
      streaming = false;
    }

    if (one_shot) {
      position = one_shot_position_;
      orientation = one_shot_orientation_;
      have_local = true;
    } else if (streaming) {
      have_local = sampleBuffer(now, position, orientation, world_orientation, sampled_fov);
    }
    std::swap(pending_fov, pending_fov_);
  }

  if (pending_fov) {
    // Treat fov as horizontal, convert to Ogre's vertical FOVy via the aspect ratio.
    const double aspect = camera_->getAspectRatio() > 0.0f ? camera_->getAspectRatio() : 1.0;
    camera_->setFOVy(
      Ogre::Radian(static_cast<Ogre::Real>(2.0 *
      std::atan(std::tan(*pending_fov / 2.0) / aspect))));
  } else if (sampled_fov > 0.0) {
    const double aspect = camera_->getAspectRatio() > 0.0f ? camera_->getAspectRatio() : 1.0;
    camera_->setFOVy(
      Ogre::Radian(static_cast<Ogre::Real>(2.0 * std::atan(std::tan(sampled_fov / 2.0) / aspect))));
  }

  if (have_local) {
    needs_initial_placement_ = false;
    // Only re-render when the local pose actually changes, so a held or idle stream
    // does not pin the GUI thread to a continuous redraw of a static view.
    if (!placed_ || position != applied_position_ || orientation != applied_orientation_) {
      placeCamera(position, orientation, world_orientation);
      applied_position_ = position;
      applied_orientation_ = orientation;
      placed_ = true;
      context_->queueRender();
    }
  } else if (needs_initial_placement_) {
    needs_initial_placement_ = false;
    applyInitialPlacement();
    context_->queueRender();
  }

  // Publish the live pose on a period, mirroring the gazebo backend's pose feed.
  const double publish_period = publish_period_property_->getFloat();
  if (publish_period > 0.0) {
    publish_accumulator_ += ros_dt;
    if (publish_accumulator_ >= publish_period) {
      publish_accumulator_ = 0.0;
      publishPose();
    }
  }
}

void ViewportViewController::publishPose()
{
  if (!pose_pub_ || !camera_) {
    return;
  }
  const Ogre::Vector3 pos = camera_->getDerivedPosition();
  const Ogre::Quaternion rot = camera_->getDerivedOrientation();
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = node_->now();
  msg.header.frame_id = context_->getFixedFrame().toStdString();
  msg.pose.position.x = pos.x;
  msg.pose.position.y = pos.y;
  msg.pose.position.z = pos.z;
  msg.pose.orientation.w = rot.w;
  msg.pose.orientation.x = rot.x;
  msg.pose.orientation.y = rot.y;
  msg.pose.orientation.z = rot.z;
  pose_pub_->publish(msg);
}

void ViewportViewController::lookAt(const Ogre::Vector3 & point)
{
  Ogre::SceneNode * node = camera_ ? camera_->getParentSceneNode() : nullptr;
  const Ogre::Vector3 eye = node ? node->getPosition() : Ogre::Vector3::ZERO;
  std::lock_guard<std::mutex> lock(mutex_);
  one_shot_position_ = eye;
  one_shot_orientation_ = lookAtOrientation(eye, point);
  one_shot_ = true;
  streaming_ = false;
}

void ViewportViewController::handleMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  // Manual interaction overrides any pending drive, the user grabbed the view.
  const bool acting =
    event.left() || event.middle() || event.right() || event.wheel_delta != 0;
  if (acting) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      one_shot_ = false;
      streaming_ = false;
    }
    needs_initial_placement_ = false;
    placed_ = false;  // manual move invalidates the cache, so the next drive re-applies
  }

  Ogre::SceneNode * node = camera_ ? camera_->getParentSceneNode() : nullptr;
  if (!node) {
    return;
  }

  int diff_x = 0;
  int diff_y = 0;
  if (event.type == QEvent::MouseButtonPress) {
    dragging_ = true;
  } else if (event.type == QEvent::MouseButtonRelease) {
    dragging_ = false;
  } else if (dragging_ && event.type == QEvent::MouseMove) {
    diff_x = event.x - event.last_x;
    diff_y = event.y - event.last_y;
  }

  // Orbit about a focal point a fixed reach ahead of the camera, mirroring the
  // animated controller's orbit feel without an eye/focus property tree.
  const Ogre::Vector3 forward = node->getOrientation() * Ogre::Vector3::NEGATIVE_UNIT_Z;
  const Ogre::Vector3 focus = node->getPosition() + forward *
    static_cast<Ogre::Real>(orbit_distance_);

  if (event.left() && !event.shift()) {
    const Ogre::Vector3 offset = node->getPosition() - focus;
    Ogre::Quaternion yaw;
    yaw.FromAngleAxis(Ogre::Radian(-diff_x * 0.005f), Ogre::Vector3::UNIT_Z);
    const Ogre::Vector3 right = node->getOrientation() * Ogre::Vector3::UNIT_X;
    Ogre::Quaternion pitch;
    pitch.FromAngleAxis(Ogre::Radian(-diff_y * 0.005f), right);
    const Ogre::Vector3 rotated = yaw * pitch * offset;
    node->setPosition(focus + rotated);
    node->setFixedYawAxis(true, Ogre::Vector3::UNIT_Z);
    node->setDirection(focus - node->getPosition(), Ogre::Node::TS_PARENT);
  } else if (event.middle() || (event.shift() && event.left())) {
    const Ogre::Vector3 right = node->getOrientation() * Ogre::Vector3::UNIT_X;
    const Ogre::Vector3 up = node->getOrientation() * Ogre::Vector3::UNIT_Y;
    const float scale = static_cast<float>(orbit_distance_) * 0.001f;
    node->translate((right * static_cast<float>(-diff_x) + up * static_cast<float>(diff_y)) *
        scale);
  } else if (event.right()) {
    orbit_distance_ = std::max(0.01, orbit_distance_ + diff_y * 0.01 * orbit_distance_);
    node->setPosition(focus - forward * static_cast<Ogre::Real>(orbit_distance_));
  }

  if (event.wheel_delta != 0) {
    orbit_distance_ = std::max(0.01, orbit_distance_ - event.wheel_delta * 0.001 * orbit_distance_);
    node->setPosition(focus - forward * static_cast<Ogre::Real>(orbit_distance_));
  }

  if (acting) {
    context_->queueRender();
  }
}

}  // namespace rviz_viewport_control

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  rviz_viewport_control::ViewportViewController, rviz_common::ViewController)
