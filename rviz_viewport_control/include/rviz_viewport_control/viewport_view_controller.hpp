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

#ifndef RVIZ_VIEWPORT_CONTROL__VIEWPORT_VIEW_CONTROLLER_HPP_
#define RVIZ_VIEWPORT_CONTROL__VIEWPORT_VIEW_CONTROLLER_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <OgreVector.h>
#include <OgreQuaternion.h>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <rviz_common/frame_position_tracking_view_controller.hpp>

#include <viewport_control_msgs/msg/viewport_view.hpp>
#include <viewport_control_msgs/srv/viewport_capture.hpp>
#include <viewport_control_msgs/srv/viewport_set_projection.hpp>
#include <viewport_control_msgs/srv/viewport_set_reference_frame.hpp>
#include <viewport_control_msgs/srv/viewport_set_view.hpp>

namespace rviz_common
{
namespace properties
{
class FloatProperty;
class VectorProperty;
}  // namespace properties
}  // namespace rviz_common

namespace rviz_viewport_control
{

/// rviz view controller that exposes the rviz camera over the same ROS surface a
/// gazebo viewport user-camera has, driven by the viewport_control_msgs contract.
///
/// Services (relative to the rviz node, namespaced like the gazebo backend):
///   viewport/set_view             ViewportSetView             one-shot look from eye toward target
///   viewport/set_reference_frame  ViewportSetReferenceFrame   frame the view is expressed in
///   viewport/set_projection       ViewportSetProjection       perspective | orthographic
///   viewport/capture              ViewportCapture             deferred, returns success=false
/// Subscribes viewport/cmd_view (ViewportView) for streamed keyframes and publishes
/// the live camera pose on viewport/camera_pose (PoseStamped).
///
/// The base FramePositionTrackingViewController tracks the target TF frame on
/// target_scene_node_, and the camera sits on its child camera_scene_node_, so the
/// camera pose is composed as reference (the scene node) times local (set here),
/// matching the gazebo backend's reference * local. ROS callbacks arrive off the
/// GUI thread, store state under a mutex, and update() applies it on the render
/// thread, never touching Ogre from a callback.
class ViewportViewController : public rviz_common::FramePositionTrackingViewController
{
  Q_OBJECT

public:
  ViewportViewController();
  ~ViewportViewController() override;

  void onInitialize() override;
  void reset() override;
  void update(float dt, float ros_dt) override;
  void lookAt(const Ogre::Vector3 & point) override;
  void handleMouseEvent(rviz_common::ViewportMouseEvent & event) override;

private:
  using SetView = viewport_control_msgs::srv::ViewportSetView;
  using SetReferenceFrame = viewport_control_msgs::srv::ViewportSetReferenceFrame;
  using SetProjection = viewport_control_msgs::srv::ViewportSetProjection;
  using Capture = viewport_control_msgs::srv::ViewportCapture;
  using ViewportView = viewport_control_msgs::msg::ViewportView;

  /// A local-frame camera placement tagged with the wall time it is due.
  struct Keyframe
  {
    rclcpp::Time time;
    Ogre::Vector3 position{Ogre::Vector3::ZERO};
    Ogre::Quaternion orientation{Ogre::Quaternion::IDENTITY};
    bool world_orientation{false};
    double fov{0.0};  // <= 0 leaves fov unchanged
  };

  /// Wire the ROS surface onto the rviz node. Render thread, called from onInitialize.
  void setupRos();

  /// Drain queued ROS state onto the camera and publish its pose. Render thread.
  void applyToCamera(float ros_dt);

  /// Set the projection type, mirroring the gazebo set_projection. Render thread.
  void applyProjection();

  /// Place the camera in target-frame-local coords from an eye and a look direction.
  void placeCamera(
    const Ogre::Vector3 & eye, const Ogre::Quaternion & orientation, bool world_orientation);

  /// Place the camera at the configured initial framing (distance, focal point, pitch, yaw).
  void applyInitialPlacement();

  /// Publish the live camera world pose on camera_pose. Render thread.
  void publishPose();

  /// Reduce a reference orientation per the FULL / YAW_ONLY / POSITION_ONLY mode.
  static Ogre::Quaternion reduceReference(const Ogre::Quaternion & orientation, uint8_t mode);

  /// Sample view_buffer_ at now, lerping position and slerping orientation between the
  /// keyframes bracketing the time, clamping to the ends. Returns false when empty.
  bool sampleBuffer(
    const rclcpp::Time & now, Ogre::Vector3 & position, Ogre::Quaternion & orientation,
    bool & world_orientation, double & fov) const;

  // ROS, owned by the rviz node fetched from the context.
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<SetView>::SharedPtr set_view_srv_;
  rclcpp::Service<SetReferenceFrame>::SharedPtr set_reference_frame_srv_;
  rclcpp::Service<SetProjection>::SharedPtr set_projection_srv_;
  rclcpp::Service<Capture>::SharedPtr capture_srv_;
  rclcpp::Subscription<ViewportView>::SharedPtr view_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  // cmd_view stamps are wall time from the cam, so the buffer is timed on a system
  // clock, not the node's sim clock (rviz runs with use_sim_time).
  rclcpp::Clock stream_clock_{RCL_SYSTEM_TIME};

  // Desired state set off-thread, applied on the render thread, guarded by mutex_.
  std::mutex mutex_;
  bool one_shot_{false};                 // a set_view placement to apply once, then release
  bool streaming_{false};                // cmd_view is driving the camera continuously
  rclcpp::Time last_view_{0, 0, RCL_SYSTEM_TIME};
  Ogre::Vector3 one_shot_position_{Ogre::Vector3::ZERO};
  Ogre::Quaternion one_shot_orientation_{Ogre::Quaternion::IDENTITY};
  std::deque<Keyframe> view_buffer_;     // streamed keyframes, interpolated by time
  std::optional<double> pending_fov_;
  std::optional<bool> pending_orthographic_;
  std::optional<std::string> pending_frame_;  // requested target frame, empty -> fixed frame
  std::optional<uint8_t> pending_mode_;        // requested reference reduction mode

  // Render-thread only.
  uint8_t reference_mode_{SetReferenceFrame::Request::FULL};
  double publish_accumulator_{0.0};
  bool dragging_{false};
  double orbit_distance_{10.0};  // manual orbit reach to the focal point
  bool needs_initial_placement_{true};  // apply the configured initial framing on the next update
  bool placed_{false};  // an applied_* pose exists to diff the next placement against
  Ogre::Vector3 applied_position_{Ogre::Vector3::ZERO};
  Ogre::Quaternion applied_orientation_{Ogre::Quaternion::IDENTITY};

  rviz_common::properties::FloatProperty * publish_period_property_;
  rviz_common::properties::FloatProperty * stream_timeout_property_;
  rviz_common::properties::FloatProperty * distance_property_;
  rviz_common::properties::VectorProperty * focal_point_property_;
  rviz_common::properties::FloatProperty * pitch_property_;
  rviz_common::properties::FloatProperty * yaw_property_;
};

}  // namespace rviz_viewport_control

#endif  // RVIZ_VIEWPORT_CONTROL__VIEWPORT_VIEW_CONTROLLER_HPP_
