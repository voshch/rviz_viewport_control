// Copyright 2026 voshch
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "viewport_control_msgs/msg/viewport_view.hpp"
#include "viewport_control_msgs/srv/viewport_capture.hpp"
#include "viewport_control_msgs/srv/viewport_set_projection.hpp"
#include "viewport_control_msgs/srv/viewport_set_reference_frame.hpp"
#include "viewport_control_msgs/srv/viewport_set_view.hpp"

// Schema guard: assigning each field to an explicitly-typed local makes a
// rename, removal, or retype fail to compile.

TEST(SchemaGuard, ViewportView)
{
  viewport_control_msgs::msg::ViewportView m;
  builtin_interfaces::msg::Time target_time = m.target_time;
  geometry_msgs::msg::Pose pose = m.pose;
  bool world_orientation = m.world_orientation;
  double fov = m.fov;
  (void)target_time;
  (void)pose;
  (void)world_orientation;
  (void)fov;
  SUCCEED();
}

TEST(SchemaGuard, ViewportCaptureRequest)
{
  viewport_control_msgs::srv::ViewportCapture::Request req;
  geometry_msgs::msg::Pose pose = req.pose;
  bool world_orientation = req.world_orientation;
  double fov = req.fov;
  (void)pose;
  (void)world_orientation;
  (void)fov;
  SUCCEED();
}

TEST(SchemaGuard, ViewportCaptureResponse)
{
  viewport_control_msgs::srv::ViewportCapture::Response res;
  bool success = res.success;
  std::string message = res.message;
  sensor_msgs::msg::Image image = res.image;
  (void)success;
  (void)message;
  (void)image;
  SUCCEED();
}

TEST(SchemaGuard, ViewportSetProjectionRequest)
{
  viewport_control_msgs::srv::ViewportSetProjection::Request req;
  std::string projection = req.projection;
  (void)projection;
  SUCCEED();
}

TEST(SchemaGuard, ViewportSetProjectionResponse)
{
  viewport_control_msgs::srv::ViewportSetProjection::Response res;
  bool success = res.success;
  std::string message = res.message;
  (void)success;
  (void)message;
  SUCCEED();
}

TEST(SchemaGuard, ViewportSetReferenceFrameRequest)
{
  viewport_control_msgs::srv::ViewportSetReferenceFrame::Request req;
  std::string entity = req.entity;
  geometry_msgs::msg::Pose pose = req.pose;
  bool has_pose = req.has_pose;
  uint8_t mode = req.mode;
  (void)entity;
  (void)pose;
  (void)has_pose;
  (void)mode;
  SUCCEED();
}

TEST(SchemaGuard, ViewportSetReferenceFrameResponse)
{
  viewport_control_msgs::srv::ViewportSetReferenceFrame::Response res;
  bool success = res.success;
  std::string message = res.message;
  (void)success;
  (void)message;
  SUCCEED();
}

TEST(SchemaGuard, ViewportSetViewRequest)
{
  viewport_control_msgs::srv::ViewportSetView::Request req;
  geometry_msgs::msg::Point eye = req.eye;
  geometry_msgs::msg::Point target = req.target;
  double fov = req.fov;
  (void)eye;
  (void)target;
  (void)fov;
  SUCCEED();
}

TEST(SchemaGuard, ViewportSetViewResponse)
{
  viewport_control_msgs::srv::ViewportSetView::Response res;
  bool success = res.success;
  std::string message = res.message;
  (void)success;
  (void)message;
  SUCCEED();
}
