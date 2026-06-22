# rviz_viewport_control

RViz2 view controller that exposes the rviz camera over the same ROS surface
a Gazebo viewport user-camera has, driven by the `viewport_control_msgs` contract.

## Packages

| Package | Description |
| --- | --- |
| `rviz_viewport_control` | RViz2 `ViewController` plugin driven by `viewport_control_msgs`. |
| `viewport_control_msgs` | Message and service definitions for viewport camera control. |

## How it works

The `ViewportViewController` plugin loads as an `rviz_common::ViewController` and
exposes these ROS endpoints (relative to the rviz node namespace):

- `viewport/set_view` (`ViewportSetView`): one-shot: aim from `eye` toward `target`.
- `viewport/set_reference_frame` (`ViewportSetReferenceFrame`): attach to a TF frame with
  FULL, YAW_ONLY, or POSITION_ONLY orientation inheritance.
- `viewport/set_projection` (`ViewportSetProjection`): switch perspective or orthographic.
- `viewport/capture` (`ViewportCapture`): deferred, returns `success=false` for now.
- `viewport/cmd_view` (`ViewportView`, subscribed): streamed keyframe feed, interpolated by
  target time.
- `viewport/camera_pose` (`PoseStamped`, published): live camera world pose.

ROS callbacks store state under a mutex, `update()` applies it on the render thread, so
Ogre is never touched from a callback.

## Selecting the controller

Add the **Viewport Control** view controller in RViz (Views panel -> Type -> rviz_viewport_control/ViewportControl).

## Build

```sh
colcon build --packages-up-to rviz_viewport_control
```

## Branches

This repository follows the ROS distro-branch layout:

- `jazzy` (default): ROS 2 Jazzy and Kilted.
- `lyrical`: ROS 2 Lyrical, where `rviz_common`'s view-controller `update()` takes `std::chrono::nanoseconds`.

## Quality

Both packages claim [REP-2004](https://www.ros.org/reps/rep-2004.html) **Quality Level 3**.
See [rviz_viewport_control/QUALITY_DECLARATION.md](rviz_viewport_control/QUALITY_DECLARATION.md)
and [viewport_control_msgs/QUALITY_DECLARATION.md](viewport_control_msgs/QUALITY_DECLARATION.md).

## License

`rviz_viewport_control` is BSD-3-Clause. `viewport_control_msgs` is Apache-2.0.
