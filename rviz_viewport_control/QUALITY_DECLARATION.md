This document is a declaration of software quality for the `rviz_viewport_control` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# rviz_viewport_control Quality Declaration

The package `rviz_viewport_control` claims to be in the **Quality Level 3** category.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the Package Requirements for Quality Level 3 in REP-2004.

## Version Policy [1]

### Version Scheme [1.i]
`rviz_viewport_control` uses [`semver`](https://semver.org/).

### Version Stability [1.ii]
`rviz_viewport_control` is at a version below `1.0.0`. A stable version is not required at Quality Level 3.

### Public API Declaration [1.iii]
The public API consists of:
- the `rviz_viewport_control::ViewportViewController` RViz view controller, exported through `pluginlib` as an `rviz_common/ViewController` by `plugin.xml`;
- the ROS surface: `viewport/set_view`, `viewport/set_reference_frame`, `viewport/set_projection`, `viewport/capture` services, `viewport/cmd_view` subscription, and `viewport/camera_pose` publication.

Internal helpers in the anonymous namespace of `src/viewport_view_controller.cpp` are implementation detail and not part of the public API.

### API and ABI Stability [1.iv] - [1.vii]
API and ABI stability is not guaranteed before version `1.0.0`, and is not required at Quality Level 3.

## Change Control Process [2]

### Change Requests [2.i]
All changes occur through a pull request on [GitHub](https://github.com/voshch/rviz_viewport_control).

### Contributor Origin [2.ii]
A formal confirmation of contributor origin (e.g. DCO) is not enforced. This is not required at Quality Level 3.

### Peer Review Policy [2.iii]
A formal peer-review policy is not enforced (single maintainer). This is not required at Quality Level 3.

### Continuous Integration [2.iv]
All pull requests are built and tested by GitHub Actions (`.github/workflows/ci.yaml`) on the `jazzy` and `kilted` distributions. The `lyrical` distribution is tracked on the `lyrical` branch.

### Documentation Policy [2.v]
Changes affecting the public API are reflected in the `README.md` and in the message and service field comments of `viewport_control_msgs`.

## Documentation [3]

### Feature Documentation [3.i]
Features are documented in the repository [`README.md`](../README.md).

### Public API Documentation [3.ii]
The ROS surface is documented in `README.md`; the message and service contract is documented inline in the `.msg` and `.srv` files of `viewport_control_msgs`.

### License [3.iii]
The license is BSD-3-Clause, declared in `package.xml` and reproduced in [`LICENSE`](LICENSE).

### Copyright Statement [3.iv]
Copyright is held by `voshch`, stated in [`LICENSE`](LICENSE).

## Testing [4]
`rviz_viewport_control` has no automated tests; Quality Level 3 imposes no testing requirement. The package builds with `-Wall -Wextra -Wpedantic`, which serves as its static-analysis baseline.

## Dependencies [5]

### Direct Runtime ROS Dependencies [5.i]
- `viewport_control_msgs` (Quality Level 3, this repository)
- `rclcpp` (Quality Level 1)
- `geometry_msgs` (Quality Level 1)
- `sensor_msgs` (Quality Level 1)
- `builtin_interfaces` (Quality Level 1)
- `pluginlib` (see its own Quality Declaration)
- `rviz_common` (no Quality Declaration published)
- `rviz_default_plugins` (no Quality Declaration published)

Quality Level 3 permits dependencies below Level 3. `rviz_common` and `rviz_default_plugins` do not publish Quality Declarations; this is disclosed rather than treated as a blocker.

### Direct Runtime non-ROS Dependencies [5.iii]
- `qtbase5-dev` (Qt 5) or `qt6-base-dev` (Qt 6), mature, widely used system dependencies.

## Platform Support [6]
`rviz_viewport_control` targets the Tier 1 platforms of its ROS distributions (Ubuntu 24.04 Noble), verified in CI for `jazzy` and `kilted`.

## Security [7]

### Vulnerability Disclosure Policy [7.i]
A vulnerability disclosure policy is published in [`SECURITY.md`](../SECURITY.md), directing private reports to the maintainer.
