This document is a declaration of software quality for the `viewport_control_msgs` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# viewport_control_msgs Quality Declaration

The package `viewport_control_msgs` claims to be in the **Quality Level 3** category.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the Package Requirements for Quality Level 3 in REP-2004.

## Version Policy [1]

### Version Scheme [1.i]
`viewport_control_msgs` uses [`semver`](https://semver.org/).

### Version Stability [1.ii]
`viewport_control_msgs` is at a version below `1.0.0`. A stable version is not required at Quality Level 3.

### Public API Declaration [1.iii]
The public API consists of the `ViewportView` message and the `ViewportCapture`, `ViewportSetProjection`, `ViewportSetReferenceFrame`, and `ViewportSetView` service definitions, documented field-by-field in their `.msg` and `.srv` files.

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
Changes affecting the public API are reflected in the `README.md` and in the message and service field comments.

## Documentation [3]

### Feature Documentation [3.i]
Features are documented in the repository [`README.md`](../README.md).

### Public API Documentation [3.ii]
The message and service contract is documented inline in the `.msg` and `.srv` field comments of `viewport_control_msgs`.

### License [3.iii]
The license is Apache-2.0, declared in `package.xml` and reproduced in [`LICENSE`](LICENSE).

### Copyright Statement [3.iv]
Copyright is held by `voshch`, stated in the appendix of [`LICENSE`](LICENSE).

## Testing [4]
Quality Level 3 imposes no testing requirement. A schema-guard gtest (`test/test_messages.cpp`) verifies that field renames or retypes fail to compile.

## Dependencies [5]

### Direct Runtime ROS Dependencies [5.i]
- `builtin_interfaces` (Quality Level 1)
- `geometry_msgs` (Quality Level 1)
- `sensor_msgs` (Quality Level 1)

### Direct Runtime non-ROS Dependencies [5.iii]
None.

## Platform Support [6]
`viewport_control_msgs` targets the Tier 1 platforms of its ROS distributions (Ubuntu 24.04 Noble), verified in CI for `jazzy` and `kilted`.

## Security [7]

### Vulnerability Disclosure Policy [7.i]
A vulnerability disclosure policy is published in [`SECURITY.md`](../SECURITY.md), directing private reports to the maintainer.
