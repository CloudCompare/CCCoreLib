# CCCoreLib

[![Actions Status](https://github.com/CloudCompare/CCCoreLib/workflows/Build/badge.svg)](https://github.com/CloudCompare/CCCoreLib/actions)

This library provides data structures and algorithms for working with 3D point cloud data.

It was originally part of the [CloudCompare repository](https://github.com/CloudCompare/CloudCompare) (as "CCLib").

We have separated it into its own repository because it is useful on its own without having to download the entire CloudCompare repository. This makes it easier to work with and gives this part of the CloudCompare project some visibility it would not otherwise have.

It uses CMake, requires C++14, and compiles & runs on Linux, macOS, and Windows.

## Main CMake Options

| Option        | Description | Default  |
| ------------- |-------------| ---------|
| CC-CORE-LIB_USE_CGAL | Use CGAL to enable Delaunay 2.5D triangulation with a GPL compliant licence | OFF |
| CC-CORE-LIB_USE_TBB | Use Intel Threading Building Blocks lib to enable some parallel processing | OFF |
| CC-CORE-LIB_USE_QT | Use Qt to enable parallel processing using QtConcurrent | ON |
| CC-CORE-LIB_SHARED | Compile as a shared library | ON |
| CC-CORE-LIB_SCALAR_DOUBLE | Define _ScalarType_ as double (instead of float) | OFF |

### Qt Option (Qt5_DIR)

If `CC-CORE-LIB_USE_QT` is on (it is by default), then you may need to set `Qt5_DIR` to point to your Qt inatallation directory. This is the directory containing the `bin`, `doc`, `include`, `lib`, [etc.] directories.

From this, cmake will determine `Qt5Concurrent_DIR`, `Qt5Core_DIR`, and `Qt5Widgets_DIR`, so there's no need to set those explicitly.

## Things We Have Yet To Do

- contribution guidelines (including coding style)
- documentation

## How You Can Help

- [report issues](https://github.com/CloudCompare/CCCoreLib/issues)
- help with documentation
- [contribute improvements](https://github.com/CloudCompare/CCCoreLib/pulls)

## License
This project as a whole is licensed under the **LGPL 2.0+** license - see the [LICENSE](LICENSE.txt) file for details.

Individual source files contain the following tag instead of the full license text:

	SPDX-License-Identifier: LGPL-2.0-or-later

The CMake files are licensed under the **MIT** license - see the [LICENSE-MIT](LICENSE-MIT.txt) file for details.

These files contain the following tag instead of the full license text:

	SPDX-License-Identifier: MIT

Using SPDX enables machine processing of license information based on the [SPDX License Identifiers](https://spdx.org/ids) and makes it easier for developers to see at a glance which license they are dealing with.

### License Special Note
Two files (BoundingBox.cpp and RayAndBox.h) were previously licensed under the **GPL 2.0+** license by mistake - see the [LICENSE-GPL-2.0](LICENSE-GPL-2.0.txt) file for details.

These files contain the following tag instead of the full license text:

	SPDX-License-Identifier: GPL-2.0-or-later

We are working on securing the necessary permissions to relicense these under LGPL.
