# Changelog

## v1.3.0 - 2020-11-10

### Added

- Add multi-cast support
- Add saved_by_rows argument
- Add different point types( XYZI & XYZIRT)

### Changed

- Update driver core, please refer to CHANGELOG in rs_driver for details
- Update some documents
- Change angle_path argument to hiding parameter

### Removed

- Remove RSAUTO for lidar type
- Remove device_ip argument



## v1.2.1 - 2020-09-04

### Fixed

- Fix bug in driver core, please refer to changelog in rs_driver for details.


## v1.2.0 - 2020-09-01

### Added
- Add camera software trigger (base on target angle)

### Changed
- Update driver core, please refer to changelog in rs_driver for details
- Update the compiler version from C++11 to C++14


## v1.1.0 - 2020-07-01

### Added

- Add ROS2 support

### Changed
- Replace while loop with cv.wait
- Update the vector copy part 
- Update the program structure

### Removed
- Remove some unused variables in message struct

## v1.0.0 - 2020-06-01

### Added

- New program structure

- Support ROS & Protobuf-UDP functions

  