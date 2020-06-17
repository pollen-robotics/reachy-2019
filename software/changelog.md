# Changelog 

## Version 1.1.0

* Add support for orbita board
* Add support for sensors in WsIO (eg. for Unity simulation)
* Add support for dual camera (left or right), only one can be active at the same time.
* Add support for fans in the arm.
* Support for pyluos 1.2.2

### Bugfix 

* fix id on left hands
* fix temperature in ws
* fix bug in discovery when used with a single gate connected
* fix potential bug when switching camera
* fix bounds in IK.

## Version 1.0.0

### Bugfix

* remove Hands from parts (avoid duplicate motors in reachy.motors)
* Adjust parts length in kinematic definition to match new version.