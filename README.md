# slerp-animation
Second homework for Projective Geometry course at the Faculty of Mathematics in Belgrade, Serbia. Slerp interpolation of an object between two points animated in OpenGl, using C++ and Eigen library.

![animation](https://raw.githubusercontent.com/daviddimic/slerp-animation/master/screenshots/slerp-animation.gif)

## Isometry functions

Implemented functions for isometry:
- Euler2A
- A2AxisAngle
- Rodrigues
- A2Euler
- AxisAngle2Q
- Q2AxisAngle
- slerp

## Requirements

- C++11
- [OpenGL](https://www.opengl.org/)
- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)

## Usage
To compile program use `make`, and then `bin/slerp`.

|Key | Description |
|---|---|
|`ESC` or `q` | Exit |
|`space` | Start animation |
|`s` | Stop |
|`r` | Reset |
