# Path Planning Project

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)
[![Build Status](https://semaphoreci.com/api/v1/sgalkin/carnd-t3p1/branches/master/shields_badge.svg)](https://semaphoreci.com/sgalkin/carnd-t3p1)
[![codecov](https://codecov.io/gh/sgalkin/CarND-T3P1/branch/master/graph/badge.svg)](https://codecov.io/gh/sgalkin/CarND-T3P1)
[![CircleCI](https://circleci.com/gh/sgalkin/CarND-T3P1.svg?style=svg)](https://circleci.com/gh/sgalkin/CarND-T3P1)

---

## Overview
This project implements path planning algorithm for Udacity self-driving car
simulator.

### Goals
In this project the goal is to safely navigate around a virtual highway with
other traffic. Using the car's localization and sensor fusion data. There is
also a sparse map list of waypoints around the highway.

Acceptance criteria:
  1. The car should try to go as close as possible to the 50 MPH speed limit,
     which means passing slower traffic when possible.
  2. The car should avoid hitting other cars at all cost.
  3. The car should avoid driving inside of the marked road lanes at all times,
     unless going from one lane to another.
  4. The car should be able to make one complete loop around the 6946m highway.
  5. The car should not experience total acceleration over 10 m/s^2
  6. the car should not experience jerk that is greater than 10 m/s^3

Other vehicles on the highway:    
  1. Are driving ±10 MPH of the 50 MPH speed limit.
  2. Might try to change lanes too.

## Demo

`./t3p1`

[Video](https://vimeo.com/260849645)

---

## Usage
```sh
Usage:
  t3p1 [options]
Available options:
  -w, --waypoints  File with route waypoints (defaults: data/highway_map.csv)
  -p, --port       Port to use (default: 4567)
  -h, --help       print this help screen
```

---

## Dependencies
### Runtime
* [Term 3 Simulator](https://github.com/udacity/self-driving-car-sim/releases)

### Tools
* `cmake` >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* `make` >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* `gcc/g++` >= 5.4, clang
  * Linux: gcc/g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

### Libraries not included into the project
* [`uWebSocketIO`](https://github.com/uWebSockets/uWebSockets) == v0.13.0
  * Ubuntu/Debian: the repository includes [`install-ubuntu.sh`](./scripts/install-ubuntu.sh) that can be used to set
    up and install `uWebSocketIO`
  * Mac: the repository includes [`install-mac.sh`](./scripts/install-mac.sh)
    that can be used to set up and install `uWebSocketIO`
  * Windows: use either Docker, VMware, or even [Windows 10 Bash on     Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/)

### Libraries included into the project
* [`Eigen`](http://eigen.tuxfamily.org/index.php?title=Main_Page) - C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms
* [`JSON for Modern C++`](https://github.com/nlohmann/json) - JSON parser
* [`Catch2`](https://github.com/catchorg/Catch2) - Unit-testing framework
* [`ProgramOptions.hxx`](https://github.com/Fytch/ProgramOptions.hxx) - Single-header program options parsing library for C++11
* [`nanoflann`](https://github.com/jlblancoc/nanoflann) - a C++11 header-only library for Nearest Neighbor (NN) search wih KD-trees
* [`spline.h`](http://kluge.in-chemnitz.de/opensource/spline) - Cubic Spline interpolation in C++

## Build
### Local manual build
0. Clone this repo.
1. `mkdir build`
2. `cd build`
3. `cmake .. -DCMAKE_BUILD_TYPE=Release -DLOCAL_BUILD=ON -DDOCKER_BUILD=OFF`
4. `make`
5. `make test`

### Pre-built Docker container
0. docker pull sgalkin/carnd-t3p1

### Manual build using containerized development environment
0. Clone this repo.
1. `mkdir build`
2. `cd build`
3. `cmake .. -DCMAKE_BUILD_TYPE=Release -DLOCAL_BUILD=OFF -DDOCKER_BUILD=ON`
4. `make docker-build`
5. `make docker-test`
6. `make docker-run` or `make docker-shell; ./t3p1`

---

## Map
Each waypoint in the list contains [_x_,_y_,_s_,_dx_,_dy_] values.
* _x_ and _y_ are the waypoint's map coordinate position
* _s_ value is the distance along the road to get to that waypoint in meters
* _dx_ and _dy_ values define the unit normal vector pointing outward of
  the highway loop

The highway's waypoints loop around so the frenet _s_ value,
distance along the road, goes from _0_ to _6945.554_.

![](docs/track.png)


## Protocol
The project uses `uWebSocketIO` request-response protocol in communicating with the simulator.

_INPUT_: values provided by the simulator to the c++ program
```json
{
  "x": "(float) - The car's x position in map coordinates",
  "y": "(float) - The car's y position in map coordinates",
  "yaw": "(float) - The car's speed in MPH",
  "speed": "(float) - The velocity of the vehicle (magnitude)",
  "s": "(float) - The car's s position in frenet coordinates",
  "d": "(float) - The car's d position in frenet coordinates",
  "previous_path_x": "(Array<float>) - The global x positions of the previous path, processed points removed",
  "previous_path_y": "(Array<float>) - The global y positions of the previous path, processed points removed",
  "end_path_s": "The previous list's last point's frenet s value",
  "end_path_d": "The previous list's last point's frenet d value",
   "sensor_fusion": [ "2D vector of cars and then that car's",
    [
      "<id> - car's unique ID",
      "<x> - car's x position in map coordinates",
      "<y> - car's y position in map coordinates",
      "<vx> - car's x velocity in m/s",
      "<vy> - car's y velocity in m/s",
      "<s> - car's s position in frenet coordinates",
      "<d> - car's d position in frenet coordinates",
    ]
  ]
}
```

_OUTPUT_: values provided by the c++ program to the simulator
```json
{
  "next_x": "(Array<float>) - The global x positions of the trajectory",
  "next_y": "(Array<float>) - The global y positions of the trajectory",
}
```

## Algorithm
### Constants
* Path planning horizon - _1 second_
* Points in path - _50_ (horizon / simulator tick)
* Hard speed limit - _50 MPH_
* Recommended speed - _99%_ of hard speed limit


* Comfort forward gap (distance to the obstacle in front) - _30 m_
* Comfort backward gap (distance to the obstacle behind) - _10 m_
* Minimal gap (safety limit) - _15 m_

### Fusion
* Position of each vehicle predicted for the end of current path (up to 1 second)
* Constant velocity model is used for prediction
* Closest car (if any) and its velocity associated to each lane

### Reference lane
* Cost function used in order to choose lane for the next iteration
* Safety check applied for each lane - no obstacles closer than _minimal gap_
  * the only possible action - keep the lane
* Main terms of the cost function
  * penalty for changing lanes (try to stay on lane)
  * penalty for using leftmost lane
  * forward gap size
  * velocity of the car in front (if any)
* Implicit FSM used in the project in order to protect the car during lane change

### Reference velocity
* Slow down if obstacle too close (closer than _minimal gap_)
* Follow the car in front with its velocity
* Speed up to the recommended velocity

### Trajectory generation
* At each step the application extends _previous path_ with new points,
  this guaranties smoothness of the path
* Cubic spline used in the project as a reference trajectory
* Velocity is constant for each step

### Control
* The car uses a perfect controller and will visit every (_x_, _y_) point
  it receives in the list every _0.02_ seconds
* The units for the (_x_, _y_) points are in meters
* The spacing of the points determines the speed of the car
* The vector going from a point to the next point in the list dictates the
  angle of the car.
* Acceleration both in the tangential and normal directions is measured along
  with the _jerk_ (the rate of change of total acceleration)

## TODO
1. Try to use more advanced cost function.
2. Increase test coverage.
3. Tune safety parameters.
4. Try to change not only to neighboring lane
