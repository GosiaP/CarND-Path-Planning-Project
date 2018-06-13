# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image Referencengs)
[image1]: ./images/traffic_planner_screenshot.png


## Intoduction
The goal of this project is to navigate a car around a simulated road, including traffic - other cars driving on the same road side.
The road, ego car and other cars data are provided as waypoint data, telemetry and sensor fusion data respectively.

The implementation is based on the suggestions from the Q&A session and can be summarized in the following steps:
1. ego car parameters and traffic related cars are localized
2. information about all lanes on the road in driving direction is determined
3. behaviour planning - optimal target lane and target speed for ego car are detected
4. optimal trajectory for ego car is calculated using of the target lane and speed

## Implementation
The steps described above are done based on telemetry and sensor fusion data updated in constant time interval and provided by traffic planner implemented in TrafficPlaner class (`traffic_planner.h, cpp` files).
The TrafficPlanner provides a `getEgoCarPath()` method which provides a best trajectory for ego car. The trajectory is represented as list of global map coordinates.

### 1. Ego car and other car position
The telemetry and sensor fusion data are provided in json format, that are loaded in preprocessed. They represent current ego car position, its previous position inclusive a lane number the car is located. See `EgoCar`, `NonEgoCar` and `EgoCarLocalization` classes in `car.h, cpp` files.

### 2. Lanes on the road
To be able to find an optimal trajectory of ego car the data about each lane on a road are collected. The data contain, as suggested in the lessons, information about nearest car in front and back of ego car, its speed and the gap towards the ego car.
The implementation is done in `LaneInfo` and `LaneInfoOnRoad` classes in `road.h, cpp` files.

### 3. Behaviour planning
The behaviour planning takes a decision about optimal target lane and target speed of ego car. For that the planner keeps an internal state and data representing next steps that must be done - target lane and target speed of ego car.
The implementation is done in `BehaviourPlanner` class in 'traffic_planner.h, cpp files`.
Following internal states are available:
* initial
* keep lane
* prepare changing lane
* changing lane

The transition between states are possible at any time the planner is updated. The initial state is possible only one time on first update of telemetry and sensor fusion data.
The planner calculates best lanes for ego car and set is a target lane for next iteration. The optimal speed of is estimated and evaluated to avoid violation of speed limit.

For calculation of speed a safety margins for maximum speed are used. The length of gap between car in front and back of ego car are analyzed.
This is done in `BehaviourPlanner::findBestLane` method. The best lane is defined as a lane with maximum speed depending on current lane situation as represented by LaneInfoOnRoad. Adjusting of speed is implemented in `TrafficPlanner::adaptSpeed` method in `traffic_planner.h, cpp` files.

### 4. Optimal Trajectory
The target trajectory of ego car is calculated based on suggestions form lessons.
An information about current and next predicted locations are used to determine a discrete sequence of trajectory points - 3 next points in 30 meter space. They are interpolated to spline curve later to get smooth trajectory.

The interpolation is provided in `TrafficPlanner::interpolatePath` method using open source library in `spline.h` file. The optimal trajectory is calculated in `TrafficPlanner::createTrajectory` method and provided as list of points in global map coordinates.

## Reflection
The resulting traffic planner works well, but not perfectly. Sometime I got problems with violations of maximal jerk. Testing of the implementation required more time as testing if the ego car is driving at least 4.32 miles without incident requires some time.
Impelmetaion of the traffic planner was very interesting. I attached a screenshot and video from one of successful drives.

![Screenshot][image1]

[Video](https://github.com/GosiaP/CarND-Path-Planning-Project/blob/master/images/traffic_planner_video.mp4)



### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

