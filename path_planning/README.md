# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

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

# Model Generation

The path planner project exposes an important aspect of the Self Driving Car. The
responsibilities of this module is to generate the path from the source to the target
There are a couple of different parts to the path planning module. The main
components to these are
* Multiple trajectory Generation
* Evaluating the cost functions for these trajectories
* Implementing the best cost state.

I liked the approach from the Path Planning Walkthrough. I mean we control car’s
velocity instead of acceleration. If we do not increase speed more than 0.224 mph
in 0.02 sec time frame, our car will not suffer from longitudinal jerks. If we use
spline library to smooth our path, we will not suffer from latitudinal jerks.
This approximation is rough but sufficient for our toy track (actually, sometimes
I still able to see maximum jerk violation message).

To analyze and predict car’s behavior I work with frenet coordinates. It is not
easy to determine from Cartesian coordinates whether two cars are in the same lane.
It is better to use frenet coordinates for this. The idea behind frenet coordinates
is pretty simple. If we take the middle of the road as y axis and define d as a
distance from the y axis, we get frenet coordinates. Now to determine whether two
cars are in the same lane, we just need to compare d-value of frenet coordinates.

I used this approach to find out which lanes are occupied by which vehicles. The
advantage of this approach is that it lets us only compare for collision for cars
in a particular lane.

For each frame the overall algorithm used is described in the steps below:
* Initialize the position of the Ego Vehicle.
* Initialize the traffic on the road
  - This involves finding out which cars are in lanes 0 to 2 and initializing them in a map so that they can be queried easily.
* Check if the current car position is too close to the cars in the same lane.
* If it is not then increase the speed.
* If it is then generate trajectories to compute.
* If so look at the following cases:
  - Lane 0, generate trajectories for Lane 0 (by slowing) and Lane 1
  - Lane 1, generate trajectories for Lane 0, Lane 1 (by slowing) and Lane 2
  - Lane 2, generate trajectories for Lane 2 (by slowing) and Lane 1
* Trajectory Generation algorithm goes as follows:
  - Generate 5 points, 2 from the previous step and 3 points at 30m intervals
  - Now create a spline using these 5 points.
* Now compute cost for all the trajectories generated.
  - Not in middle lane cost
  - Too many cars in target lane
  - Less than optimum speed
  - Collision cost
* Once, all the costs are computed, we pick the lowest cost state.
  - However, if the costs are almost similar then the state preferred is to
    stay in the current lane and slow down.

## Trajectory Generation
If there are no points from the previous path, then take the 1 point from the previous
path, current position and 3 more points all at 30m apart.
We use the spline library to use these 5 points to generate a spline which is
then used for the ego vehicles path.

## Cost Functions:
### Not in middle lane cost
This cost is used for keeping the ego vehicle in the middle lane. Keeping the car
in the middle lane keeps options open to transition to the right and left lane which
is useful, especially when there are too many cars.

### Too many cars in target lane
Typically the ego vehicle should try to stay away from lanes which have too many cars
in the target lane. The reasoning for this is that even though it might have a lower cost
for the ego vehicle in the short term, it would eventually slow down the ego vehicle.
However, in the implementation of this cost function, I took into account all the vehicles
in the target lane which might not be correct since some may be behind the ego vehicle.

### Less than optimum speed
Driving at less than speed limit also imposes a higher cost since its not optimum
for the ego vehicle to be driving slower to its goal.

### Collision cost
This is the most important cost since its a safety feature. The collision cost is pretty
straight-forward to compute.
- For every car in the target lane.
  - Find the new position of the car assuming constant velocity
  - For every point on the trajectory
    - If the distance < SAFE DISTANCE
      - make that the best cost
