# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Theory of PID Control

A PID controller is one of the most used controllers to regulate a processes in part due to its simple implementation and its name derives from its Proportional Integral Derivative parameters. The PID controller tried to minimize the error between a desired setpoint and the measured state. In this case, we are interested in the Cross Tracking Error (cte) which measures the difference from the vehicle to a reference trajectory.

## Proportional

The proportional factor provides the correction directly in relation to the current error and defined the Kp gain multiplied by the cte. This causes a correction that is directly proportional to the error such that a larger error results in higher output. In a proportional only controller, the vehicle will oscillate and will only reach marginal stability and constantly overshoot the desired trajectory.

## Integral

The integral factor uses accumulated error such that it corrects for offset usually due to system invariability. In other words, the I factor will look the continued error due to offset and correct the vehicle position to reach the desired trajectory

## Derivative

The derivative factor in the controller uses the previous error to account for correction of the vehicle. This means that it will become larger when an increasing rate of error is encountered, such as when the vehicle overshoots due to the proportional factor. It will therefore help to reduce the oscillations and reach stability faster.

## Approach

For this project, I used manual tuning to adjust the parameters knowing the relative effect of each of the factors in the PID loop. I started setting the P parameter to an arbitrary value of 0.5 and analyzing the effects of reducing and increasing this value. The goal was to get the vehicle to a somewhat stable state where it oscillates around the desired trajectory and maintains a good response to changes. For instance, setting it to value too small will not yield a fast enough response and would cause the vehicle to fall off the track during a turn. If set too large, the vehicle will rapidly overshoot and fall off the track. I ended up using 0.1 as it provided marginal stability while maintaining a good enough response.

I then set the D parameter to 0.5 and increased its value until oscillations were reduced. The final value I used was 1.8 which allowed the vehicle to remain on the track for several laps. I then played around with the I parameter which did not seem to play a large role in maintaining the stability of the vehicle probably due to the fact that we are dealing with a simulation and not a real system in which it would be effective. I use 0.004 as it provided good stability of the vehicle in conjunction with the other two parameters.

The vehicle is able to navigate the track and go through the different turns. While there is still some oscillations present, the PID controller provides good enough stability for the vehicle. Further improvements of the project could be made by adding a second controller for speed but will leave it for a future time. Overall, this project was great to understand the mechanics of PID controllers and how each factor plays a role in creating a stable system.


