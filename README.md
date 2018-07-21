# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program
PID Controller Project

### Project Overview
This is a project of the [Self-Driving Car Engineer Nanodegree](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013) Term 2. <br>

The main goal of the project is to build a PID controller in C++ and tune the PID hyperparameters by applying the general processing flow as described in the Udacity lessons. This project requires to test the implemented solution by racing the self driving car around the lake track in a Simulator.


### PID Controller
A proportional–integral–derivative controller (PID controller or three term controller) is a control loop feedback mechanism widely used in industrial control systems and a variety of other applications requiring continuously modulated control. A PID controller continuously calculates an error value e(t) as the difference between a desired setpoint (SP) and a measured process variable (PV) and applies a correction based on proportional, integral, and derivative terms (denoted P, I, and D respectively) which give the controller its name.

Proportional Coefficient
The proportional component (P) output is proportional to the CTE(Cross Track Error). It sets the steering angle which causes the car to steer proportional (and opposite) to the car’s distance from the lane center - if the car is far to the right it steers hard to the left, if it’s slightly to the left it steers slightly to the right. Thus it is able to steer in the correct direction.


Integral Coefficient
Integral component (I) takes into account the integral of CTE over the past time. It is used to reduce systematic bias. An integral term increases action in relation not only to the error but also the time for which it has persisted. So, if applied force is not enough to bring the error to zero, this force will be increased as time passes. 

Derivative Coefficient
A derivative term considers the rate of change of error (its derivative), trying to bring this rate to zero. It is used to reduce overshooting and damp oscillations to remain the system at a constant magnitude. Thus to keep the system marginally stable.

### Hyperparameters - Finding initial value for Kp, Ki, Kd
I first tuned initial Kp, Ki and Kd values manually. Then I implemented Twiddle algorithm to test how it works.

In my implementation, I updated the main.cpp file main() function to main(int argc, char * argv[]). Thus I can pass Kp, Ki, Kd parameter values. If I want to use twiddle algorithm, I can enter the 'twiddle' as the fourth parameter.

From my experiments, these (Kp, Ki, Kd) values produced good performances at racing the car around the lake track:
* Non-twiddle: (0.225 0 3.0)
* Twiddle: (0.13 0.0002 3.05)

Racing record of videos in the simulator with twiddle algorithm and non-twiddle are provided in the submitted video/ directory.

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it with PID coefficient values: `./pid [Kp] [Ki] [Kd]`, such as: `./pid 0.25 0.0 0.005` will run with the given initialized parameter values. 
5. Or run it with PID coefficient values and twiddle algorithm: `./pid [Kp] [Ki] [Kd] twiddle`, such as: `./pid 0.25 0.0 0.005 twiddle` will take the given parameter values to run with twiddle algorithm. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
