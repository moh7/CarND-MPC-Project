# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program
---
## Overview
This project is the tenth [task](https://github.com/udacity/CarND-MPC-Project) of the Udacity's Self Driving Car Nanodegree program. The task in this project was to design a model predictive control system to drive an autonomous car around the [simulator](https://github.com/udacity/self-driving-car-sim/releases) track. The MPC receives the information from the simulator using a uWebSockets implementation and computes the optimized actuation inputs and sends it back to the simulator.
## MPC Algorithm

### Dynamic Model of the Car
The kinematic model of the car is described by the following equations. The interaction of tire and the road is neglected in these equations.
```
x[t]    =  x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y[t]    =  y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi[t]  =  psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
v[t]    =  v[t-1] + a[t-1] * dt
cte[t]  =  f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] =  psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
```
where:

* ```x, y``` : vehicles's position
* ```psi``` : vehicles's heading direction (orientation)
* ```v``` : vehicles's velocity
* ```cte``` : cross-track error (distance of the vehicle's center line from the middle of the lane)
* ```epsi``` : orientation error

are the states of the system. ```dt``` is the time step, and ```Lf``` corresponds to the distance between the mass center of the car and its front axle. Additionally,
* ```a```: throttle (acceleration)
* ```delta``` : steering angle

are the actuator inputs which are computed by the MPC controller.

### Inside main.cpp
The MPC starts in ```main.cpp``` by taking in the following values from the simulator.

* ```ptsx``` (x-position of waypoints ahead on the track in global coordinates)
* ```ptsy``` (y-position of waypoints ahead on the track in global coordinates)
* ```px``` (current x-position of the vehicle's position in global coordinates)
* ```py``` (current y-position of the vehicle's position in global coordinates)
* ```psi``` (current orientation angle of the vehicle)
* ```v``` (current velocity of the vehicle)
* ```steering_angle``` (current steering angle)
* ```throttle``` (current throttle)

To simplify the computations, everything is first transformed from the global coordinate system to the vehicle's coordinate system. First, the waypoint vectors ```ptsx``` and ```ptsy``` are transformed  to the vehicle's coordinate system using the transformation matrix. The new waypoint vectors are called ```x_vehicle``` and ```y_vehicle```. These two vectors essentially define the path that the vehicle should follow along the defined time horizon.

Next, a three order polynomial is fitted to these waypoints using ```polyfit()```. Since the path is transformed to the car's coordinate system, the current position and orientation of the car ```px```, ```py``` and ```psi``` are all zeros in the car's coordinate system. The cross-track error ```cte``` is then calculated by evaluating the polynomial function at the current position of the car using ```polyeval()```.
The current orientation error ```epsi``` is also computed by taking the derivative of the fitted line at the current position.

### Accounting for latency
To account for latency, an additional step was added to predict the state of the system after 100 milliseconds of delay. This predicted state is then fed to the controller. The state of the system after this 100 milliseconds delay can be predicted using the mathematical model of the system and the current state values.

Finally, the state of the system along with the polynomial coefficients are fed to the MPC controller. The optimized control outputs (steering angle and throttle values) are computed by the controller ```mpc.solve()```  and are sent back to the simulator to keep the car on the desired path

### Inside MPC.cpp

The objective of the controller is to minimize a cost function that depends on different factors including:
* Sum of square values of ```cte``` and ```epsi``` to minimize cross-track and orientation errors.
* Sum of square values of ```(v - v_ref)``` to minimize the difference of the speed with the reference speed.
* Sum of square of actuator values ```a``` and ```delta``` to penalize large actuator actions.
* Sum of square values of the difference between two consecutive actuator values to penalize sharp changes.

I used weight parameters to prioritize the importance of each factor in the cost function. The appropriate weight values were obtained by the try-and-error method. It was noticed that the weights corresponding to the steering angle input and its rate have the most significant impact on the performance of the system and choosing large values for these two weights (```W_DELTA``` and ```W_DDELTA```) helps improving the stability of the car and avoiding the erratic and sudden steering behavior. Using the following weight values, a smooth and safe behavior can be obtained.   
```
#define W_CTE 2
#define W_EPSI 1
#define W_V 1
#define W_DELTA 3000
#define W_A 1
#define W_DDELTA 2000
#define W_DA 1
```
### Timestep Length and Elapsed Duration (N & dt):
The parameters ```N```(the number of points) and ```dt``` define the prediction horizon. Choosing a long prediction horizon can theoretically improve the prediction; but in practice, it increases the computational complexity. With too many points, the controller becomes slower and can become unstable. After some try and error and experimenting different values, I found that the car behaves well with ```N = 10``` and ```dt = 0.1``` which corresponds to a 1-second time horizon.  


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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
