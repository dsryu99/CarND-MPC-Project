# **Model Predictive Control (MPC)**
---

**Model Predictive Control (MPC) Project**

The goals / steps of this project are the following:
* Implement Model Predictive Control to drive te car around the track.
* Not given the cross track error, have to calculate that yourself.
* 100 milisecond latency between actuations commands on top of the connection latency.

## Rubric Points
###Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/896/view) individually and describe how I addressed each point in my implementation.  

---
### Compilation

#### 1. Your code should compile.

My code compiles without errors with `cmake` and `make`.

### Implementation

#### 1. The Model
In the model, the following states are used.

|state        | description|
|:-----------:|:--------------------------------------------:|
|x (float)    | The global x position of the vehicle. |
|y (float)    | The global y position of the vehicle. |
|psi (float)  | The orientation of the vehicle in radians. |
|speed (float)| The current velocity in mph. |
|cte (float)  | Cross Track Error. The error between the center of the road and the vehicle's position. |
|epsi (float) | The orientation error. It is the desired orientation subtracted from the current orientation. |


Two actuators are as follows:

| actuator    | description |
|:-----------:|:-----------:|
|delta (float)| The steering angle in radians. |
|a (float)    | The throttle value [-1, 1]. Acceleration both positive or negative. |


The next states are predicted in the following ways.
- x_t+1 = x_t + v_t x cos(psi_t) x dt
- y_t+1 = y_t + v_t x sin(psi_t) x dt
- psi_t+1 = psi_t + v_t / L_f x delta_t x dt
- v_t+1 = v_t + a_t x dt
- cte_t+1 = f(x_t) - y_t + v_t x sin(epsi_t) x dt
- epsi_t+1 = psi_t - psi_des_t + v_t / L_f x delta_t x dt


#### 2. Timestep Length and Elapsed Duration (N & dt)

The prediction horizon is the duration over which future predictions are made. We refer to this as T which is the product of two other variables, N and dt. N is the number of timesteps in the horizon. dt is how much time elapses between actuations. I tuned the two parameters with different settings. Because N determines the number of variables optimized by the Model Predictive Control (MPC), I started tuning it with the value not larger than 30.

|case|description|
|:-:|:-:|
|(N=20, dt=0.1)| The vehicle leaves the track shortly after it starts. Large values of dt result in less frequent actuations. It is hard to accurately approximate a continuous reference trajectory. This is an discretization error. |
|(N=30, dt=0.01)| The vehicle overshoots sharply from side to side in a short time interval. |
|(N=30, dt=0.05)| The trajectory is incorrectly predicted on the way that the vehicle turns rapidly. |
|(N=25, dt=0.05)| The trajectory is incorrectly predicted on the way that the vehicle turns rapidly. |
|(N=20, dt=0.05)| The trajectory is correctly predicted in the entire track. |
My final choice is (N=20, dt=0.05).

#### 3. Polynomial Fitting and MPC Preprocessing
A polynomial is fitted to waypoints. Before polynomial fitting, waypoints are preprocessed. The input waypoints are from the map's coordinate system. Thus, I transformed the map's coordinate system to the car's coordinate system using translation and rotation.
- ptsx (Array) : the global x positions of the waypoints
- ptsy (Array) : the global y positions of the waypoints
- px (float) : the global x position of the vehicle
- py (float) : the global y position of the vehicle
- dx = ptsx - px
- dy = ptsy - py
- x' = dx x cos(psi) + dy x sin(psi)
- y' = dy x cos(psi) - dy x sin(psi)

#### 4. Model Predictive Control with latency

In this project, Model Predictive Control needs to handle a 100 millisecond latency. In order to deal with this issue, new states after a 100 millisecond latency need to be predicted. To do that, I used the update function described in the section 1 above. In this case, because we transformed the waypoints into the car's coordinate system, the current position x, y and psi are all zeros.

### Simulation

#### 1. The vehicle must successfully drive a lap around the track.

The vehicle drives around the track safely.
