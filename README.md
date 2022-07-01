# ME446_Robot_Dynamics_and_Control_lab
Implementation of different kinds of controllers on a Catalyst 5 robot arm. The repository contains the entire summary of the work along with videos and lab reports of the robot in action.

# Final Video Link
https://youtube.com/shorts/jAs9Y0I4wnY?feature=share

# Lab Website
http://coecsl.ece.illinois.edu/me446/

### Robot Dimensions

![image](https://user-images.githubusercontent.com/64373075/176832321-0cfb4d6f-ec28-4a15-86cf-231ac916317e.png)

### Averaging Filter

![image](https://user-images.githubusercontent.com/64373075/176832483-e46d1264-de24-475d-aebe-d929077161d3.png)

### PID + Feed Forward Control
![image](https://user-images.githubusercontent.com/64373075/176832584-7afdf22a-c91c-4376-8a9f-35734d985900.png)

### Response of System for butterfly trajectory

![image](https://user-images.githubusercontent.com/64373075/176832670-9e9718e3-85ec-4431-aa30-daf303fe3d77.png)

### Friction Compensation Curves for 3 joints

![image](https://user-images.githubusercontent.com/64373075/176832826-9ac49155-c3dd-4ddd-a331-9ed3135a44a0.png)

### Inverse Dynamics Controller to PD Controller

![image](https://user-images.githubusercontent.com/64373075/176833162-cfb04489-c8f2-4f17-af81-cd6fdee9a1f2.png)

![image](https://user-images.githubusercontent.com/64373075/176833220-990f83a4-8c41-4269-b989-b02fd85b9b79.png)

Implemented two control strategies namely Inverse Dynamics control and PD
control. Both controls had friction compensation which made them act as if there was no friction.
Both controllers were tuned to meet the desired requirements of a rise time less than 300ms and a
percent overshoot of less than 1% with minimal steady state error. Then to make the trajectory
smooth we also implemented a cubic trajectory with reference to the trajectory mentioned in the
report. From the plots, we can say that the parameters for with and without mass are reasonably
accurate. However, we think that for with mass situation the accuracy of calculating the dynamics
of the system can be further refined. Moreover, we can also improve on the friction modeling, but
unless we need super high accuracy, we need not go with refining the friction coefficients.
