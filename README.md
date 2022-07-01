# lab_hw
 # SoccerRobot
 #Afshin Tavakolikia 9823022
***

in this homework we design a pid compensator to acheive desirable conditions of any mission.


### PID Compensator
First of all we defined a class, named PID. we calculate the P and D  by dividing the error by the Delta_time in code and the I by errors summation. The output of the P controller is proportional to the input error by a KP gain, so there is no overshoot, reduced rise time, and SS (Steady State) error. A PI controller uses an integrator to eliminate steady-state error but increases overshoot and settling time. A PD controller uses a derivative block to reduce settling time and process overhead. A PID controller is a combination of all the controllers described above and can be used to achieve a critical damping response with minimum final error. 

### Deadzone
Deadzones causes the process error to become larger (the motor does not respond until the input gets to a specific value). for example if the deadzone bound for our DC motor is 0.5 and we use a P contorller with kp=10, the errors lower than 0.05 correspond to a value lowers than 0.5 for motor input.
## M1(A to B one direction)
### Code Functionality
***
In this code, we set a y postion to move robot to that position without changing x position. 
## M2(Angle control)
### Code Functionality
***
In this code we use 'self.get_compass_heading()' to get the heading position of robot and we control robot to rotate by controlling each wheel speed (direction of wheel rotation must be opposite of each other).
## M3(A to B any direction)
### Code Functionality
***
In this code we use two previous pid for controlling the distance to desire postion and angle if we need to move two direction. 
## M4(chasing ball)
### Code Functionality
***
In this code the robot chasing the ball and always faces to gate side. for this section we must control robot angle and position due to ball position and achieve a very short distance between ball and robot.
