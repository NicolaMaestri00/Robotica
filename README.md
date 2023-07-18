# Introduction to Robotics (UniTn)

## Laboratories

## Final Assignment
Handing a microphone to random people that want to ask questions after a talk, can be a duly task, 
and we want to automatize it. The assignment for the final project is to design a giraffe robot that 
is able to place a microphone in front of a person in a small theatre/conference room. 
The robot is located in the middle of the room and attached to the ceiling. The room is 4 m high 
and the robot should be able to reach 1 m high locations in a 5x12 meters area.
The robot should have 5 degrees of freedom: a spherical joint at the base (2 revolute joints with 
intersecting axes), one prismatic joint that is able to achieve a long extension and 2 revolute joints 
to properly orient the microphone (not necessarily with intersecting axes). 
We want to be able to locate the microphone at any point in the 5x5 conference room, with a certain
pitch orientation (30 deg) with respect to the horizontal (the task is 4D), to allow people to talk 
comfortably in the microphone. 

The project can be approached through the following incremental steps:
1. Construct the URDF model of the robot, selecting appropriate link lengths and
arranging frames suitably.
2. Compute the forward kinematics (position/orientation) and differential kinematics
(Jacobian) of the end-effector.
3. Use Pinocchio library’s RNEA native function to create a simulator of the motion.
4. Plan a polynomial trajectory (in the task space) to move from a coming configuration
qhome to a given end-effector configuration-orientation pdes + Θdes.
5. Write an inverse-dynamics (computed torque) control action in the task space to
linearize the system and achieve tracking of the task.
6. Set the PD gains of the Cartesian controller implemented on the linearized system
to achieve a settling time of 7s without overshoot.
1
Final Assignment 2
7. In the null-space of the task minimize the distance with respect to a given configuration q0 of your choice.
8. Simulate the robot to reach the location pdes = [1, 2, 1] from the homing configuration
qhome = [0, 0, 0, 0].
