# Introduction to Robotics (UniTn)

## Laboratories
### Lab 1
- Learning the basic procedure to visualize a robot model using the Unified Robot Description Format (URDF)
- Compute and visualize the direct/inverse kinematics of a 4-DoF serial manipulator
- Compute and analyze the forward/inverse dynamics of a 4-DoF serial manipulator using the Recursive Newton-Euler Algorithm (RNEA)
### Lab 2-3
- Learning the basic procedure to design a motion controller in the joint space for a manipulator in free-motion (i.e. not in contact)
- Analyze the advantages/disadvantages of decentralized/ centralized approaches (i.e. feedback linearization)
- Implement the interaction with the environment with a compliant contact mode
### Lab 4-5-6
- Acquire confidence in some invariant properties of floating base dynamics.
- Contact consistent (fixed) base dynamics
- Floating base robot: quasi-static control of locomotion stability
### Lab 7
- Learning the basic procedure to design an admittance controller for the end-effector of a manipulator in contact with the environment with the purpose to control the interaction with a human.
- Implement an obstacle avoidance planning algorithm base on potential fields.

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
4. Plan a polynomial trajectory (in the task space) to move from a coming configuration $q_{home}$ to a given end-effector configuration-orientation $p_{des}+ \Theta_{des}$.
5. Write an inverse-dynamics (computed torque) control action in the task space to linearize the system and achieve tracking of the task.
6. Set the PD gains of the Cartesian controller implemented on the linearized system to achieve a settling time of 7s without overshoot.
7. In the null space of the task minimize the distance with respect to a given configuration q0 of your choice.
8. Simulate the robot to reach the location $p_{des} = [1, 2, 1]$ from the homing configuration $q_{home}= [0, 0, 0, 0]$.
