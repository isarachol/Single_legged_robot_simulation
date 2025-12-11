# Single-Legged Robot Simulation

This project is part of the ME500A2 Dynamics modeling class at BU. In this project, I designed a 5-DOF manipulator, derived the Lagrangian dynamical model, and simulated the motions in MATLAB under different conditions (with/without gravity, with/without controller)

Legged robots are more robust at traversing through uneven terrains and are more flexible conpared to wheed robots or drones. However, there exists challenging modeling and control strategies due to the hybrid dynamics of its motion when interacting with the environment, such as jumping. To simplify the problem, I decided to simulate a 5-DOF manipulator instead.

The main simulation files are in the `simulation/` directory. The files are
- sim_static.m: Simulate kinematics of the robot given a specific configuration
- sim_dynamic_5joint_control.m: Simulate dynamic motions of the robot under PD+ control.
