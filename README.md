# DONUts_Simulation
This is a simulation framework to support the Deformable self-Organizing Nomadic Units (DONUts) modular robot,
developed in the Collective Embodied Intelligence Lab at Cornell University. Contact: njw68@cornell.edu with
questions.

This is a simulation framework to support coordination of the DONUts modular robot.
This includes algorithms to support basic functionalities of the DONUts such as communication, sensing and movement.
There are 4 different centralized coordination strategies:
1) an optimal A*-based oracle, all-seeing planner, with a priori knowledge of the environment
2) an A*-based local planner without a priori knowledge of the environment
3) a 'farthest first' algorithm that climbs a local gradient, and
4) a more dynamic 'farthest first' algorithm that climbs a gradient.

INSTRUCTIONS:

Operation is done from the 'oracle', 'faf', or 'a-star' folder's Main.m file.

Sample configurations and obstacles can be imported from the 'Configurations' and 'Obstacles' folders respectively.

All 'Main.m' files are initially set to run import a 10 module configuration and a 3 obstacle course.

Data is saved in the 'Data\trial1' folder.

All 'Main.m' files are configured to run various simulations with a few adjustments:

1) change all file locations to reflect your personal directories
2) change the desired parameters to set up your simulation environment
