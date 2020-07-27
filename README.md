# DONUts_Simulation
This is a simulation framework to support the Deformable self-Organizing Nomadic Units (DONUts) modular robot,
developed in the Collective Embodied Intelligence Lab at Cornell University. The paper is available here: https://doi.org/10.3389/frobt.2020.00044 Contact: njw68@cornell.edu with
questions.

This is a simulation framework to support coordination of the DONUts modular robot. The DONUts-Hardware folder contains
schematic files for the DONUts hardware.
Included are algorithms to support basic functionalities of the DONUts such as communication, sensing and movement.
There are 4 different centralized coordination strategies:
1) oracle: an optimal, A* search-based planner, with a priori knowledge of the environment
2) A-star: an A* search-based local planner without a priori knowledge of the environment
3) faf1: a 'farthest first' algorithm that climbs a local gradient
4) faf0: a more dynamic 'farthest first' algorithm that climbs a local gradient.

INSTRUCTIONS:

Operation is done from the 'oracle', 'faf', or 'a-star' folder's Main.m file.

Sample configurations and obstacles can be imported from the 'Configurations' and 'Obstacles' folders respectively.

All 'Main.m' files are initially set to import a 10 module configuration and manuver through 3 obstacles.

Data is saved in the 'Data\trial1' folder.

All 'Main.m' files are configured to run various simulations with a few adjustments:

1) change all file locations to reflect your personal directories
2) change the desired parameters to set up your simulation environment

These steps are numbered in each 'Main.m' file. There are more detailed instructions for each step there.

