%%Oracle simulation for the DONUTS
%%this algorithm uses A* search to plan an optimal path to the goal

%%the current parameters will run a 10 module simulation, starting in a
%%cluster, in a clear environment

%change all paths to your file locations
addpath(genpath('path\DONUts\util'))
rmpath('path\DONUts\util\util-cen')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; close all;

I.ranConfig = 1; %changed based on your configuration
I.configName = num2str(I.ranConfig);
I.ranObs = 1;  %change based on your obstacle
I.obsName = num2str(I.ranObs);

%struct of constant parameters
I.numBots = 10; %Number of bots in the collective
I.numEPMs = 12; %Number of EPMs on each bot
I.numIR = 4; %Number of IR sensors on each bot

I.numObstacles = 3;%Number of obstacles that will populate the world
%The location to save video, images, and .mat files
I.pathname = 'path\';
I.readfileObs = 'path\Obstacles\obs';
I.readfileBots = 'path\Configurations\';
I.planname = strcat(I.configName,'.mp4');

I.numBotsX = 3; %Number of columns of bots
I.numBotsY = 4; %Number of rows of bots
I.config = 0; %0 for fixed intial configuration, 1 for random configuration, 2 for imported configuration
I.obs = 3; %1 = placed polygon obstacle, 2 = imported polygon obstacle, 3 = no obstacle, 4 = random obstacle
I.Converge = 0.1; %convergence criteria, 2 = 2R, 0.1 = 2R + some std, 0.2 = within a specified radius
I.heuristic = 7; %2 = Dmin, 2.5 Dcom A*, 7 Dcom oracle
I.alpha = 0; %increase to improve connection redundancy
I.debugOn = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Initialize Robots and obstacles
[bots,world,I]=Initialization(I);

%Optimal search algorithm
[win_state,visited,stack,childrenPerNode] = Astar(bots,world,I);

%Construct path
plan = Reconstruct(win_state,visited,I);

%Get test metrics
Metrics(plan,childrenPerNode,stack,visited,I);

%Animate path
Animate(plan,I);

close all;