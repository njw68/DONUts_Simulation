%%A* simulation for the DONUTS
%%this algorithm iterativley identifies the highest gradient in the
%%collective and plans a path for all modules to converge on that local
%%goal until the global goal is reaches

%%the current parameters will run a 10 module simulation, starting in a
%%cluster, in a clear environment

%change all paths to your file locations
addpath(genpath('path\Versions\DONUts\util'))
rmpath('path\DONUts\util\util-oracle')
clear all; close all;

I.ranConfig = 1; %changed based on your configuration
I.ranObs = 1; %change based on your obstacle
I.obsName = num2str(I.ranObs);
I.configName = num2str(I.ranConfig);

%The location to save video, images, and .mat files
I.pathname = 'path\';
%locations of saved obstacles and configurations
I.readfileObs = 'path\obs';
I.readfileBots = 'path\';
I.planname = strcat(I.configName,'.mp4');

%%Variables for operation
I.numBots = 10; %Number of bots in the collective
I.numEPMs = 12; %Number of EPMs on each bot
I.numIR = 4; %Number of IR sensors on each bot
I.numBotsX = 5; %Number of columns of bots
I.numBotsY = 1; %Number of rows of bots
I.config = 0; %0 for fixed intial configuration, 1 for random configuration, 2 for imported, 3 for cluster start
I.obs = 3; %1 for circle obstacle, 2 for imported polygon, 3 for no obstacles, 4 for random
I.numObstacles = 1;%Number of obstacles that will populate the world
I.alpha = 0; %increase to improve connection redundancy
I.Converge = 2; %convergence criteria, %convergence criteria, 2 = 2R, 0.1 = 2R + some std, 0.2 = within a specified radius
I.heuristic = 2.5; %2 = Dmin, 2.5 Dcom A*, 7 Dcom oracle
I.debugOn = 1;

%%Initialize the bots and the world
[bots,world,I]=Initialization(I);

%%Gradient following algorithm, this runs the above described algorithm and
%%returns the constructed plan to the goal
botPlan = CenA(bots,world,I);

%Plots a time graph and saves relavent data to a .mat file
I = Metrics(botPlan,I);

%Create a video of the path
Animate(botPlan,I);
close all;