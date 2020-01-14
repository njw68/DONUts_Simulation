%%A*-search simulation for the DONUTS
%%this algorithm iterativley identifies the highest gradient in the
%%collective and plans a path for all modules to converge on that local
%%goal until the global goal is reaches

%%the current parameters will run a 10 module simulation, starting in an
%%imported configuration and imported obstacle-filled environment
warning off MATLAB:polyshape:repairedBySimplify
clear all; close all;

%1) File paths
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%adding/removing folders to the path
addpath(genpath('C:\path\DONUts\util'))
rmpath('C:\path\DONUts\util\util-oracle')

%The location to save videos, images, and .mat files
I.pathname = 'C:\path\DONUts\Data\trial1\';
%The locations to import saved configurations and obstacles
I.readfileObs = 'C:\path\DONUts\Data\Obstacles\';
I.readfileBots = 'C:\path\DONUts\Data\Configurations\';
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%2)Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%
I.numBots = 10; %Number of modules in the collective
I.numEPMs = 12; %Number of EPMs on each module
I.numIR = 4; %Number of IR sensors on each module

I.config = 2; %0 = fixed intial configuration, 1 = random configuration, 2 = imported, 3 = cluster start
I.obs = 2; %1 for generated obstacle, 2 for imported obstacle, 3 for no obstacles, 4 for random
%Note: the below parameters not relevant to the selected configuration and
%obstacle mode will be ignored

%For importing configurations/obstacles
I.ranConfig = 1; %the configuration number to be imported
I.ranObs = 4; %The obstacle number to be imported

%For a fixed/clustered initial configuration
I.numBotsX = 3; %Number of columns of modules
I.numBotsY = 4; %Number of rows of modules

%For obstacles
I.numObstacles = 2;%Number of obstacles that will populate the world
I.posobs=[9.5 0.5;5 9]; %x and y location/s of the generated obstacle/s

%For the A*-Search
I.alpha = 0; %increase to improve connection redundancy
I.heuristic = 2.5; %2 = Dmin, 2.5 = Dcom A*, 7 = Dcom oracle

%Convergence criteria,
I.Converge = 2; %2 = 2R, 0.1 = 2R + some std, 0.2 = within a specified radius
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%Initialize the modules and the world
[modules,world,I]=Initialization(I);

%%Run the above described algorithm and return the constructed path
modulePlan = CenA(modules,world,I);

%%Plots a graph and saves relavent data to a .mat file
I = Metrics(modulePlan,I);

%%Creates a video of the path generated
Animate(modulePlan,I);

close all;