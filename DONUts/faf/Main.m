%%FAF rigid simulation for the DONUTS
%%This algorithm moves the bot with the highest gradient forward

%%the current parameters will run a 10 module simulation, starting in a
%%cluster, in a clear environment

%change all paths to your file locations
warning off MATLAB:polyshape:repairedBySimplify
addpath(genpath('path\DONUts\util'))
rmpath('path\DONUts\util\util-oracle')
clear all; close all;

I.ranConfig = 1;
I.ranObs = 1;
I.obsName = num2str(I.ranObs);
I.configName = num2str(I.ranConfig);

%struct of constant parameters
I.numBots = 10; %Number of bots in the collective
I.numEPMs = 12; %Number of EPMs on each bot
I.numIR = 4; %Number of IR sensors on each bot
I.numObstacles = 3;%Number of obstacles that will populate the world

%The location to save video, images, and .mat files
I.pathname = 'path\'; %file save location
I.readfileObs = 'path\obs';
I.readfileBots = 'path\';
I.planname = strcat(I.configName,'.mp4');
% I.posobs=[9.5 0.5]; %x and y location of the imported obstacle/s
I.numBotsX = 3; %Number of columns of bots
I.numBotsY = 4; %Number of rows of bots
I.config = 0; %0 for fixed intial configuration, 1 for random configuration, 2 for imported, 3 for cluster start
I.obs = 2; %1 for circle obstacle, 2 for imported polygon, 3 for no obstacles, 4 for random
I.Converge = 2; %convergence criteria, 2 = 2R, 0.1 = 2R + some std, 0.2 = within a specified radius

%%Initialize the bots and the world
[bots,world,I]=Initialization(I);

%%Gradient following algorithm, this runs the above described algorithm and
%%returns the constructed path to the goal
botPlan = Faf0(bots,world,I); %can change to faf0

%Plots a time graph and saves relavent data to a .mat file
I = Metrics(botPlan,I);

Animate(botPlan,I);

close all;