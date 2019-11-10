%Initialization
%%Takes in the variables set by the user and returns the world and bots

function [bots,world,I] = Initialization(I)
I.testing = 0;
I.goalBot = [];
I.test = 0;
I.l = 0;
I.c = 0.7;
I.IRSensorData = xlsread('IRSensorData',2,'B2:L38');
I.tempgoal = 0;
I.goal = [0 0];
I.plotXLimit = I.goal(1)+7.5; % creates a variable to set the x-axis limit
I.plotYLimit = I.goal(2)+7.5; % creates a variable to set the y-axis limit

coordinates = zeros(1,2); %x y coordinates of the center of the bot
%Stores EPM IDs, positions relative to the bot, x,y coordinates, connecting
%bots, and possible transitions allowed at each EPM
EPMs = zeros(5,I.numEPMs);
%Stores IR sensor IDs, positoins relative to the bot, x,y coordinates, and
%if obstacles are detected
IRsensors = zeros(4,I.numIR); 

%%Creates lines to plot the circular bots
%%Number of sides to plot circular bots (increasing this number produces
%smooother circles)
I.numSides = 104;
I.t1=((1/(2*I.numSides):1/I.numSides:((2*I.numSides) + 1)/(2*I.numSides))'*2*pi);

%%The position of each EPM with respect to the inertial frame of
%%reference. Changes as the bots move.
for i=1:I.numEPMs
    EPMs(1,i) = i;
end

%%The position of each IR sensor with respect to the inertial frame of
%%reference. Changes as the bots move
for i = 1:I.numIR
    IRsensors(1,i) = i;
end

%Creates a struct for all of the bots and their properties
bot.geometry = [];
bot.coordinates = coordinates;
bot.EPMs = EPMs;
bot.IRsensors = IRsensors;

%creates struct of bots
bots = bot;
for i=2:I.numBots
    bots(end+1) = bot;
end

%Creates the geometry for bot1
bots(1).geometry = cos(I.t1);
bots(1).geometry = [bots(1).geometry,(sin(I.t1))];
%Determines the coordinates of the center of bot1
bots(1).coordinates(1) = mean(bots(1).geometry(:,1));
bots(1).coordinates(2)  = mean(bots(1).geometry(:,2));
%Calculates the bot diameter
I.botDiam = sqrt(((bots(1).coordinates(1,1) - bots(1).geometry(1,1))^2) + ...
    ((bots(1).coordinates(1,2) - bots(1).geometry(1,2))^2))*2;
if I.config==0
    %Creates the fixed initial configuration and environment
    [bots,I] = FixedBotConfig(bots,I);
    for i=1:I.numBots
        for j=1:I.numEPMs
            if bots(i).EPMs(4,j) > I.numBots
                bots(i).EPMs(4,j) = 0;
                bots(i).EPMs(5,j) = 0;
            end
        end
        x(i) = bots(i).coordinates(1);
        y(i) = bots(i).coordinates(2);
    end
    
    I.startCOMX = round(mean(x),4); I.startCOMY = round(mean(y),4);
    I.setAxis = [I.startCOMX-(I.numBots*I.botDiam)/2 I.goal(1)+(I.numBots*I.botDiam)/2 I.startCOMY-(I.numBots*I.botDiam)/2 I.goal(2)+(I.numBots*I.botDiam)/2];
elseif I.config==1
    %Creates the random initial configuration
    [bots,I] = RandomBotInitialConfig(bots,I);
    for i=1:I.numBots
        for j=1:I.numEPMs
            if bots(i).EPMs(4,j) > I.numBots
                bots(i).EPMs(4,j) = 0;
                bots(i).EPMs(5,j) = 0;
            end
        end
        x(i) = bots(i).coordinates(1);
        y(i) = bots(i).coordinates(2);
    end
    I.startCOMX = round(mean(x),4); I.startCOMY = round(mean(y),4);
    I.goal(1) = round(I.startCOMX + (I.numBots*I.botDiam)*cosd(I.theta),4);
    I.goal(2) = round(I.startCOMY + (I.numBots*I.botDiam)*sind(I.theta),4);
    world.goal = rectangle('Position',[(I.goal(1)-.2) (I.goal(2)-.2) .55 .55],'FaceColor',[0 1 0]);
elseif I.config==2
    matfile = strcat(I.readfileBots,I.configName,'.mat');
    load(matfile,'bots');
    x = zeros(I.numBots,1); y = zeros(I.numBots,1);
    for i=1:I.numBots
        for j=1:I.numEPMs
            if bots(i).EPMs(4,j) > I.numBots
                bots(i).EPMs(4,j) = 0;
                bots(i).EPMs(5,j) = 0;
            end
        end
        x(i) = bots(i).coordinates(1);
        y(i) = bots(i).coordinates(2);
    end

    I.startCOMX = round(mean(x),4); I.startCOMY = round(mean(y),4);
    I.goal(1) = round(I.startCOMX + (I.numBots*I.botDiam)/sqrt(2),4);
    I.goal(2) = round(I.startCOMY + (I.numBots*I.botDiam)/sqrt(2),4);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    I.EPMMarks = (0:2*pi/I.numEPMs:2*pi*(I.numEPMs-1)/I.numEPMs);
    %%The angles where a blue dot will be plotted on each circle to represent
    %%the location of the IR sensors
    I.IRMarks = (2*pi/I.numEPMs/2:2*pi/I.numIR:2*pi*(1 - (1/I.numEPMs/2)));
    for currentBot = I.numBots
        [bots,I] = SensorEPMPositions(bots,I,currentBot);
    end
    I.setAxis = [I.startCOMX-(I.numBots*I.botDiam)/2 I.goal(1)+(I.numBots*I.botDiam)/2 I.startCOMY-(I.numBots*I.botDiam)/2 I.goal(2)+(I.numBots*I.botDiam)/2];

elseif I.config == 3
    [bots,I] = FixedBotConfig(bots,I);
    bots = bots(1:I.numBots);
    x = zeros(I.numBots,1); y = zeros(I.numBots,1);
    for i=1:I.numBots
        for j=1:I.numEPMs
            if bots(i).EPMs(4,j) > I.numBots
                bots(i).EPMs(4,j) = 0;
                bots(i).EPMs(5,j) = 0;
            end
        end
        x(i) = bots(i).coordinates(1);
        y(i) = bots(i).coordinates(2);
    end
    I.startCOMX = round(mean(x),4); I.startCOMY = round(mean(y),4);
    I.goal(1) = 0;
    I.goal(2) = 10;
    I.setAxis = [I.startCOMX-(I.numBots*I.botDiam)/2 I.goal(1)+(I.numBots*I.botDiam)/2 I.startCOMY-(I.numBots*I.botDiam)/2 I.goal(2)+(I.numBots*I.botDiam)/2];
end
if I.obs == 4
    [world,I] = RandomObstacleGeneration(bots,I);
elseif I.obs == 1
    world = FixedObsConfig(I);
elseif I.obs == 2
    world.goal = rectangle('Position',[(I.goal(1)-.2) (I.goal(2)-.2) .55 .55],'FaceColor',[0 1 0]);
    matfile = strcat(I.readfileObs,I.obsName,'.mat');
    load(matfile,'obstacles');
    world.obstacles = obstacles;
    hold on
    for i=1:size(world.obstacles,3)
        aobstacle(1,:) = world.obstacles(:,1,i);
        aobstacle(2,:) = world.obstacles(:,2,i);
        fill(aobstacle(1,:), aobstacle(2,:),'r')
        hold on
        plot([transpose(aobstacle(1,:)); aobstacle(1,1)], [transpose(aobstacle(2,:)); aobstacle(2,1)],'b')
        hold on
    end
elseif I.obs == 3
    world.goal = rectangle('Position',[(I.goal(1)-.2) (I.goal(2)-.2) .55 .55],'FaceColor',[0 1 0]);
    world.obstacles = [];
end
I.setAxis = [I.startCOMX-(I.numBots*I.botDiam)/2 I.goal(1)+(I.numBots*I.botDiam)/2 I.startCOMY-(I.numBots*I.botDiam)/2 I.goal(2)+(I.numBots*I.botDiam)/2];

I.tempgoal = 0;
I.replan = 0;
I.state = 1;
world.obstacleDetect = zeros(2,I.numBots);

I.seenObs = 1:size(world.obstacles,3);
I.first = 1;
I.ang = zeros(1,I.numBots);
I.sign = ones(2,I.numBots);
bots = ConnectionDetermination(bots,I);
[bots,world,I] = TransitionDetermination(bots,world,I);
I = Plot(bots,I); %plots the bots and world
end