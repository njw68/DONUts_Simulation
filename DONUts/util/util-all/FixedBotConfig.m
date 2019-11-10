%FixedInitConfig
function [bots,I] = FixedBotConfig(bots,I)
maxBot=1; %Bot to determine EPM and IR sensor positions for
%Determines the angles at which the EPMs and IR sensors will be plotted and
%sets them for bot1
I.EPMMarks = (0:2*pi/I.numEPMs:2*pi*(I.numEPMs-1)/I.numEPMs);
%%The angles where a blue dot will be plotted on each circle to represent
%%the location of the IR sensors
I.IRMarks = (-pi/4)+(2*pi/I.numIR):(2*pi/I.numIR):(2*pi);
[bots,I] = SensorEPMPositions(bots,I,maxBot);

count = 0;
%%Sets the bots in the initial configuration
for j = 0:I.numBotsY-1
    for i = 0:I.numBotsX-1
        count = count + 1;
        bots(count).geometry(:,1) = round((bots(1).geometry(:,1)+cos(2*pi/I.numSides*0)*((2*I.numSides)-1)/(I.numSides)*i+(I.botDiam/2 +(I.botDiam/2*1/60))*((((-1)^(j+1))+1)/2)),4);
        bots(count).geometry(:,2) = round((bots(1).geometry(:,2)+sin(2*pi/I.numEPMs*4)*((2*I.numEPMs))/(I.numEPMs)*j),4);
    end
end

%Calculates the center of each robot
center = zeros(I.numBots,2);
for k = 1:I.numBots
    bots(k).coordinates(1) = mean(bots(k).geometry(:,1));
    bots(k).coordinates(2) = mean(bots(k).geometry(:,2));
    center(k,1) = mean(bots(k).geometry(:,1));
    center(k,2) = mean(bots(k).geometry(:,2));
end
centerX = round(mean(center(:,1)),4);
centerY = round(mean(center(:,2)),4);

%%Recenters all of the bots at the world origin
for k = 1:I.numBots
    bots(k).geometry(:,1) = bots(k).geometry(:,1) - centerX;
    bots(k).geometry(:,2) = bots(k).geometry(:,2) - centerY;
end
%%Recalculates the center coordinate for each bot and calculates the
%%position of the magnets and the sensors around each robot.
for k = 1:I.numBots
    bots(k).coordinates(1) = mean(bots(k).geometry(:,1));
    bots(k).coordinates(2) = mean(bots(k).geometry(:,2));
    bots = SensorEPMPositions(bots,I,k);
end
for k = 1:I.numBots
    bots(k).coordinates(1) = mean(bots(k).geometry(:,1));
    bots(k).coordinates(2) = mean(bots(k).geometry(:,2));
    center(k,1) = mean(bots(k).geometry(:,1));
    center(k,2) = mean(bots(k).geometry(:,2));
end
%centers the modules and sets the goal location
I.startCOMX = round(mean(center(:,1)),4);
I.startCOMY = round(mean(center(:,2)),4);
I.goal(1) = I.startCOMX + (I.numBots*I.botDiam)/sqrt(2);
I.goal(2) = I.startCOMY + (I.numBots*I.botDiam)/sqrt(2);