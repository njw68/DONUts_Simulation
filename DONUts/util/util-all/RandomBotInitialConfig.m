%%Creates random starting positions for all of the DONUTS
function [bots,I] = RandomBotInitialConfig(bots,I)
count = 1;
currentBot = 1;
maxBot = currentBot;
%Determines the angles at which the EPMs and IR sensors will be plotted and
%sets them for bot1
minBotDist = I.botDiam;
I.EPMMarks = (0:2*pi/I.numEPMs:2*pi*(I.numEPMs-1)/I.numEPMs);
%%The angles where a blue dot will be plotted on each circle to represent
%%the location of the IR sensors
I.IRMarks = (-pi/4)+(2*pi/I.numIR):(2*pi/I.numIR):(2*pi);
[bots,I] = SensorEPMPositions(bots,I,maxBot);

while(count < I.numBots)
    count = count + 1;
    validDir = 0;
    %%Checks if the randomly generated direction is valid
    while(validDir == 0)
        %%Generates a random direction to expand the group of bots.
        randDir = randi(12);
        %%Checks for the surrounding directions of the random direction and
        %%makes the necessary changes in the +/- 1 directions of the new
        %%random direction.
        %%Checks for the -1 direction
        if(randDir == 1)
            randDirMinusOne = 12;
        else
            randDirMinusOne = randDir - 1;
        end
        %%Checks for the +1 direction
        if(randDir == 12)
            randDirPlusOne = 1;
        else
            randDirPlusOne = randDir + 1;
        end
        
        %%Checks if this is a valid direction to expand the tree.     
        if(bots(currentBot).EPMs(4,randDir) ~= 0 || bots(currentBot).EPMs(4,randDirPlusOne) ~= 0 || bots(currentBot).EPMs(4,randDirMinusOne) ~=0)
            validDir = 0;
        else
            validDir = 1;
        end
        
        if validDir == 1
            futureCenterX = bots(currentBot).coordinates(1) + I.botDiam*cos(randDir*2*pi/12);
            futureCenterY = bots(currentBot).coordinates(2) + I.botDiam*sin(randDir*2*pi/12);
            for int2=1:length(bots)
                x2 = bots(int2).coordinates(1); y2 = bots(int2).coordinates(2);
                x1 = futureCenterX; y1 = futureCenterY;
                euDist = sqrt(((x2-x1)^2)+((y2-y1)^2));
                if euDist < minBotDist
                    validDir = 0;
                end
            end
        
        end
    end
    
    %%Generate the new position for the added bot
    currentBot = currentBot + 1;
    bots(currentBot).coordinates(1) = bots(currentBot-1).coordinates(1) + I.botDiam*cos(randDir*2*pi/12);
    bots(currentBot).coordinates(2) = bots(currentBot-1).coordinates(2) + I.botDiam*sin(randDir*2*pi/12);
    %%Defines for the following scripts what is the maximum iteration of
    %%the bot that will be iterated to.
    maxBot = currentBot;
    bots = SensorEPMPositions(bots,I,maxBot);
    bots = ConnectionDetermination(bots,I);
end
    

%%Creates the geometry for the remaining bots
for k = 2:I.numBots
    bots(k).geometry(:,1) = bots(k).coordinates(1) + cos(I.t1);
    bots(k).geometry(:,2) = bots(k).coordinates(2) + sin(I.t1);
end






