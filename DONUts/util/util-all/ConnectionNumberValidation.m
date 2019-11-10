%%Iterates through each robot to check the number of
%%connections that it has. If it has more than a certain value then it
%%can definitely not move.
%%0 = cannot move, 1 = CW, -1 = CCW, 2 = both

function [bots,world,I] = ConnectionNumberValidation(bots,world,I)

%Generates a random order in which to iterate through the bots
robotsRandom = RandomBotOrderGenerator(I.numBots);

for k = 1:I.numBots
    countConnections = 0;
    for i = 1:I.numEPMs
        
        if bots(robotsRandom(k)).EPMs(4,i) ~= 0 
            countConnections = countConnections + 1;
        else
            bots(robotsRandom(k)).EPMs(5,i) = 0;
        end
    end
    if(countConnections == 0 || countConnections >= 5)
        world.canMove(robotsRandom(k)) = 0;
    end
    %%%
    if world.canMove(robotsRandom(k)) == 0
        for i = 1:I.numEPMs
            bots(robotsRandom(k)).EPMs(5,i) = 0;
        end
    end
    %%%
    possibleToMove = 0;
    for i = 1:I.numEPMs
        if bots(robotsRandom(k)).EPMs(5,i) ~= 0 
            possibleToMove = possibleToMove + 1;
        end
    end
    if(possibleToMove == 0)
        world.canMove(robotsRandom(k)) = 0;
    end
end


