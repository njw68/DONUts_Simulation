%Transition Determination
function [bots,world,I] = TransitionDetermination(bots,world,I)
%%At the start of each iteration, all robots start as having 24
%%possible moves. This is identified by all having a value of 1 in the
%%canMove array and a value of 2 in the possibleMove array.
world.canMove = ones(1,I.numBots);
for i=1:I.numBots
    bots(i).EPMs(5,:) = 2;
end
%for keeping track of which bots have detected obstacles
world.obstacleDetect = zeros(2,I.numBots);
world.botDetect = zeros(1,I.numBots);
%%Checks if a robot has less than 5 connections. If it does then it is 
%%marked as still able to move.
[bots,world,I] = ConnectionNumberValidation(bots,world,I);
bots = ConnectionAngleValidation(bots,world,I);
%%updates possible transitions
for k = 1:I.numBots
    possibleToMove = 0;
    for i = 1:I.numEPMs
        if bots(k).EPMs(5,i) ~= 0
            possibleToMove = possibleToMove + 1;
        end
    end
    if(possibleToMove == 0)
        world.canMove(k) = 0;
    end
end
%%obstacle avoidance
[bots,world,I] = ObstacleAvoidance(bots,world,I);
%%update possible transitions
for k = 1:I.numBots
    possibleToMove = 0;
    for i = 1:I.numEPMs
        if bots(k).EPMs(5,i) ~= 0
            possibleToMove = possibleToMove + 1;
        end
    end
    if(possibleToMove == 0)
        world.canMove(k) = 0;
    end
end
%check for disconnections
[bots,world] = DisconnectionCheck(bots,world,I);


            
            
            
            
            
            