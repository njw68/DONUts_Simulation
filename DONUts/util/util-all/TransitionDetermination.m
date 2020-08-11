%Transition Determination
%%This function calculates the possible moves each module can take
%%in the current time step, and returns a updated 'bots', 'world',
%%and 'I' structs

function [bots,world,I] = TransitionDetermination(bots,world,I)
world.canMove = ones(1,I.numBots);
for i=1:I.numBots
    bots(i).EPMs(5,:) = 2;
end
%For keeping track of which bots have detected obstacles
world.obstacleDetect = zeros(2,I.numBots);
world.botDetect = zeros(1,I.numBots);

%Ensures 'Motion Restriction 2' is not violated
[bots,world,I] = ConnectionNumberValidation(bots,world,I);
bots = ConnectionAngleValidation(bots,world,I);

%Updates possible transitions
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

%Ensures the move won't cause a collision with
%an obstacle
[bots,world,I] = ObstacleAvoidance(bots,world,I);

%Updates possible transitions
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

%Ensures 'Motion Restriction 3' is not violated;
%the move won't disrupt global connectivity
%in the collective
[bots,world] = DisconnectionCheck(bots,world,I);


            
            
            
            
            
            
