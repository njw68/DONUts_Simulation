%%Checks if a module has its connecting module spaced out adequately for it
%%to move in specific directions.
%%i.e. If # of connections is more than 2 and the summation of the angles
%%of separation is more than 180 degrees then the robot cannot move.
function bots = ConnectionAngleValidation(bots,world,I)

modulesRandom = RandomBotOrderGenerator(I.numBots);
for k = 1:I.numBots
    for i = 1:I.numEPMs
        possibleDirections(k,i,1) = 1;
        possibleDirections(k,i,2) = 1;
    end
end

for k = 1:I.numBots
    if world.canMove(modulesRandom(k)) == 1
        for i = 1:I.numEPMs
            
            if bots(modulesRandom(k)).EPMs(4,i) ~= 0
                nextMagnet = i;
                prevMagnet = i;
                %%Makes sure it is checking it is still possible to move about
                %%the magnet it is iterating through
                
                if bots(modulesRandom(k)).EPMs(5,i) ~= 0 
                    %%Iterates through five magnets in the clockwise while the
                    %%magnet has not yet prohibited the module from moving.
                    countAdjMagnets = 0;
                    while countAdjMagnets < 5 && bots(modulesRandom(k)).EPMs(5,i) ~= 0
                        countAdjMagnets = countAdjMagnets + 1;
                        nextMagnet = nextMagnet + 1;
                        prevMagnet = prevMagnet - 1;
                        if(nextMagnet > I.numEPMs)
                            nextMagnet = abs(I.numEPMs - nextMagnet);
                        end
                        if(prevMagnet < 1)
                            prevMagnet = abs(I.numEPMs + prevMagnet);
                        end

                        %%Checks if there is a module connected at the
                        %%location of nextMagnet
                        if bots(modulesRandom(k)).EPMs(4,nextMagnet) ~= 0
                            possibleDirections(modulesRandom(k),i,1) = 0;
                        end
                        %%Checks if there is a module connected at the
                        %%location of prevMagnet
                        if bots(modulesRandom(k)).EPMs(4,prevMagnet) ~= 0
                            possibleDirections(modulesRandom(k),i,2) = 0;
                        end
                        if((possibleDirections(modulesRandom(k),i,1) == 0) && (possibleDirections(modulesRandom(k),i,2) == 0))
                            bots(modulesRandom(k)).EPMs(5,i) = 0;
                        elseif((possibleDirections(modulesRandom(k),i,1) == 1) && (possibleDirections(modulesRandom(k),i,2) == 0))
                            bots(modulesRandom(k)).EPMs(5,i) = 1;
                        elseif((possibleDirections(modulesRandom(k),i,1) == 0) && (possibleDirections(modulesRandom(k),i,2) == 1))
                            bots(modulesRandom(k)).EPMs(5,i) = -1; 
                        elseif((possibleDirections(modulesRandom(k),i,1) == 1) && (possibleDirections(modulesRandom(k),i,2) == 1))
                            bots(modulesRandom(k)).EPMs(5,i) = 2; 
                        end
                    end
                end
            end
        end
    end
end