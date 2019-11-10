%MoveBot
function bots = MoveBot(bots,I,currentBot,magNum)
%%Calculate the next position of a robot regardless of the number of
botRad = 22.9183; %%mm

%%Check currentBots if the robot can move in the CW direction
if bots(currentBot).EPMs(5,magNum) == 1 
    for int = 1:I.numEPMs
        if bots(bots(currentBot).EPMs(4,magNum)).EPMs(4,int) == currentBot 
            position = int;
        end
    end
    nextStep = position - 1;
    if(nextStep < 1)
        nextStep = abs(I.numEPMs + nextStep);
    end

    bots(currentBot).coordinates(1) = bots(bots(currentBot).EPMs(4,magNum)).coordinates(1) + cos(2*pi/I.numEPMs*(nextStep-1))*((2*I.numEPMs))/(I.numEPMs);
    bots(currentBot).coordinates(2) = bots(bots(currentBot).EPMs(4,magNum)).coordinates(2) + sin(2*pi/I.numEPMs*(nextStep-1))*((2*I.numEPMs))/(I.numEPMs);

end
%%Check currentBots if the robot can move in the CCW direction
if bots(currentBot).EPMs(5,magNum) == -1 
    for int = 1:I.numEPMs
        if bots(bots(currentBot).EPMs(4,magNum)).EPMs(4,int) == currentBot 
            position = int;
        end
    end
    nextStep = position + 1;
    if(nextStep > I.numEPMs)
        nextStep = abs(I.numEPMs - nextStep);
    end

    bots(currentBot).coordinates(1) = bots(bots(currentBot).EPMs(4,magNum)).coordinates(1) + cos(2*pi/I.numEPMs*(nextStep-1))*((2*I.numEPMs))/(I.numEPMs);
    bots(currentBot).coordinates(2) = bots(bots(currentBot).EPMs(4,magNum)).coordinates(2) + sin(2*pi/I.numEPMs*(nextStep-1))*((2*I.numEPMs))/(I.numEPMs);

end
%%Check currentBots if the robot can move in the CW direction
if (bots(currentBot).EPMs(5,magNum) == -1) %|| (bots(currentBot).EPMs(5,magNum) == 2)
    %%Labels the new position of the magnets with respect to the inertial
    %%frame of reference. This allows for easier plotting methods
    for selfMagnets = 1:I.numEPMs
        newMagnetPosition = bots(currentBot).EPMs(1,selfMagnets)-1;
        if(newMagnetPosition < 1)
            newMagnetPosition = abs(I.numEPMs + newMagnetPosition);
        end
        bots(currentBot).EPMs(1,selfMagnets) = newMagnetPosition;
    end
    %find angle between world reference
    index1 = find(bots(currentBot).EPMs(1,:)==1);
    epmVec = [bots(currentBot).EPMs(2,index1)-bots(currentBot).coordinates(1) bots(currentBot).EPMs(3,index1)-bots(currentBot).coordinates(2) 0];
    xVec = [bots(currentBot).coordinates(1)+I.botDiam 0 0];
    radAng = 2*pi/I.numEPMs*(index1-1);
    %update the sensor position
    for ii = 1:4
        bots(currentBot).IRSensor(2,ii) = bots(currentBot).coordinates(1) + botRad*cos(radAng - pi/4 +(2*pi/4*ii));
        bots(currentBot).IRSensor(3,ii) = bots(currentBot).coordinates(2) + botRad*sin(radAng - pi/4 +(2*pi/4*ii));
    end
%%Check currentBots if the robot can move in the CCW direction
elseif (bots(currentBot).EPMs(5,magNum) == 1)
    %%Labels the new position of the magnets with respect to the inertial
    %%frame of reference. This allows for easier plotting methods
    for selfMagnets = 1:I.numEPMs
        newMagnetPosition = bots(currentBot).EPMs(1,selfMagnets)+1;
        if(newMagnetPosition > I.numEPMs)
            newMagnetPosition = abs(I.numEPMs - newMagnetPosition);
        end
        bots(currentBot).EPMs(1,selfMagnets) = newMagnetPosition;
    end
    %find angle between world reference
    index1 = find(bots(currentBot).EPMs(1,:)==1);
    epmVec = [bots(currentBot).EPMs(2,index1)-bots(currentBot).coordinates(1) bots(currentBot).EPMs(3,index1)-bots(currentBot).coordinates(2) 0];
    xVec = [bots(currentBot).coordinates(1)+I.botDiam 0 0];
    radAng = 2*pi/I.numEPMs*(index1-1);
    %update the sensor position
    for ii = 1:4
        bots(currentBot).IRSensor(2,ii) = bots(currentBot).coordinates(1) + botRad*cos(radAng - pi/4 +(2*pi/4*ii));
        bots(currentBot).IRSensor(3,ii) = bots(currentBot).coordinates(2) + botRad*sin(radAng - pi/4 +(2*pi/4*ii));
    end
end


