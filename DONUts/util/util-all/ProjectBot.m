%ProjectBot returns the new center coordinates, epm, and sensor positions
%of a module if it is moved in a certain direction
function [coordinates,epmPos,irPos] = ProjectBot(bots,I,currentBot,magNum,position,adjBot)
    %%Check currentBots if the robot can move in the CW direction
    if bots(currentBot).EPMs(5,magNum) == 1
        nextStep = position - 1;
        if(nextStep < 1)
            nextStep = abs(I.numEPMs + nextStep);
        end
        coordinates(1) = bots(adjBot).coordinates(1) + cos(2*pi/I.numEPMs*(nextStep-1))*((2*I.numEPMs))/(I.numEPMs);
        coordinates(2) = bots(adjBot).coordinates(2) + sin(2*pi/I.numEPMs*(nextStep-1))*((2*I.numEPMs))/(I.numEPMs);
        coordinates(1) = round(coordinates(1),4);
        coordinates(2) = round(coordinates(2),4);
    end
    %%Check currentBots if the robot can move in the CCW direction
    if bots(currentBot).EPMs(5,magNum) == -1
        nextStep = position + 1;
        if(nextStep > I.numEPMs)
            nextStep = abs(I.numEPMs - nextStep);
        end
        coordinates(1) = bots(adjBot).coordinates(1) + cos(2*pi/I.numEPMs*(nextStep-1))*((2*I.numEPMs))/(I.numEPMs);
        coordinates(2) = bots(adjBot).coordinates(2) + sin(2*pi/I.numEPMs*(nextStep-1))*((2*I.numEPMs))/(I.numEPMs);
    end
    %%Update epm and sensor positions
    for k = 1:I.numEPMs
        epmPos(1,k) = round(coordinates(1) + cos(I.EPMMarks(k)),4);
        epmPos(2,k) = round(coordinates(2) + sin(I.EPMMarks(k)),4);
    end
    for k=1:I.numIR
        irPos(1,k) = round(coordinates(1) + cos(I.IRMarks(k)),4);
        irPos(2,k) = round(coordinates(2) + sin(I.IRMarks(k)),4);
    end
end