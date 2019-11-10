%%Calculates the position of each of the magnetMarks and sensorMarks
%%Recalculates the center coordinate for each robot and calculates the
%%position of the magnets and the sensors around each robot.

function [bots,I] = SensorEPMPositions(bots,I,currentBot)
    %%The angles where a red dot will be plotted on each circle to represent
    %%the location of the EPMs 
    for k = 1:I.numEPMs
        bots(currentBot).EPMs(2,k) = round((bots(currentBot).coordinates(1) + cos(I.EPMMarks(k))),4);
        bots(currentBot).EPMs(3,k) = round((bots(currentBot).coordinates(2) + sin(I.EPMMarks(k))),4);
    end
    for k=1:I.numIR
        bots(currentBot).IRsensors(2,k) = round((bots(currentBot).coordinates(1) + cos(I.IRMarks(k))),4);
        bots(currentBot).IRsensors(3,k) = round((bots(currentBot).coordinates(2) + sin(I.IRMarks(k))),4);
    end
end
    
