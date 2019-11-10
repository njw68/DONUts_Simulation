%Converge
%determines if the collective has clustered around the goal

function [converge,stdD] = ConvergeCheck(child,I,C)
stdD = 0;
converge = 0;

if I.Converge == 2
    %by collective COM distance from the goal
    if (child.COM/I.botDiam <= 1)
        converge=1;
    else
        converge = 0;
    end
elseif I.Converge == 0.1
    %by collective COM distance from goal and std of bots from goal
    d = zeros(I.numBots,2);
    for i=1:I.numBots
        d(i,1) = child.bots(i).coordinates(1);
        d(i,2) = child.bots(i).coordinates(2);
    end
    if (child.COM <= I.botDiam) && (std(d(:,1)) <= I.botDiam+0.3) && (std(d(:,2)) <= I.botDiam+0.3)
        converge=1;
    else
        converge=0;
    end
elseif I.Converge == 0.2
    %by modules being within a specified radius (faf)
    x = zeros(I.numBots,1); y = zeros(I.numBots,1);
    for i=1:I.numBots
        x(i) = child.bots(i).coordinates(1);
        y(i) = child.bots(i).coordinates(2);
    end
    values = inpolygon(x,y,C(1,:),C(2,:));
    modOut = 0;
    for i=1:length(values)
        if values(i) == 0
            modOut = 1; 
        end
    end
    if modOut == 0 && child.COM <= I.botDiam
        converge = 1;
    else
        converge = 0;
    end
end