function [farBot,closeBot,dist,comDist,farBotDist,closeBotDist] = DistanceCOM(state,I,goal)
    %calculates the distance from the collective's COM to the goal
    xcoords = zeros(1,I.numBots); ycoords = zeros(1,I.numBots); dist = zeros(1,I.numBots);
    for indx = 1:I.numBots
        %get x and y coordinates of each bot in time step
        xcoords(indx) = state.bots(indx).coordinates(1);
        ycoords(indx) = state.bots(indx).coordinates(2);
        dist(1,indx) = pdist2([xcoords(indx) ycoords(indx)],goal,'euclidean');
    end
    [farBotDist,farBot] = max(dist); %calculate the distance of the farthest bot
    [closeBotDist,closeBot] = min(dist); %calculate the distance of the closest bot
    comDist = pdist2([mean(xcoords) mean(ycoords)],goal,'euclidean');
end