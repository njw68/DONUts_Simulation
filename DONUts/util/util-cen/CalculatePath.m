function [path,pathD,botBlockedDir] = CalculatePath(bots,I,path,moverBot,closeMag,closeBot,dir,lastMag)
    %remove skipped bots from the path
    pos = closeMag; %set the first epm position to project at, minus 1 because it is not necessary to project at the goal epm location
    adjBot = closeBot; %set the first adjacent (neighboring) bot to project at
    pathPos = length(path);
    deleteBots = [];
    inic = 0;
    minBotDist = 2.4653;
    pathD = 0;
    botBlockedDir = 0;
    tempBots = bots; %creating bots struct that can be changed for projections
    tempBots(moverBot).EPMs(4,:) = 0;
    calculating = true;
    tempBots(moverBot).EPMs(5,:) = -dir; %changing moverBot's transitions to the opposite direction, this is incase it previously had both transitions
    %CCW because to search from the bottom of the stack to the top, bots
    %muct be projected in the reverse direction
    %iterate through the epms on the perimeter of the collective
    its = 0;
    while adjBot ~= path(2,3) && calculating == true && pos ~= lastMag
        [coordinates, epmPos] = ProjectBot(tempBots,I,moverBot,pos,pos,adjBot); %project bot onto next step
        tempBots(moverBot).EPMs(2:3,:) = epmPos; %update epm position values
        tempBots(moverBot).EPMs(4,:) = 0;
        tempBots = ConnectionDetermination(tempBots,I); %find connections in projected step
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %get connected bots
        connBots = [];
        dconnBots = [];
        botObs = [];
        for y = 1:I.numEPMs
            %for bots connected to current bot
            if tempBots(moverBot).EPMs(4,y) ~= 0
                connBots = [connBots,tempBots(moverBot).EPMs(4,y)];
            end
            %for bots connected to neighboring bot
            if tempBots(adjBot).EPMs(4,y) ~= 0
                connBots = [connBots,tempBots(adjBot).EPMs(4,y)];
            end
        end
        %get disconnected bots
        for inc = 1:I.numBots
            indic = 0;
            for inc2 = 1:length(connBots)
                if inc == connBots(inc2)
                    indic=1;
                end
            end
            if indic==0
                if pdist2(coordinates,tempBots(inc).coordinates,'euclidean') < I.botDiam*3
                    dconnBots(end+1) = inc;
                end
            end
        end
        
        %add disconnected bots' magnets and sensors to an array
        for inc = 1:length(dconnBots)
            amagnet(1,:) = bots(dconnBots(inc)).EPMs(2,:);
            amagnet(2,:) = bots(dconnBots(inc)).EPMs(3,:);
            asensor(1,:) = bots(dconnBots(inc)).IRsensors(2,:);
            asensor(2,:) = bots(dconnBots(inc)).IRsensors(3,:);
            botObs = [botObs;amagnet';asensor'];
        end
        for int2=1:length(dconnBots)
                x2 = tempBots(dconnBots(int2)).coordinates(1); y2 = tempBots(dconnBots(int2)).coordinates(2);
                x1 = coordinates(1); y1 = coordinates(2);
                euDist = round(sqrt(((x2-x1)^2)+((y2-y1)^2)),4);
                if euDist < minBotDist
                    botBlockedDir = 1;
                    pathD = inf;
                    calculating = false;
                end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        connection = 0;
        %check if any projected epms connect
        for i=1:I.numEPMs
            if tempBots(moverBot).EPMs(4,i) ~= 0 && tempBots(moverBot).EPMs(4,i) ~= adjBot
                connection = 1;
            end
        end
        %if there is no connection, increment the epm position
        if connection == 0 && calculating == true
            pos = pos+dir;
            if pos > I.numEPMs
                pos = 1;
            elseif pos < 1
                pos = I.numEPMs;
            end
        %if there is a connection, update the path, adjBot, and pos
        elseif connection == 1 && calculating == true
            for i=1:size(path,1)
                for j=1:I.numEPMs
                    if path(i,3) == tempBots(moverBot).EPMs(4,j) && path(i,3) ~= adjBot && length(find(path(1:end-1,3)==path(i,3))) ~= 2 %check if the bot in the path is the connected bot
                        for h=1:size(path,1)-1
                            if path(h,3) == adjBot
                                pathPos = h; %get the current path position
                            end
                        end
                        adjBot = tempBots(moverBot).EPMs(4,j); %get new adjacent bot
                        pathLoc = find(path(:,3) == adjBot);
                        if length(pathLoc) == 1
                            deleteBots = i; %index where to delete path list from top
                        else
                            deleteBots = pathLoc(end);
                        end
                        inic = 1;
                        break
                    end
                    if inic == 1
                        break
                        j = I.numEPMs+1;
                    end
                end
                if inic == 1
                    break
                end
            end
            inic = 0;
            if pathPos - deleteBots > 1
                path(deleteBots+1:pathPos-1,:) = []; %deleting the skipped bots from the path
                pathPos=0; deleteBots=0;
            end
            for i=1:I.numEPMs
                if tempBots(adjBot).EPMs(4,i) == moverBot
                    pos = i;%+1; %set new epm position
                    break
                end
            end
        end
        %"time" out
        its = its+1;
        if its >= 50
            adjBot = path(2,3);
            calculating = false;
            pos = lastMag;
        end
    end
    if pathD == 0
        for i=1:size(path,1)-1
            pathD = pathD + pdist2(path(i,2:3),path(i+1,2:3),'euclidean');
        end
    end
    end