%ShortestPath
function [magNum,direction,prevdir,blockDir,closeMag] = ShortestPath(bots,world,I,moverBot,closeBot,blockDir,prevdir,direction)
magNum = [];
botBlockDir = [0 0];
posAr = [7 8 9 10 11 12 1 2 3 4 5 6];
CCWar = [11 12 1 2 3 4];
CWar = [5 6 7 8 9 10];

cwPath = [bots(moverBot).coordinates, moverBot];
ccwPath = [bots(moverBot).coordinates, moverBot];
%find the closest magnet to the light source
marksAr=[];
for i=1:I.numEPMs
    marksAr(end+1,1) = bots(closeBot).EPMs(2,i);
    marksAr(end,2) = bots(closeBot).EPMs(3,i);
end
d = pdist2(I.goal,marksAr,'euclidean');
[~,closeMag]=min(d);

plus = zeros(1,3);
plus(1) = closeMag+1; plus(2) = closeMag+2; plus(3) = closeMag+3;
for i=1:3
    if plus(i) > I.numEPMs
        plus(i) = plus(i)-I.numEPMs;
    end
end
neg = zeros(1,3);
neg(1) = closeMag+1; neg(2) = closeMag+2; neg(3) = closeMag+3;
for i=1:3
    if neg(i) > I.numEPMs
        neg(i) = neg(i)-I.numEPMs;
    end
end

%select closeMag that isn't occupied
while bots(closeBot).EPMs(4,closeMag) ~= 0 && sum(plus) ~= 0 && sum(neg) ~= 0
    d(closeMag) = inf;
    [~,closeMag]=min(d);
end

%get array of transmagnets and their directions of movement
transMags=[]; neighbor=[];
for j=1:I.numEPMs
    if bots(moverBot).EPMs(5,j) ~= 0
        transMags(1,end+1) = j; %store magnetNum
        transMags(2,end) = bots(moverBot).EPMs(5,j); %store direction
        transMags(3,end)=bots(moverBot).EPMs(4,j); %store the neighbor
    end
end

if world.obstacleDetect(1,moverBot) == 1
    blockDir(1) = 1;
elseif world.obstacleDetect(1,moverBot) == -1
    blockDir(2) = 1;
elseif world.obstacleDetect(1,moverBot) == 2
    blockDir(1) = 1;
    blockDir(2) = 1;
end

if size(transMags,2) == 1 && transMags(2,1) ~= 2

    prevdir = direction;
    direction = transMags(2,1);
    disp(direction);
    magNum = transMags(1,1);
elseif isempty(transMags)
    magNum = [];
    prevdir = direction;
else
    for curMagnet=1:size(transMags,2)
        if transMags(2,curMagnet) == 1 %CW direction
            pbot = transMags(3,curMagnet); %present bot is the first neighbor of mover bot
            pmag = posAr(transMags(1,curMagnet)); %present epm is the first epm on present bot
            nextMag=0; %nextMag will be the next magnet to check in the CW direction
            cwPath(end+1,1:2) = bots(pbot).coordinates;
            cwPath(end,3) = pbot;
            if world.obstacleDetect(1,moverBot) == 1 || world.obstacleDetect(1,moverBot) == 2
                for i=1:6
                    if pmag == CWar(i)
                        blockDir(1)=1;
                    end
                end
            end
            %do search until you find the closest bot and closest epm
            while (pbot ~= closeBot || pmag ~= closeMag) && pbot ~= moverBot
                %check if there is an obstacle in this direction, stop searching
                %this way
                if blockDir(1) == 1
                    break
                end
                if world.obstacleDetect(1,pbot) == 1 || world.obstacleDetect(1,pbot) == 2
                    for i=1:6
                        if posAr(transMags(1,curMagnet)) == CWar(i)
                            blockDir(1)=1;
                            break
                        end
                    end
                end
                %look for the next CW magnet/bot
                if pmag-1 > 0
                    nextMag=pmag-1;
                else
                    nextMag=I.numEPMs;
                end
                if bots(pbot).EPMs(4,nextMag) == 0
                    pmag = nextMag;
                else
                    pbot = bots(pbot).EPMs(4,nextMag);
                    cwPath(end+1,1:2) = bots(pbot).coordinates;
                    cwPath(end,3) = pbot;
                    pmag = posAr(nextMag);
                end
            end

        elseif transMags(2,curMagnet) == -1 %CCW direction
            pbot = transMags(3,curMagnet); %present bot is the first neighbor of mover bot
            pmag = posAr(transMags(1,curMagnet)); %present magnet is the first magnet on present bot
            nextMag=0; %nextMag will be the next magnet to check in the CW direction
            ccwPath(end+1,1:2) = bots(pbot).coordinates;
            ccwPath(end,3) = pbot;
            if world.obstacleDetect(1,moverBot) == -1 || world.obstacleDetect(1,moverBot) == 2
                for i=1:6
                    if pmag == CCWar(i)
                        blockDir(2)=1;%
                        
                    end
                end
            end
            while (pbot ~= closeBot || pmag ~= closeMag) && pbot ~= moverBot
                %check if there is an obstacle in this direction
                if blockDir(2) == 1
                    
                    hopPath(2,curMagnet) = 0;
                    break
                end
                if world.obstacleDetect(1,pbot) == -1 || world.obstacleDetect(1,pbot) == 2
                    for i=1:6
                        if posAr(transMags(1,curMagnet)) == CCWar(i)
                            blockDir(2)=1;%
                            
                            hopPath(2,curMagnet) = 0; %sets hopPath in this direction to 0 (will be NaN later)
                            break
                        end
                    end
                end
                %look for the next CCW magnet/bot
                if pmag+1 > I.numEPMs
                    nextMag=1;
                else
                    nextMag=pmag+1;
                end
                if bots(pbot).EPMs(4,nextMag) == 0
                    pmag = nextMag;
                else
                    pbot = bots(pbot).EPMs(4,nextMag);
                    ccwPath(end+1,1:2) = bots(pbot).coordinates;
                    ccwPath(end,3) = pbot;
                    pmag = posAr(nextMag);
                end
            end
        else %check both directions
            %CW direction
            pbot = transMags(3,curMagnet); %present bot is the first neighbor of mover bot
            pmag = posAr(transMags(1,curMagnet)); %present magnet is the first magnet on present bot
            nextMag=0; %nextMag will be the next magnet to check in the CW direction
            cwPath(end+1,1:2) = bots(pbot).coordinates;
            cwPath(end,3) = pbot;
            if world.obstacleDetect(1,moverBot) == 1 || world.obstacleDetect(1,moverBot) == 2
                for i=1:6
                    if pmag == CWar(i)
                        blockDir(1)=1;%
                        
                    end
                end
            end
            while (pbot ~= closeBot || pmag ~= closeMag) && pbot ~= moverBot
                %check if there is an obstacle in this direction
                if blockDir(1) == 1
                    
                    hopPath(1,curMagnet) = 0;
                    break
                end
                if world.obstacleDetect(1,pbot) == 1 || world.obstacleDetect(1,pbot) == 2
                    for i=1:6
                        if posAr(transMags(1,curMagnet)) == CWar(i)
                            blockDir(1)=1;
                            break
                        end
                    end
                end
                %look for the next CW magnet/bot
                if pmag-1 > 0
                    nextMag=pmag-1;
                else
                    nextMag=I.numEPMs;
                end
                if bots(pbot).EPMs(4,nextMag) == 0
                    pmag = nextMag;
                else
                    pbot = bots(pbot).EPMs(4,nextMag);
                    cwPath(end+1,1:2) = bots(pbot).coordinates;
                    cwPath(end,3) = pbot;
                    pmag = posAr(nextMag);
                end
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %CCW direction
            pbot = transMags(3,curMagnet); %present bot is the first neighbor of mover bot
            pmag = posAr(transMags(1,curMagnet)); %present magnet is the first magnet on present bot
            nextMag=0; %nextMag will be the next magnet to check in the CW direction
            ccwPath(end+1,1:2) = bots(pbot).coordinates;
            ccwPath(end,3) = pbot;
            if world.obstacleDetect(1,moverBot) == -1 || world.obstacleDetect(1,moverBot) == 2
                for i=1:6
                    if pmag == CCWar(i)
                        blockDir(2)=1;
                        
                    end
                end
            end
            while (pbot ~= closeBot || pmag ~= closeMag) && pbot ~= moverBot
                %check if there is an obstacle in this direction
                if blockDir(2) == 1
                    
                    hopPath(2,curMagnet) = 0;
                    break
                end

                if world.obstacleDetect(1,pbot) == -1 || world.obstacleDetect(1,pbot) == 2
                    for i=1:6
                        if posAr(transMags(1,curMagnet)) == CCWar(i)
                            blockDir(2)=1;
                            break
                        end
                    end
                end
                %look for the next CCW magnet/bot
                if pmag+1 > I.numEPMs
                    nextMag=1;
                else
                    nextMag=pmag+1;
                end
                if bots(pbot).EPMs(4,nextMag) == 0
                    pmag = nextMag;
                else
                    pbot = bots(pbot).EPMs(4,nextMag);
                    ccwPath(end+1,1:2) = bots(pbot).coordinates;
                    ccwPath(end,3) = pbot;
                    pmag = posAr(nextMag);
                end
            end
        end
    end
    if pbot == moverBot
        closeBotPosCW = find(cwPath(:,3)==closeBot);
        cwPath = cwPath(1:closeBotPosCW,:);
        closeBotPosCCW = find(ccwPath(:,3)==closeBot);
        ccwPath = ccwPath(1:closeBotPosCCW,:);
    end
    cwPath(end+1,1:2) = marksAr(closeMag,:);
    cwPath(end,3) = closeMag;
    ccwPath(end+1,1:2) = marksAr(closeMag,:);
    ccwPath(end,3) = closeMag;
    blockDir(1) = 0; blockDir(2) = 0; %comment to continue doing blocked direction checks
    if blockDir(1) == 1
        cwPath = inf;
        prevdir = direction;
        direction = -1;
        
    elseif blockDir(2) == 1
        ccwPath = inf;
        prevdir = direction;
        direction = 1;
        
    end
    if blockDir(1) == 0 && blockDir(2) == 0
        %cwPath
        for u=1:size(transMags,2)
            if transMags(2,u) == 1 || transMags(2,u) == 2
                lastMag = transMags(1,u);
            end
        end
        if size(cwPath,1) > 1
            [~,cwPathD,~] = CalculatePath(bots,I,cwPath,moverBot,closeMag,closeBot,1,lastMag);
        else
            cwPathD = cwPath(3);
        end
        %ccwPath
        for u=1:size(transMags,2)
            if transMags(2,u) == -1 || transMags(2,u) == 2
                lastMag = transMags(1,u);
            end
        end
        if size(ccwPath,1) > 1
            [ccwPath,ccwPathD,block] = CalculatePath(bots,I,ccwPath,moverBot,closeMag,closeBot,-1,lastMag);
        else
            ccwPathD = ccwPath(3);
        end
        prevdir = direction;
        if cwPathD < ccwPathD
            direction = 1;
        elseif ccwPathD < cwPathD
            direction = -1;
        else
            a = rand;
            if a < .5
                direction = 1;
            else
                direction = -1;
            end
        end
        if botBlockDir(1) == 1 && botBlockDir(2) == 0
            direction = -1;
            for i=1:size(transMags,2)
                if transMags(2,i) == direction || transMags(2,i) == 2
                    magNum = transMags(1,i);
                    break
                end
            end
        elseif botBlockDir(1) == 0 && botBlockDir(2) == 1
            direction = 1;
            for i=1:size(transMags,2)
                if transMags(2,i) == direction || transMags(2,i) == 2
                    magNum = transMags(1,i);
                    break
                end
            end
        elseif botBlockDir(1) == 0 && botBlockDir(2) == 0
            for i=1:size(transMags,2)
                if transMags(2,i) == direction || transMags(2,i) == 2
                    magNum = transMags(1,i);
                    break
                end
            end
        else
            magNum = [];
            prevdir = direction;
        end
    end
end