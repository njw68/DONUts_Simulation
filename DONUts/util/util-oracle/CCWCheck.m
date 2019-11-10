%Oracle CCWCheck
function [bots,world,moves] = CCWCheck(bots,world,I,bot_rotate,C_rotateX,C_rotateY,bot,ang,moves,both,magpos,position)
    minBotDist = 1.9714;
    radius = 3*(I.botDiam/2);
    r = 2.9714;
    botObs = []; vr = -1;
    connBots = bot; %add the current bot to connected bot array
    dconnBots = [];
    obsBlock=0; botBlock=0;
    %get connected bots
    for y = 1:I.numEPMs
        %for bots connected to current bot
        if bots(bot).EPMs(4,y) ~= 0
            connBots = [connBots,bots(bot).EPMs(4,y)];
        end
        %for bots connected to neighboring bot
        if bots(bot_rotate).EPMs(4,y) ~= 0
            connBots = [connBots,bots(bot_rotate).EPMs(4,y)];
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
            dconnBots(end+1) = inc;
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
    %creates the CCW checked regions
    CCWx = [C_rotateX (C_rotateX + (radius)*cosd(ang(1))) (C_rotateX + (radius)*cosd(ang(2))) C_rotateX];
    CCWy = [C_rotateY (C_rotateY + (radius)*sind(ang(1))) (C_rotateY + (radius)*sind(ang(2))) C_rotateY];
    tri = polyshape(CCWx,CCWy);
    circle =  polyshape([r.*[cos(0:0.01:2*pi)]+C_rotateX], [r.*[sin(0:0.01:2*pi)]+C_rotateY]);

    th1 = ang(1); th2 = ang(2);
    if (th1 == 0 || th2 == 0) && (th1 == 300 || th2 == 300)
        th1 = 300; th2 = 360;
    end

    if (th1 == 330 || th2 == 330) && (th1 == 30 || th2 == 30)
        th1 = 330; th2 = 390;
    end
    th1 = deg2rad(th1); th2 = deg2rad(th2);
    if th1 < th2
        sem = polyshape([r.*[cos(th1:0.01:th2)]+C_rotateX], [r.*[sin(th1:0.01:th2)]+C_rotateY]);
    else
        sem = polyshape([r.*[cos(th2:0.01:th1)]+C_rotateX], [r.*[sin(th2:0.01:th1)]+C_rotateY]);
    end

    xtri = tri.Vertices(:,1); ytri = tri.Vertices(:,2);
    xsem = sem.Vertices(:,1); ysem = sem.Vertices(:,2);
    ObsAll = [];
    for i = 1:size(world.obstacles,3)
        ObsAll = [ObsAll; world.obstacles(:,:,i)];
    end
    %check obstacles: creates array of intersecting points
    if ~isempty(ObsAll)
        [inTri,onTri] = inpolygon(ObsAll(:,1),ObsAll(:,2),xtri,ytri);
        [inSem,onSem] = inpolygon(ObsAll(:,1),ObsAll(:,2),xsem,ysem);
    end
    %check bots: create array of intersecting points
    if ~isempty(botObs)
        [inTri2,onTri2] = inpolygon(botObs(:,1),botObs(:,2),xtri,ytri);
        [inSem2,onSem2] = inpolygon(botObs(:,1),botObs(:,2),xsem,ysem);
    end
    %checks if bots are in regions
    transOld = bots(bot).EPMs(5,magpos);
    bots(bot).EPMs(5,magpos) = -1;
    %find the starting position on bot_rotate
    for int = 1:I.numEPMs
        if bots(bot_rotate).EPMs(4,int) == bot
            position = int;
        end
    end
    [coordinates,epmPos,irPos] = ProjectBot(bots,I,bot,magpos,position,bot_rotate);
    bots(bot).EPMs(5,magpos) = transOld;
    for int=1:size(botObs,1)
        if inTri2(int) == 1 || inSem2(int) == 1
            for int2=1:length(dconnBots)
                x2 = bots(dconnBots(int2)).coordinates(1); y2 = bots(dconnBots(int2)).coordinates(2);
                x1 = coordinates(1); y1 = coordinates(2);
                euDist = sqrt(((x2-x1)^2)+((y2-y1)^2));

                if euDist < minBotDist
                    botBlock = 1;
                    obstacleDetect(bot) = -1;
                end
            end
            break
        end
    end
    %checks if obstacles are in regions
    for int=1:size(ObsAll,1)
        if inTri(int) == 1 || inSem(int) == 1
            obsBlock = 1;
            obstacleDetect(bot) = -1;
        end
    end
    %identifies transitons that need to be removed (if necessary)
    if botBlock == 1 || obsBlock == 1
        if isempty(both)
            moves = 0;
        else
            moves(2) = 0;
        end
    else
    
    end

end