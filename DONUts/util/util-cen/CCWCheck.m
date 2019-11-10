function [bots,world,moves,I] = CCWCheck(bots,world,I,moves,bot,bot_rotate,both,Tri,sensorPos,magpos)
%Centralized CCWCheck
%add only non connected bot magnets and sensors to the obstacle array
minBotDist = 1.971497;
botObs = []; vr = 1;
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

%find coordinates of the projected position
transOld = bots(bot).EPMs(5,magpos);
bots(bot).EPMs(5,magpos) = -1;
%find the starting position on bot_rotate
for int = 1:I.numEPMs
    if bots(bot_rotate).EPMs(4,int) == bot
        position = int;
    end
end
[coordinates,~,~] = ProjectBot(bots,I,bot,magpos,position,bot_rotate);
bots(bot).EPMs(5,magpos) = transOld;

%iterate through the regions to be checked
for i=1:length(Tri)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    %get vertices of the region to be checked
    xtri = Tri(i).Vertices(:,1);
    ytri = Tri(i).Vertices(:,2);
    %check bots: create array of intersecting points
    if ~isempty(botObs)
        [inbotObs,~] = inpolygon(botObs(:,1),botObs(:,2),xtri,ytri);
    end
    %checks if bots are too close
    for int=1:size(botObs,1)
        if inbotObs(int) == 1
            for int2=1:length(dconnBots)
                x2 = bots(dconnBots(int2)).coordinates(1); y2 = bots(dconnBots(int2)).coordinates(2);
                x1 = coordinates(1); y1 = coordinates(2);
                euDist = sqrt(((x2-x1)^2)+((y2-y1)^2));
                if euDist < minBotDist
                    botBlock = 1;
                end
                if botBlock == 1
                    bots(bot).IRsensors(4,sensorPos(i)) = 1;
                    if world.botDetect(1,bot) == 0 || world.botDetect(1,bot) == -1
                        world.botDetect(1,bot) = -1;
                    elseif world.botDetect(1,bot) == 1
                        world.botDetect(1,bot) = 2;
                    end
                end
            end
            break
        end
    end

    %checks if obstacles are in regions
    if ~isempty(world.obstacles)
        px = []; py = [];
        %get the points for an obstacle and created the intersecting points
        %vector
        for obsNum = 1:length(I.seenObs)
            px = world.obstacles(:,1,I.seenObs(obsNum));
            py = world.obstacles(:,2,I.seenObs(obsNum));
            obsPoly = polyshape(px,py); %new
            collide = intersect(Tri(i),obsPoly); %new
            %iterate through vector to see if any points are in the checked
            %region
                if collide.NumRegions ~= 0
                    world.obstacleDetect(2,bot) = I.seenObs(obsNum);
                    %check of projected (oracle) cone based on the point of
                    %rotation
                    C_rotateX = bots(bot_rotate).coordinates(1);
                    C_rotateY = bots(bot_rotate).coordinates(2);
                    radius = 3*(I.botDiam/2);
                    r = 2.9714;
                    %get correct angles to project based on which epm is
                    %currently being evaluated
                    angles = [0 30 60 90 120 150 180 210 240 270 300 330];
                    moverMagPos = [7 8 9 10 11 12 1 2 3 4 5 6];
                    if magpos ==11
                        ang(1) = angles(moverMagPos(magpos)); ang(2) = angles(moverMagPos(1)); %CCW angles
                    elseif magpos ==12
                        ang(1) = angles(moverMagPos(magpos)); ang(2) = angles(moverMagPos(2)); %CCW angles
                    else
                        ang(1) = angles(moverMagPos(magpos)); ang(2) = angles(moverMagPos(magpos+2)); %CCW angles
                    end
                    
                    %creates the CCW checked regions
                    CCWx = [C_rotateX (C_rotateX + (radius)*cosd(ang(1))) (C_rotateX + (radius)*cosd(ang(2))) C_rotateX];
                    CCWy = [C_rotateY (C_rotateY + (radius)*sind(ang(1))) (C_rotateY + (radius)*sind(ang(2))) C_rotateY];
                    roTri = polyshape(CCWx,CCWy);

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
                    %create intersecting object
                    collide_roTri = intersect(roTri,obsPoly);
                    collide_sem = intersect(sem,obsPoly);
                    if (collide_roTri.NumRegions ~= 0) || (collide_sem.NumRegions ~= 0)
                        obsBlock = 1;
                        bots(bot).IRsensors(4,sensorPos(i)) = 1;
                        if world.obstacleDetect(1,bot) == 0 || world.obstacleDetect(1,bot) == -1
                            world.obstacleDetect(1,bot) = -1;
                        elseif world.obstacleDetect(1,bot) == 1
                            world.obstacleDetect(1,bot) = 2; 
                        end
                    end
                end
        end
    end
    %identifies transitons that need to be removed (if necessary)
    if botBlock == 1 || obsBlock == 1
        if isempty(both)
            moves = 0;
        else
            moves(2) = 0;
        end
    end
end
end