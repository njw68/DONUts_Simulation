function [botPlan,bots,world] = CenA(bots,world,I)
    tic
    I.timer = [];
    stateNum = 1; parent = 0;
    %save the first states' variables 
    botPlan(1).state = stateNum; botPlan(1).parent = parent; botPlan(1).bots = bots; botPlan(1).world = world; botPlan(1).moverBot = 0;
    botPlan(1).redun = 0; botPlan(1).cost = 0; botPlan(1).tcost = 0; botPlan(1).pcost = 0; botPlan(1).COM = 0; botPlan(1).mCOM = 0;
    botPlan(1).I = I;
    [sc,usc] = SortCoordinates(botPlan,1,I);
    botPlan.sortCoords = sc;
    botPlan.unsortCoords = usc;
    botPlan.min = [sc(1,1),min(sc(:,2))];
    botPlan.max = [sc(end,1),max(sc(:,2))];
    botPlan.fDist = 0;
    %move one bot at a time until the light source is reached
    %identify the farthest & closest bots to the target & the COM
    goal = I.goal;
    converge=0;
    I.test = 0;
    livelock = 0;
    
    %determine the obstacles currently visible in the first state 
    while converge ~= 1
        %%find the bot with the lowest gradient
        [farBot,lowBot,dist] = DistanceCOM(botPlan(end),I,goal);
        I.tempgoal = bots(lowBot).coordinates; %tempgoal is not 0 so the current seenObs will not get updated
        I.goalBot = lowBot;
        bots(I.goalBot).EPMs(5,:) = 0;
        I.replan = 1;
        t1 = toc;
        I.timer(end+1) = t1;
        if sum(I.timer) >= 7200 %2 hr
            disp('simulation timed out')
            I.replan = 0;
            converge = 1;
        end
        while I.replan == 1
            %find the closest magnet to the light source
            marksAr=[];
            for i=1:I.numEPMs
                marksAr(end+1,1) = bots(lowBot).EPMs(2,i);
                marksAr(end,2) = bots(lowBot).EPMs(3,i);
            end
            d = pdist2(I.goal,marksAr,'euclidean');
            [~,closeMag]=min(d);
            %determine convergence radius
            A = I.numBots*pi*((I.botDiam)^2);
            epmRadius = sqrt(A/pi)*.75;

            %pass current world, current bots, return future states and win state
            [win_state,visited,~,~,~,~,~,iteration] = Astar(bots,world,I,botPlan(end),lowBot,closeMag,epmRadius);
            %reconstruct the predicted path
            path = Reconstruct(win_state,visited);
            I.replan = 0;
            i = 1;
            %iterate through the path and add to the bot plan if no new
            %obstacles are detected
            while i <= length(path)
                moverBot = path(i).moverBot; %path starts at the next step from the current config
                oldObs = zeros(1,I.numObstacles);
                for k=1:I.numBots
                    if world.obstacleDetect(2,k) ~= 0
                        oldObs(world.obstacleDetect(2,k)) = world.obstacleDetect(2,k);
                    end
                end
                %save current seen obs
                seenObs = I.seenObs;
                %let all obstacles be seen
                I.seenObs = 1:size(world.obstacles,3);
                %do obstacle detection for all bots in the current state
                [bots,newworld,I] = ObstacleAvoidance(path(i).bots,path(i).world,I);
                %reset the seen obstacles
                I.seenObs = seenObs;
                newObs = zeros(1,I.numObstacles);
                for j=1:size(world.obstacleDetect,2)
                    if newworld.obstacleDetect(2,j) ~= 0
                        newObs(newworld.obstacleDetect(2,j)) = newworld.obstacleDetect(2,j);
                    end
                end
                oldLenSeenObs = length(I.seenObs);
                for m=1:length(newObs)
                    if newObs(m) ~= 0
                        if isempty(find(I.seenObs == newObs(m)))
                            I.seenObs(end+1) = newObs(m);
                            disp('new obstacle added to I.seenObs');
                        end
                    end
                end
                %if there are no new obstacles detected, move the moverBot
                %forward 1 step
                if oldLenSeenObs == length(I.seenObs)
                    parent = stateNum; stateNum = stateNum+1;
                    I.state = stateNum; I.parent = parent;
                    bots = path(i).bots; world = path(i).world;
                    botPlan(stateNum) = path(i);
                %if not, update the world and quit moving bots through the
                %predetermined path, prepare to recalculate a path
                else
                    I.test = 1;
                    world = newworld;
                    I.replan = 1;
                    i = length(path);
                end
                i = i+1;
            end
            %check for global convergence
            I.algorithm = 0.1; %change to global end criteria
            converge = ConvergeCheck(botPlan(end),I,I);
            if converge == 1
                I.replan = 0;
            end
            I.algorithm = 2; %change back to local criteria
            
            %%find the bot with the lowest gradient            
            [farBot,lowBot,dist,comDist] = DistanceCOM(botPlan(end),I,goal);
            %add state to the path
            botPlan(end).COM = comDist/I.botDiam;
            %check for livelock
            if iteration == 1
                disp('WARNING: modules may have entered livelock')
                livelock = livelock+1;
            end
            
            if livelock > 1
                disp('livelock entered. moving random module')
                %select random module and epm position
                botsRandom = RandomBotOrderGenerator(I.numBots);
                for k=1:I.numBots
                    if world.canMove(botsRandom(k)) == 1 && botsRandom(k) ~= moverBot
                        currentBot = botsRandom(k);
                        disp(['random module: ' num2str(currentBot)]);
                        break 
                        
                    end
                end
                %select the epm it moves on
                for j=1:I.numEPMs
                    if bots(currentBot).EPMs(5,j) ~= 0
                        magNum = j;
                        if bots(currentBot).EPMs(5,j) == 2 %if can go both directions, pick CW
                            bots(currentBot).EPMs(5,j) = 1;
                        end
                        break
                    end
                end
            
                %move random module here
                stateNum = stateNum+1; %increment the state number
                
                %move the module about the selected EPM in the allowed direction
                bots = MoveBot(bots,I,currentBot,magNum); %moves the bot that has been determined as the best to move
                [bots,I] = GeoSensorEPMPositions(bots,I,currentBot); %updates the geometry, EPM, and sensor locations of the moved bot
                bots = ConnectionDetermination(bots,I);%determines which bots are now connected to each other
                [bots,world,I] = TransitionDetermination(bots,world,I); %determines the allowed transitions for all bots

                %record the new configuration (child) as a new state and save the state variables
                state.state = stateNum; state.parent = botPlan(end).state;
                state.bots = bots; state.world = world;
                state.moverBot = currentBot;
                state.cost = botPlan(end).cost +1;
                state.I = I;
                cost = state.cost;
                %calculate distance from collective COM to target
                [farBot,closeBot,dist,COM,fDist,mDist] = DistanceCOM(state,I,I.goal);
                state.COM = COM; state.mCOM = mDist; state.fDist = fDist;
                %calculate redundancy factor
                redun = Redundancy(bots,I);
                state.redun = redun;
                %calculate predicted remaining cost
                %Dcom centralized
                hopDiam=4;
                pcost = ((I.alpha*COM)/I.botDiam)*hopDiam*I.numBots; %(closestBot distance from goal/botDiam/2)*hopRadius*numBots
                tcost = cost + pcost + redun;
                state.cost = cost;
                state.pcost = pcost;
                state.tcost = tcost;

                [sortCoords,usc] = SortCoordinates(state,1,I);
                state.sortCoords = sortCoords;
                state.unsortCoords = usc;
                state.min = sortCoords(1,:);
                state.max = sortCoords(end,:);
                %%%%%%%%%%%%%%%%%%%%%%%%%
                
                %add new state to botPlan
                botPlan(end+1) = state;                
                %reset livelock
                livelock = 0;
            end
            %check for global convergence
            I.algorithm = 0.1; %change to global end criteria
            converge = ConvergeCheck(botPlan(end),I,I);
            if converge == 1
                I.replan = 0;
            end
            I.algorithm = 2; %change back to local criteria
        end
    end
    %plot time lapse images
%     seg = round(length(botPlan)/4);
%     Plot2(botPlan(1).bots,botPlan(1).world,I,1,0,0,0);
%     Plot2(botPlan(seg).bots,botPlan(seg).world,I,seg,seg-1,0,0);
%     Plot2(botPlan(seg*2).bots,botPlan(seg*2).world,I,seg*2,(seg*2)-1,0,0);
%     Plot2(botPlan(end).bots,botPlan(end).world,I,length(botPlan),length(botPlan)-1,0,0);
end