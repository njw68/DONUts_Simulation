%Faf1
function state = Faf1(bots,world,I)
    stateNum = 1; parent = 0;
    I.first = 0;
    %save the first states' variables 
    stateNum = 1; parent = 0;
    state(1).state = stateNum; state(1).parent = parent; state(1).bots = bots; state(1).world = world;
    state(1).moverBot = 0; state(1).I = I; 
    %move one bot at a time until the light source is reached
    prevdir=1;
    prevMoverBot = 0;
    converge=0;
    direction = [];
    blockDir = [0 0];
    oneBotReach = 0;
    tic
    while converge ~= 1 %COM   
        I.state = stateNum;
        magNum = [];
        %ensure if the bot no longer has a transition, a new bot will be
        %selected as the mover
        while isempty(magNum) && converge ~= 1
            %find the farthest bot away
            [moverBot,closeBot,distance,~,~] = DistanceCOM(state(end),I,I.goal);
            dist = distance;
            %if it can't move, find the next farthest away bot that can
            while world.canMove(moverBot) == 0
                
                dist(moverBot) = 0; %remove the farthest bot from the distance list
                [~,moverBot] = max(dist);
                %if no bots can move, end the simulation
                if isempty(find(world.canMove,1))
                    disp('No modules can move. Ending simulation');
                    converge = 1;
                    magNum = 13;
                    break
                end
            end

            
            %code to move a random module
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            x = rand; 
            if x <= 0.2 %constant
                movableBots = find(world.canMove==1);  
                i2 = find(movableBots==moverBot);  
                movableBots(i2) = [];  
                if length(movableBots) > 1  
                    k10 = randi([1 length(movableBots)]);  
                    moverBot = movableBots(k10);  
                elseif ~isempty(movableBots)  
                    moverBot = movableBots;  
                end
                blockDir = [0 0];  
                %find the transition magnets and determine which direction is best
                [magNum,direction,prevdir,~,~] = ShortestPath(bots,world,I,moverBot,closeBot,blockDir,prevdir,direction);
                bots(moverBot).EPMs(5,magNum) = direction;  
                
                %select the epm it moves on
                for j=1:I.numEPMs  
                    if bots(moverBot).EPMs(5,j) ~= 0
                        magNum = j;  
                        if bots(moverBot).EPMs(5,j) == 2 %if can go both directions, pick CW const
                            bots(moverBot).EPMs(5,j) = 1;  
                        end
                        break
                    end
                end
                %move the random module
                bots = MoveBot(bots,I,moverBot,magNum);
                [bots,I] = GeoSensorEPMPositions(bots,I,moverBot); %updates the geometry, EPM, and sensor locations of the moved bot
                bots = ConnectionDetermination(bots,I);%determines which bots are now connected to eachother
                [bots,world,I] = TransitionDetermination(bots,world,I); %determines the allowed transitions for all bots
                parent = stateNum; stateNum = stateNum+1;  
                %save new variables
                state(stateNum).state = stateNum; state(stateNum).parent = parent; state(stateNum).bots = bots; state(stateNum).world = world;  
                state(stateNum).moverBot = moverBot; state(stateNum).I = I;  
                %calculate distance from collective COM to target
                [farBot,closeBot,dist,COM,mCOM] = DistanceCOM(state(end),I,I.goal);  
                %save more variables
                I.time = toc;
                state(stateNum).COM = COM; state(stateNum).mCOM = mCOM;  
            end    
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if converge == 0  
                %make sure the moverBot and closeBot aren't the same
                if closeBot == moverBot  
                    distance(moverBot) = Inf; %remove the moverBot bot from the distance list const
                    [~,targetBot] = min(distance);  
                else
                    targetBot = closeBot;  
                end
                
                %find the moverBot transition epms and determine which direction is best
                [magNum,direction,prevdir,blockDir,closeMag] = ShortestPath(bots,world,I,moverBot,closeBot,blockDir,prevdir,direction);
                bots(moverBot).EPMs(5,magNum) = direction;  
                prevMoverBot = moverBot;  
                
                if pdist2([bots(targetBot).EPMs(2,closeMag) bots(targetBot).EPMs(3,closeMag)],I.goal,'euclidean') < 0.99
                    oneBotReach = 1;
                end
                
                %%%%%%%%%%%%%%%%%%%%
                atCloseMag = 0;  
                %keep moving the moverBot until it is closest
                switchedDir = 0;  
                epmDistList = [];  
                A = I.numBots*pi*((I.botDiam)^2);  
                epmRadius = (sqrt(A/pi)*.75);  
                
                while ((~isempty(magNum)) && (atCloseMag == 0) && (magNum ~= 13)) && (converge ~= 1)
                    %find magNum of the moverBot, set it to the same direction
                    magNum = [];  
                    
                    for k=1:I.numEPMs %numEPMs
                        if bots(moverBot).EPMs(5,k) == direction || bots(moverBot).EPMs(5,k) == 2  
                            magNum = k;  
                            bots(moverBot).EPMs(5,magNum) = direction;  
                            break
                        end
                    end
                    %check if it can no longer move in the same direction,
                    %if it can't, switch the direction
                    if isempty(magNum)  
                        for k=1:I.numEPMs %numEPM
                            if bots(moverBot).EPMs(5,k) == -direction  
                                magNum = k;  
                                direction = -direction;  
                                switchedDir = switchedDir + 1;  
                            end
                        end
                    end
                    %code to alternate between i and ii
                    y = rand;  
                    if y < .5  
                        toggle = 1;  
                    else
                        toggle = 3;  
                    end
                    if switchedDir >= toggle %toggle 1 and 3 to try different livelock fixes const
                        magNum = [];  
                    end
                    prevMoverBot = moverBot;  
                    
                    %if magNum is empty, this means the current moverBot
                    %can no longer move in any direction, and this section
                    %is skipped
                    if ~isempty(magNum)  
                        if magNum ~= 13  
                            %move the bot
                            bots = MoveBot(bots,I,moverBot,magNum);
                            [bots,I] = GeoSensorEPMPositions(bots,I,moverBot); %updates the geometry, EPM, and sensor locations of the moved bot
                            bots = ConnectionDetermination(bots,I);%determines which bots are now connected to eachother
                            [bots,world,I] = TransitionDetermination(bots,world,I); %determines the allowed transitions for all bots
                            parent = stateNum; stateNum = stateNum+1;  
                            I.state = stateNum;% might mess things up const
                            %plot state
%                             hold on
                            circle = viscircles([bots(targetBot).EPMs(2,closeMag) bots(targetBot).EPMs(3,closeMag)],epmRadius);  
                            cx = circle.Children(1).XData;  
                            cy = circle.Children(1).YData;  
                            C = [cx;cy];  
                            
                            %save new variables
                            I.time = toc;
                            state(stateNum).state = stateNum; state(stateNum).parent = parent; state(stateNum).bots = bots; state(stateNum).world = world;  
                            state(stateNum).moverBot = moverBot; state(stateNum).I = I; 
                            %calculate distance from collective COM to target
                            [farBot,closeBot,dist,COM,mCOM] = DistanceCOM(state(end),I,I.goal);  
                            %save more variables
                            state(stateNum).COM = COM; state(stateNum).mCOM = mCOM;  
                            
                            %convergence check
                            converge = ConvergeCheck(state(end),I,C);
                        end
                    end
                    %all of this is checking if the moverBot has reached
                    %the closest possible position to the top of the
                    %gradient
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %check if the close bot has reached its goal
                    atCloseMag = 0;
                    %check if the close bot has a neighbor at the close magnet
                    if oneBotReach == 0 %if no bots are close to the goal, keep moving the module until it reaches the closeMag b/c it is free
                        if bots(targetBot).EPMs(4,closeMag) == moverBot
                            atCloseMag = 1;
                        end
                    else
                        values = inpolygon(bots(moverBot).coordinates(1),bots(moverBot).coordinates(2),C(1,:),C(2,:));  
                        if isempty(find(values==1, 1)) %if the moverBot's coords aren't inside of the circle
                            atCloseMag = 0;
                        else
                            atCloseMag = 1;
                        end
                    end
                    
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    converge = ConvergeCheck(state(end),I,C);
                    ti = toc;
                    if ti >= 5400 %1.5 hr
                        converge = 1;
                        disp('simulation timed out')
                    end
                    
                end
                %convergence check
                converge = ConvergeCheck(state(end),I,C);
                
                if isempty(magNum)
                    world.canMove(moverBot) = 0;
                end
            end
            t = toc;
            if t >= 5400 %1.5 hr
                converge = 1;
                disp('simulation timed out')
            end
        end
        if magNum == 13
            
            converge = 1;
        end
        
    end
    midState1 = round(length(state)/4);
    midState2 = 2*round(length(state)/4);
    endState = length(state);
    Plot2(state(1).bots,state(1).world,state(1).I,state(1).state,state(1).parent,0,0)
    Plot2(state(midState1).bots,state(midState1).world,state(midState1).I,state(midState1).state,state(midState1).parent,0,0);
    Plot2(state(midState2).bots,state(midState2).world,state(midState2).I,state(midState2).state,state(midState2).parent,0,0);
    Plot2(state(endState).bots,state(endState).world,state(endState).I,state(endState).state,state(endState).parent,0,0);
end