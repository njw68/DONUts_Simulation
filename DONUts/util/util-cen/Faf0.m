%GradientFollowCOMC1
function state = Faf0(bots,world,I)
    stateNum = 1; parent = 0;
    %save the first states' variables
    stateNum = 1; parent = 0;
    state(1).state = stateNum; state(1).parent = parent; state(1).bots = bots; state(1).world = world;
    direction = [];
    blockDir = [0 0];
    converge = 0;
    prevMoverBot = 0;
    livelock = 0;
    tic
    while converge ~= 1 %COM
        I.state = stateNum;
        magNum = [];
        state(1).moverBot = 0; state(1).I = I;
        %move one bot at a time until the light source is reached

        prevdir=1;
        converge=0;
        while isempty(magNum)
            [moverBot,closeBot,dist,~,~] = DistanceCOM(state(end),I,I.goal);
            while world.canMove(moverBot) == 0 || world.canMove(moverBot) == 3
                dist(moverBot) = 0; %remove the farthest bot from the distance list
                [~,moverBot] = max(dist);
                if isempty(find(world.canMove,1))
                    %move the farthest bot that can move in the opposite direction
                    [moverBot,closeBot,dist,~,~] = DistanceCOM(state(end),I,I.goal);
                    while length(find(bots(moverBot).EPMs(5,:)==0)) == I.numEPMs
                        dist(moverBot) = 0; %remove the farthest bot from the distance list
                    end
                    %if no bot really can move, end simulation
                    if isempty(find(dist,1))
                        disp('No modules can move. Ending simulation');
                        magNum = I.numEPMs+1;
                        converge = 1;
                        break
                    end
                end
            end
            
            if converge == 0
                if moverBot ~= prevMoverBot
                    blockDir = [0 0];
                    %find the transition magnets and determine which direction is best
                    [magNum,direction,prevdir,~,closeMag] = ShortestPath(bots,world,I,moverBot,closeBot,blockDir,prevdir,direction);
                    bots(moverBot).EPMs(5,magNum) = direction;
                    prevMoverBot = moverBot;
                    A = I.numBots*pi*((I.botDiam)^2);
                    epmRadius = sqrt(A/pi)*.75;
                else
                    %find magNum, set it to the same direction
                    canGoBack = 0;
                    for k=1:I.numEPMs
                        if bots(moverBot).EPMs(5,k) ~= 0
                            if bots(moverBot).EPMs(5,k) == direction || bots(moverBot).EPMs(5,k) == 2
                                magNum = k;
                                bots(moverBot).EPMs(5,magNum) = direction;
                            elseif bots(moverBot).EPMs(5,k) == -direction
                                canGoBack = 1;
                            end
                        end
                    end
                    prevMoverBot = moverBot;
                end
                if isempty(magNum)
                    if canGoBack == 0
                        world.canMove(moverBot) = 0;
                    else
                        world.canMove(moverBot) = 3;
                    end
                end
                if ~isempty(magNum) && magNum ~= I.numEPMs+1
                    %move the bot
                    bots = MoveBot(bots,I,moverBot,magNum);
                    [bots,I] = GeoSensorEPMPositions(bots,I,moverBot); %updates the geometry, EPM, and sensor locations of the moved bot
                    bots = ConnectionDetermination(bots,I);%determines which bots are now connected to eachother
                    [bots,world,I] = TransitionDetermination(bots,world,I); %determines the allowed transitions for all bots
                    parent = stateNum; stateNum = stateNum+1;
                    circle = viscircles([bots(closeBot).EPMs(2,closeMag) bots(closeBot).EPMs(3,closeMag)],epmRadius);
                    cx = circle.Children(1).XData;
                    cy = circle.Children(1).YData;
                    C = [cx;cy];
                    %save new variables
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
        end
        t = toc;
        if t > 3600 %1 hr
            converge = 1;
            disp('simulation timed out');
        end
    end
    close all
    midState1 = round(length(state)/4);
    midState2 = 2*round(length(state)/4);
    endState = length(state);
    Plot2(state(1).bots,state(1).world,state(1).I,state(1).state,state(1).parent,0,0);
    Plot2(state(midState1).bots,state(midState1).world,state(midState1).I,state(midState1).state,state(midState1).parent,0,0);
    Plot2(state(midState2).bots,state(midState2).world,state(midState2).I,state(midState2).state,state(midState2).parent,0,0);
    Plot2(state(endState).bots,state(endState).world,state(endState).I,state(endState).state,state(endState).parent,0,0);
end