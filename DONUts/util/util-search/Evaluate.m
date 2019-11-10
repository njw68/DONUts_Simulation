%Evaluation
%%This function moves the currentBot in the determined direction 
%%one step forward and determines if it is a new state.
function [children,visited,I,stack] = Evaluate(bots,world,I,currentBot,magNum,visited,children,parentNum,parent_cost,goal,stack)
    %if a movement is allowed, load all of the state variables from the previous state
    I.state = I.state+1; %increment the state number
    %move the robot about the selected EPM in the allowed direction
    bots = MoveBot(bots,I,currentBot,magNum); %moves the bot that has been determined as the best to move
    [bots,I] = GeoSensorEPMPositions(bots,I,currentBot); %updates the geometry, EPM, and sensor locations of the moved bot
    bots = ConnectionDetermination(bots,I);%determines which bots are now connected to each other
    [bots,world,I] = TransitionDetermination(bots,world,I); %determines the allowed transitions for all bots

    %record the new configuration (child) as a new state and save the state variables
    state.state = I.state; state.parent = parentNum;
    state.bots = bots; state.world = world;
    state.moverBot = currentBot;
    state.cost = parent_cost +1;
    state.I = I;
    cost = state.cost;
    %calculate distance from collective COM to target
    [~,~,~,COM,fDist,mDist] = DistanceCOM(state,I,goal);
    state.COM = COM; state.mCOM = mDist; state.fDist = fDist;
    %calculate redundancy factor
    redun = Redundancy(bots,I);
    state.redun = redun;
    %calculate predicted remaining cost
    if I.heuristic == 2 %Dmin
        hopDiam=4;
        pcost = (mDist/I.botDiam)*hopDiam*I.numBots; %(closestBot distance from goal/botDiam/2)*hopRadius*numBots
        tcost = (cost + pcost + redun);
    elseif I.heuristic == 2.5 %Dcom centralized
        hopDiam=4;
        pcost = ((I.c*COM)/I.botDiam)*hopDiam*I.numBots; %(closestBot distance from goal/botDiam/2)*hopRadius*numBots
        tcost = cost + pcost + redun;
    elseif I.heuristic == 7 %Dcom
        hopDiam=4;
        pcost = ((COM)/I.botDiam)*hopDiam*I.numBots;
        tcost = cost + pcost + redun;
    end
    state.cost = cost;
    state.pcost = pcost;
    state.tcost = tcost;

    newStateAddedToStack = 0;
    existingStateVisited = 0;
    equalOrLesserState = 0;
    if ~isempty(visited)
        %check if the state already exists
        exstng_state = 0;
        [sortCoords,usc] = SortCoordinates(state,1,I);
        state.sortCoords = sortCoords;
        state.unsortCoords = usc;
        state.min = sortCoords(1,:);
        state.max = sortCoords(end,:);
        %check if new state already exists in visited
        for v=1:length(visited)
                    if abs(sortCoords-visited(v).sortCoords) < .1
                        if state.cost >= visited(v).cost
                            existingStateVisited = 1;
                        end
                        v = length(visited)+1; %break
                    end
%             end
        end
        %check if new state already exists in the stack
        for stackPos = 1:length(stack)
            if abs(sortCoords-stack(stackPos).sortCoords) < .1
                if (stack(stackPos).cost > state.cost)
                    %replace the old state with the new
                    %state in the stack
                    newStateAddedToStack = 1;
                    visited(end+1) = stack(stackPos);
                    stack(stackPos) = state;
                    stackPos = length(stack)+1; %break
                else
                    equalOrLesserState = 1;
                end
            end
        end
        %add the new child to the children list (this only happens if it
        %wasn't added directly to the stack in the previous step)
        if existingStateVisited == 0 && newStateAddedToStack == 0 && equalOrLesserState == 0
            if isempty(children)
                children = state;
            else
                children(end+1) = state;
            end
            
        end
    %if visited is empty, this is the first new state    
    else
        %Add the new state to the children list
        if isempty(children)
            children = state;
        else
            children(end+1) = state;
        end
    end
end