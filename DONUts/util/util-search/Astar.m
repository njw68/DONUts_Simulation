%%Astar
%%A* based search to find the optimal path to the goal in terms of the
%%fewest number of moves
function [win_state,visited,stack,childrenPerNode,bots,world,I,int22] = Astar(bots,world,I,state,lowBot,~,~)
    tic
    I.testing = 0;
    %initialize variables
    childrenPerNode = [];
    converge=0;
    I.cost = 0;
    if I.tempgoal == 0
        I.state = 1;
        parent = 0;
        goal = I.goal;
    else
        goal = I.tempgoal;
        state.state = 1;
        state.parent = 0;
    end
    Tol = I.botDiam-.2;
    exstng_state=0;
    children = [];
    %storing the initial configuration variables in the state struct
    if I.tempgoal == 0
        state.state = I.state; state.parent = parent; state.I = I;
        state.bots = bots; state.world = world; state.moverBot = 0;
    end
    state.redun = 0; state.cost = 0; state.tcost = 0.000; state.pcost = 0.000;
    [farBot,closeBot,dist,COM,fDist,mDist] = DistanceCOM(state,I,goal);
    state.COM = COM; state.mCOM = mDist; state.fDist = fDist;
    [sc,usc] = SortCoordinates(state,1,I);
    %put state one sorted and unsorted coordinates in visited list
    state.sortCoords = sc;
    state.unsortCoords = usc;
    %find bounding box of this state
    state.min = [sc(1,1),min(sc(:,2))];
    state.max = [sc(end,1),max(sc(:,2))];
    stack = state; %put state one in the stack
    visited = state; %put state 1 in visited
    int22 = 0;
    while converge ~= 1
        int22 = int22+1;
        t = toc;
        if t >= 3600 %1 hr
            stack = [];
            disp('simulation timed out');
        end
        if isempty(stack) %if stack is empty and a solution hasn't been reached, exit
            converge = 1;
            [mi,visI] = min([visited(:).COM] + [visited(:).fDist]);
            win_state = visited(visI);
        else %else, find the lowest costing state in the stack
            if length(stack) > 1
                if I.debugOn == 1
                    %check for duplicate states in stack
                    [isDupS,stackID] = CheckDuplicates(stack,I,1);
                    if isDupS == 1
                        n = length(stackID) - length(unique(stackID,'rows'));
                        if n > 1
                            disp(['WARNING: ' num2str(n) ' repeated states found in stack at iteration ' num2str(int22)]);
                            disp(['len stack: ' num2str(length(stack))]);
                            disp(['len visited: ' num2str(length(visited))]);
                            save(strcat(I.pathname,'stackID',I.configName),'stackID');
                        end
                    end
                    %check for duplicate states in visited
                    [isDupV,visitedID] = CheckDuplicates(visited,I,0);
                    if isDupV == 1
                        n = length(visitedID) - length(unique(visitedID,'rows'));
                        if n > 1
                            disp(['WARNING: ' num2str(n) ' repeated states found in visited at iteration ' num2str(int22)]);
                            disp(['len visited: ' num2str(length(visited))]);
                            save(strcat(I.pathname,'visitedID',I.configName),'visitedID');
                        end
                    end
                end
                %find lowest tcost in stack and select this state to expand
                [m,Is] = min([stack(:).tcost]);
            elseif length(stack)==1
                Is = 1;
            end
            if int22 ~= 1
                visited(end+1) = stack(Is); %add the parent to the visited list
                %(skip for 1st state b/c it was added already to establish the visited list)
            end
            parent = stack(Is); %look at the first state in the sorted stack
            bots = stack(Is).bots; %load the bots of the state
            world = stack(Is).world; %load the world of the state
            parentNum = stack(Is).state; %set the current state to the chosen state
            cost = stack(Is).cost;
            stack(Is) = []; %remove parent from the stack
            for currentBot = 1:I.numBots %iterate through each bot in the collective in the state
                if world.canMove(currentBot) == 1 %if it can move in this time step
                    if I.tempgoal == 0 %for oracle, move all possible modules
                        for magNum = 1:I.numEPMs %iterate through all of its magnets
                            %check whether or not a movement is allowed at this magnet
                            if parent.bots(currentBot).EPMs(5,magNum) == 1 || parent.bots(currentBot).EPMs(5,magNum) == -1
                                [children,visited,I,stack] = Evaluate(bots,world,I,currentBot,magNum,visited,children,parentNum,cost,goal,stack);
                            elseif parent.bots(currentBot).EPMs(5,magNum) == 2 %check whether or not a movement is allowed at this magnet
                                bots(currentBot).EPMs(5,magNum) = 1; %check move in the CW direction
                                [children,visited,I,stack] = Evaluate(bots,world,I,currentBot,magNum,visited,children,parentNum,cost,goal,stack);
                                bots(currentBot).EPMs(5,magNum) = -1; %check move in the CCW direction
                                [children,visited,I,stack] = Evaluate(bots,world,I,currentBot,magNum,visited,children,parentNum,cost,goal,stack);
                            end
                        end
                    else %for centralized, only move this module if it's not the current lowBot
                        if currentBot ~= lowBot
                            for magNum = 1:I.numEPMs %iterate through all of its magnets
                                %check whether or not a movement is allowed at this magnet
                                if parent.bots(currentBot).EPMs(5,magNum) == 1 || parent.bots(currentBot).EPMs(5,magNum) == -1
                                    [children,visited,I,stack] = Evaluate(bots,world,I,currentBot,magNum,visited,children,parentNum,cost,goal,stack);
                                elseif parent.bots(currentBot).EPMs(5,magNum) == 2 %check whether or not a movement is allowed at this magnet
                                    bots(currentBot).EPMs(5,magNum) = 1; %check move in the CW direction
                                    [children,visited,I,stack] = Evaluate(bots,world,I,currentBot,magNum,visited,children,parentNum,cost,goal,stack);
                                    bots(currentBot).EPMs(5,magNum) = -1; %check move in the CCW direction
                                    [children,visited,I,stack] = Evaluate(bots,world,I,currentBot,magNum,visited,children,parentNum,cost,goal,stack);
                                end
                            end
                        end
                    end
                end
            end
            if I.debugOn == 1
                %keep list of no. children per expanded node and length of
                %stack and visited and dcom at each iteration
                childrenPerNode(int22,1) = parentNum;
                childrenPerNode(int22,2) = length(children);
                childrenPerNode(int22,3) = length(stack);
                childrenPerNode(int22,4) = length(visited);
                childrenPerNode(int22,5) = parent.COM/I.botDiam;
            end
            %add its children to the bottom of the stack
            stack = [stack,children];
%             if mod(int22,1000) == 0
%                 disp(['plotting children from iteration' num2str(int22)]);
%             end
            %exit condition
            for new_child=1:length(children)
                if I.tempgoal == 0
                    [converge,stdD] = ConvergeCheck(children(new_child),I,I);
                else
                    [converge,stdD] = ConvergeCheck(children(new_child),I,I);
                end
%                 if mod(int22,1000) == 0
%                     disp(['state: ' num2str(children(new_child).state) ' dcom: ' num2str(children(new_child).COM/I.botDiam) ' stdD: ' num2str(stdD)])
%                     disp('goal:'); disp(goal);
%                 end
                if converge == 1
                    win_state = children(new_child); %store winning state
%                     disp(['visited len: ' num2str(length(visited)) ' stack len: ' num2str(length(stack))]);
%                     disp(['Astar winning state: ' num2str(win_state.state) ' found at iteration: ' num2str(int22)]);
                    break
                end
            end
            children = [];
        end

    end
end