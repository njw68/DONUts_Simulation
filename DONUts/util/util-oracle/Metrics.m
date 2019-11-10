%Metrics
function I = Metrics(plan,childrenPerNode,stack,visited,I)
    x= []; y = []; cen = []; dist = []; Dist = [];
    BotMoves = zeros(I.numBots,1);
    %computs number of moves per bot

    for mbots = 2:length(plan)
        arrow = plan(mbots).moverBot;
        BotMoves(arrow) = BotMoves(arrow)+1;
    end

    %computes the total moves
    TotalMoves = length(plan)-1;

    %computes the average moves made per bot
    AvgBotMoves = sum(BotMoves)/I.numBots;

    %computes most moves
    MostMoves = max(BotMoves);
    LeastMoves = min(BotMoves);

    %computes the total number of messages
    %for each move, a message must be sent from the central controller to two
    %bots, instructing them to turn on/off a magnet
    TotalMsgs = TotalMoves*2;

    for mbots =2:length(plan)
        for ind = 1:I.numBots
            %get x and y coordinates of each bot in the current time step
            x(1,ind) = plan(mbots).bots(ind).coordinates(1);
            y(1,ind) = plan(mbots).bots(ind).coordinates(2);
            %calculate each bot's distance from the goal
            dist(1,ind) = pdist2(I.goal,plan(mbots).bots(ind).coordinates);
        end

        %find the closest and farthest bots in the current time step
        Dist(mbots,1) = min(dist); Dist(mbots,2) = max(dist);

        %add the centroid in the current time step to an array
        cen(end+1) = plan(mbots).COM;
    end
    centf = 0;%cen(end);

    %Calculate effective branching factor
    b_eff = EffectiveBranching(plan(end).state,length(plan),I.numBots*10);
    
    %plots the euclidean distance from each bot and the centroid to the goal vs
    %time
    I = Distance(plan,childrenPerNode,I);
    name = strcat(I.pathname,I.configName);

    %calculate children stats
    avgChildrenPerNode = mean(childrenPerNode(:,2));
    maxChildren = max(childrenPerNode(:,2));
    minChildren = min(childrenPerNode(:,2));
    
    %save variables
    save(name, 'plan','BotMoves','TotalMoves','AvgBotMoves','MostMoves','LeastMoves','TotalMsgs','Dist','cen','centf','b_eff','visited','stack','childrenPerNode','avgChildrenPerNode','maxChildren','minChildren') 
end