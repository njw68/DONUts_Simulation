%Metrics
function I = Metrics(plan,I)

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
centf = cen(end);

%plots the euclidean distance from each bot and the centroid to the goal vs
%time
I = Distance(plan,I);
name = strcat(I.pathname,I.configName);
save(name, 'plan','BotMoves','TotalMoves','AvgBotMoves','MostMoves','LeastMoves','TotalMsgs','Dist','cen','centf')
close;