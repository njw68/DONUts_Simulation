%%Randomly generated obstacles
function [world,I] = RandomObstacleGeneration(bots,I)
world.obstacles=[];
hold on
%plot the goal
world.goal = rectangle('Position',[(I.goal(1)-.2) (I.goal(2)-.2) .55 .55],'FaceColor',[0 1 0]);
N=12; % Number Of Sides To Polygon
%%Resizing constant
resizeConst = 1;
for i = 1:I.numObstacles
    %%Generates random angles along which the vertices of the random
    %%polygon will be placed.
    a = sort(rand(N,1))*2*pi;
    %%Generates random radii for the vertices of the obstacles
    r = randi(1, N, 1);
    validObstacleLocation = 0;
    while(validObstacleLocation == 0)
        %%Generates random x and y coordinates to shift the obstacles to random
        %%locations around the environment
        randX = -I.plotXLimit + 2*I.plotXLimit*rand;
        randY = -I.plotYLimit + 2*I.plotYLimit*rand;
        %%Generates the locations of the vertices for the random polygons.
        obstacle(i,1,:) = cos(a).*r*resizeConst + randX;
        obstacle(i,2,:) = sin(a).*r*resizeConst + randY;
        notValidObstacleLocation = 0;
        for j = 1:length(obstacle(i,1,:))
            for m = 1:I.numBots
                distToFormBots = sqrt(((obstacle(i,1,j) - bots(m).coordinates(1))^2) + ((obstacle(i,2,j) - bots(m).coordinates(2))^2));
                if distToFormBots < (I.botDiam*1.5) || (obstacle(i,2,j) > I.goal(2)) || (obstacle(i,2,j) < I.startCOMY) || (obstacle(i,1,j) > I.startCOMX+(I.numBots*(I.botDiam/2))) || (obstacle(i,1,j) < I.startCOMX-(I.numBots*(I.botDiam/2))) || (obstacle(i,1,j) > I.goal(1)+(I.botDiam*(I.numBots))) %|| (obstacle(i,2,j) < I.startCOMY+(2^4)) %|| (obstacle(i,1,j) < I.goal(1)+(I.botDiam*(I.numBots)))
                    j = length(obstacle(i,1,:));
                    m = I.numBots;
                    %%This means that it is definitely not a valid location
                    %%for an obstacle, it is within the area of the
                    %%obstacles.
                    notValidObstacleLocation = 1;
                end
            end
        end
        %%Checks if it was not proven that the obstacle is close to0 one of
        %%the formbots
        if(notValidObstacleLocation == 0)
            validObstacleLocation = 1;
        end
    end
    aobstacle(1,:) = obstacle(i,1,:);
    aobstacle(2,:) = obstacle(i,2,:);
    world.obstacles(:,1,i) = obstacle(i,1,:);
    world.obstacles(:,2,i) = obstacle(i,2,:);
    
end
hold on
    disp(size(world.obstacles))
    for o=1:size(world.obstacles,3)
        aobstacle(1,:) = world.obstacles(:,1,o);
        aobstacle(2,:) = world.obstacles(:,2,o);
        fill(aobstacle(1,:), aobstacle(2,:),'r')
        hold on
        plot([transpose(aobstacle(1,:)); aobstacle(1,1)], [transpose(aobstacle(2,:)); aobstacle(2,1)],'b')
        hold on
    end

