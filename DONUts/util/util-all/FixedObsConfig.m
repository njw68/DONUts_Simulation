%FixedObsConfig
function world = FixedObsConfig(I)
    hold on
    %plot the goal
    world.goal = rectangle('Position',[(I.goal(1)-.2) (I.goal(2)-.2) .55 .55],'FaceColor',[0 1 0]);
    %plots either the image or polygon obstacle
    if I.obs==0
        %plot image obstacle
        %Create binary image and create a matrix of the shaded regions to plot
        O = rgb2gray(I.obsImage);
        BW = imbinarize(O);
        [cols,rows] = find(BW==0);
        B = [rows,cols]*.1; %xscale pix/world
        Bshift=[];
        for j=1:size(B,1)
            Bshift(end+1,1) = B(j,2)+I.posobs(1);
            Bshift(end,2) = B(j,1)+I.posobs(2);
        end
        world.obstacles = Bshift;
    %     plot(Bshift(:,1),Bshift(:,2),'kx')
    elseif I.obs==1
        %plot polygon obstacle/s
        t = 0.05:0.5:2*pi;
        obstacles=[];
        for i=1:size(I.posobs,1)
            x = I.posobs(i,1) + cos(t);
            y = I.posobs(i,2) + sin(t);
            obs = polyshape(x,y);
            pg = plot(obs);
            pg.FaceColor = [1 0 0];
            obstacles(:,1,i) = x;
            obstacles(:,2,i) = y;
        end
        world.obstacles = obstacles;
    elseif I.obs == 2
        load(I.matfile,'obstacle');
        obstacle = obstacle/23;
        obstacles=[];
        for i=1:size(obstacle,1)
            for j=1:size(obstacle,3)
                obstacles(j,1,i) = obstacle(i,1,j);
                obstacles(j,2,i) = obstacle(i,2,j);
            end
        end

        world.obstacles = obstacles;
        
        for i=1:size(world.obstacles,3)
            aobstacle(1,:) = world.obstacles(:,1,i);
            aobstacle(2,:) = world.obstacles(:,2,i);
            fill(aobstacle(1,:), aobstacle(2,:),'r')
            hold on
            plot([transpose(aobstacle(1,:)); aobstacle(1,1)], [transpose(aobstacle(2,:)); aobstacle(2,1)],'b')
            hold on
        end
    elseif I.obs == 3
        world.obstacles = [];
    end
end