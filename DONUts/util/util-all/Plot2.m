%Plot
function Plot2(bots,world,I,stateNum,parent,cost,tcost,~,~)
    hold on
    x=zeros(1,I.numBots); y=zeros(1,I.numBots);
    %%Plots the new position of the robots.
    for i = 1:I.numBots
        x(i) = bots(i).coordinates(1);
        y(i) = bots(i).coordinates(2);
        amagnet(1,:) = bots(i).EPMs(2,:);
        amagnet(2,:) = bots(i).EPMs(3,:);
        asensor(1,:) = bots(i).IRsensors(2,:);
        asensor(2,:) = bots(i).IRsensors(3,:);
        axis(I.setAxis);
        pbaspect([1 1 1]);
        plot(bots(i).geometry(:,1),bots(i).geometry(:,2),'r')
        plot(amagnet(1,:),amagnet(2,:),'r.')
        plot(asensor(1,:),asensor(2,:),'b.')
        in = findobj('type','line');
        b = num2str(i);
        tex = text(bots(i).coordinates(1)-.5,bots(i).coordinates(2),b);
        tex.FontSize = 5;
    end
    %plot the COM
    plot(mean(x),mean(y),'yx')
    
    %plot the goal
    rectangle('Position',[(I.goal(1)-.2) (I.goal(2)-.2) .55 .55],'FaceColor',[0 1 0]);
    % %plots either the image or polygon obstacle
    if I.obs==0
        %plot image obstacle
        plot(world.obstacles(:,1),world.obstacles(:,2),'kx')
        hold off
    elseif I.obs==1
        %plot polygon obstacle/s
        for i=1:size(I.posobs,1)
             x = world.obstacles(:,1,i);
             y = world.obstacles(:,2,i);
             obs = polyshape(x,y);
             pg = plot(obs);
             pg.FaceColor = [1 0 0];
        end
    elseif I.obs == 3
        %do nothing
    else
        for i=1:size(world.obstacles,3)
            aobstacle(1,:) = world.obstacles(:,1,i);
            aobstacle(2,:) = world.obstacles(:,2,i);
            fill(aobstacle(1,:), aobstacle(2,:),'r')
            hold on
            plot([transpose(aobstacle(1,:)); aobstacle(1,1)], [transpose(aobstacle(2,:)); aobstacle(2,1)],'b')
            hold on
        end
    end
    %plot title, save to file, and close
    t = strcat('State',num2str(stateNum),'-','Parent',num2str(parent),'-',num2str(cost),'-',num2str(round(tcost,2)));
    title(t);
    loc = strcat(I.configName,'-',t,'.jpeg');
    figfile = fullfile(I.pathname, loc);
    
    saveas(gcf,figfile)
    close
end
