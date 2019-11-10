%Plot
function I = Plot(bots,I)
    hold on
    %%Plots the initial position of the robots.
    for i = 1:I.numBots
        amagnet(1,:) = bots(i).EPMs(2,:);
        amagnet(2,:) = bots(i).EPMs(3,:);
        asensor(1,:) = bots(i).IRsensors(2,:);
        asensor(2,:) = bots(i).IRsensors(3,:);   
        axis(I.setAxis);
        plot(bots(i).geometry(:,1),bots(i).geometry(:,2),'k')
        plot(amagnet(1,:),amagnet(2,:),'r.')
        plot(asensor(1,:),asensor(2,:),'b.')
        in = findobj('type','line');
        b = num2str(i);
        tex = text(bots(i).coordinates(1)-.5,bots(i).coordinates(2),b);
        tex.FontSize = 5;
    end
    t = 'State1';
    title(t);
    loc = strcat(I.configName,'-State1-Parent0.jpeg');
    figfile = fullfile(I.pathname, loc);
    saveas(gcf,figfile)
    close
end
