%Distance
function I = Distance(plan,childrenPerNode,I)
    %finds the distance of each bot and the COM from the goal in each time step
    %and plots it vs time
    D=zeros(length(plan),I.numBots); Davg=[]; bot = 'bot'; t = 1:length(plan); coords = []; botname = [];
    for i = 1:length(plan)
        coords = [];
        for j=1:I.numBots
            coords(end+1,1) = plan(i).bots(j).coordinates(1);
            coords(end,2) = plan(i).bots(j).coordinates(2);
        end
        
        for k=1:I.numBots
            d = pdist2(I.goal,coords(k,:),'euclidean')/I.botDiam;
            D(i,k) = d;
        end
        Davg(end+1) = pdist2([mean(coords(:,1)) mean(coords(:,2))],I.goal,'euclidean')/I.botDiam;
    end
    for i=1:I.numBots
       num = num2str(i);
       botname(end+1).s = strcat(bot,num);
    end
    for i=1:length(Davg)
        I.RandomConfigData(i,1,I.ranConfig) = i;
        I.RandomConfigData(i,2,I.ranConfig) = Davg(i);
    end
    hold on
    plot(t,Davg,'x')
        hold on
        for j=1:I.numBots
            hold on
            plot(t,D(:,j))
        end
    xlabel('No. of Module Moves');
    ylabel('Distance to Goal (Module Diameters)');
    hold off
    name = strcat(I.pathname,I.configName);
    plotName = strcat(I.pathname,I.configName,'-plotdata','.fig');
    saveas(gcf,name,'jpeg');
    savefig(plotName);
    
    figure
    plot(1:size(childrenPerNode,1),childrenPerNode(:,2),'x')
    xlabel('Node')
    ylabel('No. Children')
    saveas(gcf,strcat(I.pathname,I.configName,'-ChilVsNodes.jpeg'))
    savefig(strcat(I.pathname,I.configName,'-ChilVsNodes.fig'))
    figure
    hold on
    plot(childrenPerNode(:,5),childrenPerNode(:,4),'ok')
    plot(childrenPerNode(:,5),childrenPerNode(:,3),'xb')
    xlabel('Dcom')
    ylabel('No. States')
    lgd = legend('stack','visited');
    lgd.Location = 'northwest';
    h = gca;
    axis([0 I.numBots h.YLim(1) h.YLim(2)]);
    set(gca, 'XDir', 'reverse')
    saveas(gcf,strcat(I.pathname,I.configName,'-DcomvsSV.jpeg'))
    savefig(strcat(I.pathname,I.configName,'-DcomvsSV.fig'))
    hold off
end