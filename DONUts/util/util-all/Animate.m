%Animate
function plan = Animate(plan,I)
close all;

vi = strcat(I.pathname,I.planname);
I.vid = VideoWriter(vi);
I.vid.FrameRate=10;
open(I.vid)
hold on
world = plan(1).world;
for f=1:length(plan)
    hold on
    stateNum = plan(f).state;
    parent = plan(f).parent;
    bots = plan(f).bots;
    moverBot = plan(f).moverBot; 
    PlotA(bots,world,I,stateNum,parent,moverBot)
end
close(I.vid)

end