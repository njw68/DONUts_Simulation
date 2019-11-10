%Reconstruct (Oracle)
function plan = Reconstruct(plan,state,I)
    while plan(1).parent ~= 0
        for i=1:length(state)
            if state(i).state == plan(1).parent
                plan = [state(i),plan];
                i = length(state) + 1;
            end
        end
    end
%     
%     seg = round(length(plan)/4);
%     Plot2(plan(1).bots,plan(1).world,I,plan(1).state,plan(1).parent,plan(1).cost,plan(1).tcost);
%     Plot2(plan(seg).bots,plan(seg).world,I,plan(seg).state,plan(seg).parent,plan(seg).cost,plan(seg).tcost);
%     Plot2(plan(seg*2).bots,plan(seg*2).world,I,plan(seg*2).state,plan(seg*2).parent,plan(seg*2).cost,plan(seg*2).tcost);
%     Plot2(plan(end).bots,plan(end).world,I,plan(end).state,plan(end).parent,plan(end).cost,plan(end).tcost);
end