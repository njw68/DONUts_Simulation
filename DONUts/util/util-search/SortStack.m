%SortStack
%sorts a list by total cost
function index = SortStack(state)

for ind=1:length(state)
    nsa1(ind,1) = state(ind).tcost; %element total cost
end

[Min,index] = min(nsa1);
index = index(1);