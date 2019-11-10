%SortCoordinates
%sorts the xycoords of a state
function [sa,nsa] = SortCoordinates(state,ind,I)
    for i=1:I.numBots
        nsa(i,1) = state(ind).bots(i).coordinates(1);
        nsa(i,2) = state(ind).bots(i).coordinates(2);
    end
    nsa = round(nsa,4);
    sa = sortrows(nsa);
end