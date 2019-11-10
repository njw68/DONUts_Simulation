%CheckDuplicates
function [isDup,listID] = CheckDuplicates(list,I,s)
    listCost = [list(:).cost]';
    listX = zeros(length(list),I.numBots);
    listY = zeros(length(list),I.numBots);
    for i=1:length(list)
        listX(i,:) = list(i).sortCoords(:,1)';
        listY(i,:) = list(i).sortCoords(:,2)';
    end
    if s == 1
        listID = [listX,listY];
    else
    listID = [listCost,listX,listY];
    end
    if size(listID) == size(unique(listID,'rows'))
        isDup = 0;
    else
        isDup = 1;
    end
    
end