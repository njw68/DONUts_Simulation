%Redundancy
function redun = Redundancy(bots,I)

redunAr = zeros(2,I.numBots);
%count the number of connections each module has
for i=1:I.numBots
    for j=1:I.numEPMs
        if bots(i).EPMs(4,j) ~= 0 
            redunAr(1,i) = redunAr(1,i)+1;
        end
    end
    if I.numBots >= 6
        if redunAr(1,i) == 2
            redunAr(2,i) = 1;
        elseif redunAr(1,i) == 1
            redunAr(2,i) = 2;
        end
    elseif I.numBots < 6
        if redunAr(1,i) == 1
            redunAr(2,i) = 1;
        end
    end
end
redun = sum(redunAr,2);
redun = redun(2)*I.alpha;
