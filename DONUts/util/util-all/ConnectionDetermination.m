%%ConnectionDetermination
%%This function determines which bots are connected to each other in the
%%current time step
function bots = ConnectionDetermination(bots,I)
tol = 0.05;
for cbot = 1:I.numBots
        for cmag = 1:I.numEPMs
            for obot = 1:I.numBots
                for omag = 1:I.numEPMs
                    if (bots(cbot).EPMs(2,cmag) <= bots(obot).EPMs(2,omag) + tol) && (bots(cbot).EPMs(2,cmag) >= bots(obot).EPMs(2,omag) - tol) &&...
                       (bots(cbot).EPMs(3,cmag) <= bots(obot).EPMs(3,omag) + tol) && (bots(cbot).EPMs(3,cmag) >= bots(obot).EPMs(3,omag) - tol) && (obot ~= cbot)
                        bots(cbot).EPMs(4,cmag) = obot; %cbot is attached to obot at cmag
                        if omag == I.numEPMs
                            bots(obot).EPMs(4,omag) = cbot;
                        end
                        cmag = I.numEPMs;
                    else
                        bots(cbot).EPMs(4,cmag) = 0;
                    end
                end
            end
        end
end  
for binx = 1:I.numBots
    if bots(binx).EPMs(4,6) ~= 0
        bots(bots(binx).EPMs(4,6)).EPMs(4,12) = binx;
    end
end