%%DisconnectionCheck
function [bots,world] = DisconnectionCheck(bots,world,I)

for presBot = 1:I.numBots
    if world.canMove(presBot) ~= 0 %checks if the present bot can move
        neigh1 = [];
        %get the present bot's neighbors
        for int1 = 1:I.numEPMs
            if bots(presBot).EPMs(4,int1) ~= 0
                neigh1(end+1) = bots(presBot).EPMs(4,int1);
            end
        end
        Acon = 1:I.numBots; Ncon = 0; Lcon = zeros(1,I.numBots); %array of connects, # of connects, list of bots w/ connects
        for int2=1:length(neigh1)
            for int1=1:I.numEPMs
                if bots(neigh1(int2)).EPMs(4,int1) ~= 0 
                    if ismember(bots(neigh1(int2)).EPMs(4,int1),neigh1)
                        [La,Loc] = ismember(bots(neigh1(int2)).EPMs(4,int1),neigh1);
                        Acon(end+1,neigh1(int2)) = neigh1(Loc);
                        Lcon(neigh1(Loc)) = 1;
                    end
                end
            end
        end
        for int3=1:I.numBots
            if Lcon(int3) == 1
                Ncon = Ncon+1;
            end
        end
        
        if length(neigh1) ~= Ncon && length(neigh1) > 1 %if not all of the neighbors are connected
            int4 = 1;
            while world.canMove(presBot) == 1 && int4 <= length(neigh1)
            %iterate through list of neighbors
                neigh = neigh1(int4); %neigh is one of the present bot's neigbors 
                if Lcon(neigh) == 0 %if the neighbor isn't connected to any other neighbors
                    pl = neigh; vl = [presBot,neigh]; fn = 0; %pl = path list, vl = visited list, fb = found neighbor
                    while ~isempty(pl)
                        neigh = pl(end); pl(end) = [];
                        for int6=1:I.numEPMs
                            inv = 0;
                            if bots(neigh).EPMs(4,int6)
                                for int7=1:length(vl)
                                    if bots(neigh).EPMs(4,int6) == vl(int7)
                                        inv = 1;
                                    end
                                end
                                if inv == 0
                                    pl(end+1) = bots(neigh).EPMs(4,int6);
                                    vl(end+1) = bots(neigh).EPMs(4,int6);
                                    if ismember(bots(neigh).EPMs(4,int6),neigh1)
                                        pl = []; fn = 1;
                                    end
                                end
                            end
                        end
                    end
                    if fn == 0 %if no path was found remove the transitions for the present bot
                        for int8=1:I.numEPMs
                            bots(presBot).EPMs(5,int8) = 0;
                            world.canMove(presBot) = 0;
                        end
                    end
                    fn = 0;
                end
            int4=int4+1;
            end


        end
    end        
end