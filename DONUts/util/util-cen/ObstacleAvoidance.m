function [bots,world,I] = ObstacleAvoidance(bots,world,I)
both = [];
for bot=1:length(world.canMove)
    if world.canMove(bot) == 1 %check if a bot can move at all
        %if it can move, get the center coordinates of the bot    
            for magpos = 1:I.numEPMs
                if bots(bot).EPMs(5,magpos) ~= 0

                    %If mover bot can move CW
                    bot_rotate = bots(bot).EPMs(4,magpos);
                    if bots(bot).EPMs(5,magpos) == 1
                        moves = 1;
                        [Tri,I] = SensorObsDetection(bots,I,bot);
                        [bots,world,moves,I] = CWCheck(bots,world,I,moves,bot,bot_rotate,both,Tri,sensorPos,magpos);
                        bots(bot).EPMs(5,magpos) = moves;
                    %If mover bot can move CCW   
                    elseif bots(bot).EPMs(5,magpos) == -1
                        moves = -1;

                        sensorPos = 1:4;
                        dir = -1;
                        [Tri,I] = SensorObsDetection(bots,I,bot);
                        [bots,world,moves,I] = CCWCheck(bots,world,I,moves,bot,bot_rotate,both,Tri,sensorPos,magpos);
                        bots(bot).EPMs(5,magpos) = moves;
                        
                     %If mover bot can move CW OR CCW   
                    elseif bots(bot).EPMs(5,magpos) == 2
                        moves = [1 -1]; both = 1;
                        %CW
                        sensorPos = 1:4;
                        dir = 1;
                        [Tri,I] = SensorObsDetection(bots,I,bot);
                        [bots,world,moves,I] = CWCheck(bots,world,I,moves,bot,bot_rotate,both,Tri,sensorPos,magpos);
                        %CCW
                        sensorPos = 1:4;
                        dir = -1;
                        [Tri,I] = SensorObsDetection(bots,I,bot);
                        [bots,world,moves,I] = CCWCheck(bots,world,I,moves,bot,bot_rotate,both,Tri,sensorPos,magpos);
                        both = [];
                        
                    if moves(1) ~= 0 && moves(2) ~= 0
                        bots(bot).EPMs(5,magpos) = 2;
                        moves = [1 -1];
                    elseif moves(1)~=0 && moves(2)==0
                        bots(bot).EPMs(5,magpos) = moves(1);
                        moves = [1 -1];
                    elseif moves(1) == 0 && moves(2) ~=0
                        bots(bot).EPMs(5,magpos) = moves(2);
                        moves = [1 -1];
                    else
                        bots(bot).EPMs(5,magpos) = 0;
                        moves = [1 -1];
                    end
                    end                    
                end             
            end
    end
end


     