%ObstacleAvoidance
function [bots,world,I] = ObstacleAvoidance(bots,world,I,~)
    both = [];
    angles = [0 30 60 90 120 150 180 210 240 270 300 330];
    moverMagPos = [7 8 9 10 11 12 1 2 3 4 5 6];
    for bot = 1:I.numBots %currentBot
        if world.canMove(bot) == 1 %check if a bot can move at all
            %if it can move, get the center coordinates of the bot    
                for magpos = 1:I.numEPMs
                    if bots(bot).EPMs(5,magpos) ~= 0 %check each magnet that can move
                        pos = magpos;
                        %get position of the magnet (ex. magnet 5 could be in position 8)
                        %check the magnet's current position and identify where the
                        %bot's center of rotation is
                        bot_rotate = bots(bot).EPMs(4,magpos);
                        C_rotateX = bots(bot_rotate).coordinates(1);
                        C_rotateY = bots(bot_rotate).coordinates(2);

                        %radius of rotation (from the center of rotation to the moving bot's diameter)
                        %If mover bot can move CW
                        if bots(bot).EPMs(5,magpos) == 1
                            moves = 1;
                            if pos ==1
                                ang(1) = angles(moverMagPos(pos)); ang(2) = angles(moverMagPos(end-1)); %CW angles
                                [bots,world,moves] = CWCheck(bots,world,I,bot_rotate,C_rotateX,C_rotateY,bot,ang,moves,both,magpos);
                            elseif pos ==2
                                ang(1) = angles(moverMagPos(pos)); ang(2) = angles(moverMagPos(end)); %CW angles
                                [bots,world,moves] = CWCheck(bots,world,I,bot_rotate,C_rotateX,C_rotateY,bot,ang,moves,both,magpos);
                            else
                                ang(1) = angles(moverMagPos(pos)); ang(2) = angles(moverMagPos(pos-2)); %CW angles
                                [bots,world,moves] = CWCheck(bots,world,I,bot_rotate,C_rotateX,C_rotateY,bot,ang,moves,both,magpos);
                            end
                            bots(bot).EPMs(5,magpos) = moves;

                        %If mover bot can move CCW   
                        elseif bots(bot).EPMs(5,magpos) == -1
                            moves = -1;
                            if pos ==11
                                ang(1) = angles(moverMagPos(pos)); ang(2) = angles(moverMagPos(1)); %CCW angles
                                [bots,world,moves] = CCWCheck(bots,world,I,bot_rotate,C_rotateX,C_rotateY,bot,ang,moves,both,magpos);
                            elseif pos ==12
                                ang(1) = angles(moverMagPos(pos)); ang(2) = angles(moverMagPos(2)); %CCW angles
                                [bots,world,moves] = CCWCheck(bots,world,I,bot_rotate,C_rotateX,C_rotateY,bot,ang,moves,both,magpos);
                            else
                                ang(1) = angles(moverMagPos(pos)); ang(2) = angles(moverMagPos(pos+2)); %CCW angles
                                [bots,world,moves] = CCWCheck(bots,world,I,bot_rotate,C_rotateX,C_rotateY,bot,ang,moves,both,magpos);
                            end
                            bots(bot).EPMs(5,magpos) = moves;

                         %If mover bot can move CW OR CCW   
                        elseif bots(bot).EPMs(5,magpos) == 2 %CW or CCW
                            moves = [1 -1]; both = 1;
                            if pos == 1
                                ang(1) = angles(moverMagPos(pos)); ang(2) = angles(moverMagPos(end-1)); %CW angles
                                [bots,world,moves] = CWCheck(bots,world,I,bot_rotate,C_rotateX,C_rotateY,bot,ang,moves,both,magpos);
                                ang(1) = angles(moverMagPos(pos)); ang(2) = angles(moverMagPos(pos+2)); %CCW angles
                                [bots,world,moves] = CCWCheck(bots,world,I,bot_rotate,C_rotateX,C_rotateY,bot,ang,moves,both,magpos);
                            elseif pos == 2
                                ang(1) = angles(moverMagPos(pos)); ang(2) = angles(moverMagPos(end)); %CW angles
                                [bots,world,moves] = CWCheck(bots,world,I,bot_rotate,C_rotateX,C_rotateY,bot,ang,moves,both,magpos);
                                ang(1) = angles(moverMagPos(pos)); ang(2) = angles(moverMagPos(pos+2)); %CCW angles
                                [bots,world,moves] = CCWCheck(bots,world,I,bot_rotate,C_rotateX,C_rotateY,bot,ang,moves,both,magpos);
                            elseif pos == 11
                                ang(1) = angles(moverMagPos(pos)); ang(2) = angles(moverMagPos(pos-2)); %CW angles
                                [bots,world,moves] = CWCheck(bots,world,I,bot_rotate,C_rotateX,C_rotateY,bot,ang,moves,both,magpos);
                                ang(1) = angles(moverMagPos(pos)); ang(2) = angles(moverMagPos(1)); %CCW angles
                                [bots,world,moves] = CCWCheck(bots,world,I,bot_rotate,C_rotateX,C_rotateY,bot,ang,moves,both,magpos);
                            elseif pos == 12
                                ang(1) = angles(moverMagPos(pos)); ang(2) = angles(moverMagPos(pos-2)); %CW angles
                                [bots,world,moves] = CWCheck(bots,world,I,bot_rotate,C_rotateX,C_rotateY,bot,ang,moves,both,magpos);
                                ang(1) = angles(moverMagPos(pos)); ang(2) = angles(moverMagPos(2)); %CCW angles
                                [bots,world,moves] = CCWCheck(bots,world,I,bot_rotate,C_rotateX,C_rotateY,bot,ang,moves,both,magpos);
                            else
                                ang(1) = angles(moverMagPos(pos)); ang(2) = angles(moverMagPos(pos-2)); %CW angles
                                [bots,world,moves] = CWCheck(bots,world,I,bot_rotate,C_rotateX,C_rotateY,bot,ang,moves,both,magpos);
                                ang(1) = angles(moverMagPos(pos)); ang(2) = angles(moverMagPos(pos+2)); %CCW angles
                                [bots,world,moves] = CCWCheck(bots,world,I,bot_rotate,C_rotateX,C_rotateY,bot,ang,moves,both,magpos);
                            end
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
end


     