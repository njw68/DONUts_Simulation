%SensorObsDetection
function [Tri,I] = SensorObsDetection(bots,I,bot)
    %%Plots the IR sensor FOV polygons around a module
    x = bots(bot).coordinates(1); y = bots(bot).coordinates(2);

    botRad = 22.9183; %%mm
    const = 7;%%mm
    Radial = [0:10:100];
    Radial = Radial + const + botRad;
    Angular = [0:5:180];
    count = 0;
    angle = [0:0.01:180];
    radius = [0:10:100];
    radius = radius + const + botRad;
    countPolygon = 0;
    
    %find angle between world reference
    index1 = find(bots(bot).EPMs(1,:)==1);
    epmVec = [bots(bot).EPMs(2,index1)-bots(bot).coordinates(1) bots(bot).EPMs(3,index1)-bots(bot).coordinates(2) 0];
    xVec = [bots(bot).coordinates(1)+I.botDiam 0 0];
    thd = atan2(norm(cross(epmVec,xVec)),dot(epmVec,xVec));
    radAng = 2*pi/I.numEPMs*(index1-1);
    for ii = 1:4
        irsensor(ii,1) = bots(bot).coordinates(1) + botRad*cos(radAng - pi/4 +(2*pi/4*ii));
        irsensor(ii,2) = bots(bot).coordinates(2) + botRad*sin(radAng - pi/4 +(2*pi/4*ii));
    end
    
    for radialCount = 1:length(Radial)
        for angularCount = 1:length(Angular)
            if(I.IRSensorData(angularCount,radialCount) > 0)
                count = count + 1;
                ex(count) = Radial(radialCount)*cos(deg2rad(Angular(angularCount)));
                ey(count)  = Radial(radialCount)*sin(deg2rad(Angular(angularCount)));
            end
        end
    end
    
    %%Draw outline of each sensor's FOV
    irx = zeros(4,44); iry = zeros(4,44);

    orientation = pi/4 + 2*pi/4*(1-1)+radAng;
    T = [cos(orientation) -sin(orientation) 0; sin(orientation) cos(orientation) 0; 0 0 1];
    P = [ex; ey; ones(1,length(ex(:)))];
    newP = T*P;
    newEx = newP(1,:);
    newEy = newP(2,:);
    boundaryPoints = boundary(transpose(ex),transpose(ey),0.9);
    irx(1,:) = (1/(22.9183))*newEx(boundaryPoints);
    iry(1,:) = (1/(22.9183))*newEy(boundaryPoints);
    Tri = polyshape(x+irx(1,:),y+iry(1,:));   
    for i = 2:4
        orientation = pi/4 + 2*pi/4*(i-1)+radAng;
        T = [cos(orientation) -sin(orientation) 0; sin(orientation) cos(orientation) 0; 0 0 1];
        P = [ex; ey; ones(1,length(ex(:)))];
        newP = T*P;
        newEx = newP(1,:);
        newEy = newP(2,:);
        boundaryPoints = boundary(transpose(ex),transpose(ey),0.9);
        irx(i,:) = (1/(22.9183))*newEx(boundaryPoints);
        iry(i,:) = (1/(22.9183))*newEy(boundaryPoints);
        Tri(end+1) = polyshape(x+irx(i,:),y+iry(i,:));
        
    end
    
end