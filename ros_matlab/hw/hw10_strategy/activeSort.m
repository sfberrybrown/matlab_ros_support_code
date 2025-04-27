function activeSort(zone, objName, objType, sort, optns)
% Generalized sorting method for all objects based on inputs:
    % zone: zone of interest
    % objName: object of interest
    % objType: can, bottle, spam, marker, or block
    % sort: color bin to be sorted
        % blue: bottles, markers, blue cubes, red cubes
        % green: cans, spam, pink cubes, green cubes

% Define zone starting pose
if zone == "zone1"
    StartZone = [0.6482   -0.2317    1.4450   -1.2133    0.0003   -0.9226];
    moveToQ("Custom",optns,StartZone);
    pause(3);
else
    StartZone = [-1.8380   -0.0978    1.8084   -1.7105    0    4.4452]; 
    moveToQ("Custom",optns,StartZone);
    pause(3);
end

% Define gripValue (amount grippers need to close)
if objType == "can"
    gripValue = 0.24;
elseif objType == "upBottle"
    gripValue = 0.526;
elseif objType == "downBottle"
    gripValue = 0.21;
elseif objType == "block"
    gripValue = 0.66;
elseif objType == "marker"
    gripValue = 0.66;
else % spam
    gripValue = 0.33;
end

% Define z_offset (height to grip object)
if objType == "can"
    z_offset = 0.18;
elseif objType == "upBottle"
    z_offset = 0.24;
elseif objType == "downBottle"
    z_offset = 0.14;
elseif objType == "block"
    z_offset = 0;
elseif objType == "marker"
    z_offset = 0.17;
else % spam
    z_offset = 0.2;

end

% Hover over object
disp("Lining up with the object...")
hover = lift(objName, 0.3);
moveTo(hover, optns);
pause(10);

% "Pick" Object
disp("Grasping Object...")
objName(3,4) = objName(3,4) + z_offset;
moveTo(objName, optns);
pause(5);
doGrip("pick", optns, gripValue);
pause(5);

% Lift Object
disp("Lifting Object...")
hover = lift(objName, 0.3);
moveTo(hover, optns);
pause(5);

% Sort and Place object into correct bin
disp("Sorting Object")
if sort == "blue"
    moveToQ("Custom", optns, [-0.8*pi 0 pi/2.3 -pi/2.3 0 0]); % Blue
    pause(7);
else
    moveToQ("Custom", optns, [5/8*pi 0 pi/2.3 -pi/2.3 0 0]); % Green
    pause(7);
end

doGrip("place", optns);
    pause(10);

% Return to start pose
moveToQ("Custom",optns,StartZone); 

end