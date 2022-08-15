
% Waypoints = [15 15;20 10; 25 15;20 20; 15.5 15.5];
Waypoints = [15 15;20 15];
startLoc = [0 0];
goalLoc = [0 0];
startLoc(1,1) = Waypoints(1,1);
startLoc(1,2) = Waypoints(1,2);

goalLoc(1,1) = Waypoints(end,1);
goalLoc(1,2) = Waypoints(end,2);
load exampleMaps.mat


%open_system('pathPlanningBicycleSimulinkModel.slx')
map = binaryOccupancyMap(emptyMap)
simulation = sim('pathPlanningBicycleSimulinkModelTest.slx');
robotPose = simulation.BicyclePose;


numRobots = size(robotPose, 2) / 3;
thetaIdx = 3;

% Translation
xyz = robotPose;
xyz(:, thetaIdx) = 0;

% Rotation in XYZ euler angles
theta = robotPose(:,thetaIdx);
thetaEuler = zeros(size(robotPose, 1), 3 * size(theta, 2));
thetaEuler(:, end) = theta;

for k = 1:size(xyz, 1)
    show(map)
    hold on;
    
    % Plot Start Location
    plotTransforms([startLoc, 0], eul2quat([0, 0, 0]))
    text(startLoc(1), startLoc(2), 2, 'Start');
    
    % Plot Goal Location
    plotTransforms([goalLoc, 0], eul2quat([0, 0, 0]))
    text(goalLoc(1), goalLoc(2), 2, 'Goal');
    
    % Plot Robot's XY locations
    plot(robotPose(:, 1), robotPose(:, 2), '-b')
    
    % Plot Robot's pose as it traverses the path
    quat = eul2quat(thetaEuler(k, :), 'xyz');
    plotTransforms(xyz(k,:), quat, 'MeshFilePath',...
        'groundvehicle.stl');
    
    pause(0.01)
    hold off;
end