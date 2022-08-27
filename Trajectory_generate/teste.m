%% Linear com velocidade constante
dx=[];dy=[];dtheta=[];dphi=[];phi=[];v=[];

% Tempo amostrado de 1ms
interval=0.001;
% Para um percurso de 10 segundos
t = 0:interval:10;


%Dados do carro
wheelbase=0.13;

%trajetória retilinea
% % Dado uma velocidade K1
% K1 = 1;
% x = K1*t; x(end)=[];
% % Dado uma inclinação da reta dada por um a=k2;
% K2 = 1;
% y=K2*x;
% theta0=atan(K1*K2/K1);
% 
% %Posição inicial do carro
% initState=[0,0,theta0,0];
% 
% theta=[theta0];

%trajetória parabólica
% Dado uma velocidade K1
K1 = 0.5;
x = K1*t; 
% Dado uma inclinação da reta dada por um a=k2;
K2 = 0.25;
y=K2*x.^2;
dy=K2*K1*t;
for i=1:length(x)-1
    dx(end+1)=K1;
end
theta=atan(0.25*t);
v=0.5.*((cos(theta)).^-1);

phi=[];
dphi=[];
for i=1:length(x)-1
   dtheta(end+1) = 0.25/(1+(0.25*t(i))^2);
   x=(dtheta(i)*wheelbase/v(i));
   phi(end+1)=atan(x);
   
   dphi(end+1)=-(0.03125*wheelbase*v(i)*t(i))/(0.0625*wheelbase^2+v(i)^2*(0.0625*t(i)^2+1)^2);
   
end


%Posição inicial do carro
initState=[0,0,theta(1),0];






%%Plot 3d simulation


iteractions=length(phi);
startLoc=[x(1),y(1)];
goalLoc=[x(end),y(end)];
load exampleMaps.mat
%open_system('pathPlanningBicycleSimulinkModel.slx')
map = binaryOccupancyMap(emptyMap);
simulation = sim('pathPlanningTest.slx',t(end));
robotPose = simulation.CarPose;
numRobots = size(robotPose, 2) / 3;
thetaIdx = 3;

% Translation
xyz = robotPose;
xyz(:, thetaIdx) = 0;

% Rotation in XYZ euler angles
theta1 = robotPose(:,thetaIdx);
thetaEuler = zeros(size(robotPose, 1), 3 * size(theta1, 2));
thetaEuler(:, end) = theta1;

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
