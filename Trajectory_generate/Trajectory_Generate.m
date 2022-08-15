%%% Geração de trajetórias %%%
%% Linear com velocidade constante
dx=[0];dy=[0];dtheta=[0];dphi=[0];phi=[0];v=[0];

% Tempo amostrado de 1ms
interval=0.1;
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
K1 = 0.25;
x = K1*t; x(end)=[];
% Dado uma inclinação da reta dada por um a=k2;
K2 = 0.2;
y=K2*x.^2;
theta0=0;

%Posição inicial do carro
initState=[0,0,theta0,0];

theta=[theta0];

%Rotina para achar as velocidades de cada eixo
for i=1:length(x)
  
        
    if(i>1)
        dx(end+1)=(x(i)-x(i-1))/interval;
        dy(end+1)=(y(i)-y(i-1))/interval;
        theta(end+1) = atan(dy(i)/dx(i));
        v(end+1)=dx(i).*(cos(theta(i))^-1);
        dtheta(end+1)=(theta(i)-theta(i-1))/interval;
        phi(end+1)=asin((dtheta(i)*wheelbase)/v(i));
        dphi(end+1)=(phi(i)-phi(i-1))/interval;
        %Dado que pela equação da restrção theta = dy*cos(theta)-dx*sin(theta)
        %temos theta = arctg(dy/dx)

    end
end
t(end)=[];




%%Plot 3d simulation


iteractions=length(x);
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
