clc
clear
close all

%% Configuring robot

DH = [
      0     76     0     pi/2; 
      0     -23.65 43.23 0;
      0     0      0     pi/2;
      0     43.18  0     -pi/2;
      0     0      0     pi/2;
      0     20     0     0
];

myrobot = mypuma560(DH);


%% Part 3.1
% Define start and end points
H1 = eul2tr([0 pi pi/2]);
H1(1:3, 4) = 100 * [-1 ; 3 ; 3] / 4;
q1 = inverse(H1, myrobot);
 
H2 = eul2tr([0 pi -pi/2]);
H2(1:3, 4) = 100 * [3; -1; 2] / 4;
q2 = inverse(H2, myrobot); 
% Compute attractive forces between q1 and q2
tau = att(q1, q2, myrobot)'
%% Part 3.2: Motion Planning without Obstacles
% Generate motion plan from q1 to q2 in 10 seconds
qref = motionplan(q1, q2, 0, 10, myrobot, [], 0.01);
t = linspace(0, 10, 300);
q = ppval(qref, t)';
figure;
plot(myrobot,q);
%% Part 3.3 Obstacle Setup and Repulsive Forces
% (Comment out part 3.2 to run this section)
setupobstacle();
% Compute joint configuration midway between q1 and q2
q3 = 0.9*q1 + 0.1*q2;
% Compute repulsive forces from the first obstacle
tau = rep(q3, myrobot, obs{1})'

% Compute repulsive forces from the sixth obstacle
q = [pi/2 pi 1.2*pi 0 0 0];
tau = rep(q, myrobot, obs{6})';

%% Motion planning with both attractive and repulsive forces
% Plot obstacles
setupobstacle();
hold on;
axis([-100 100 -100 100 0 200]);
view(-32, 50);
plotobstacle(obs);

% Generate motion plan from q1 to q2 in 10 seconds considering obstacles
qref = motionplan(q1, q2, 0, 10, myrobot, obs, 0.01);
t = linspace(0,10,300);
q = ppval(qref,t)';
plot(myrobot, q);
hold off;
