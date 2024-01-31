clc
%% Configuring the KUKA robot

% Define DH parameters for each joint
d1 = 400; a1 = 25; a2 = 315; a3 = 35; d4 = 365; a6 = 156; d6 = 161.44;

% Create the DH table for the robot
DH = [
      0     d1     a1     pi/2 ;
      0     0      a2     0    ;
      0     0      a3     pi/2 ;
      0     d4     0     -pi/2;
      0     0      0      pi/2 ;
      0     d6    -a6     0    ];

% Create the KUKA robot model using the DH parameters
kuka = mykuka(DH);

% Adjust DH parameters for force calculations (set a6 to zero)
DH_forces = DH;
DH_forces(6,3) = 0;
kuka_forces = mykuka(DH_forces);

%% Setup Obstacles for the Experiment

setupobstacle_lab4prep;
setupobstacle;

%% Define Positions for the End-Effector

% Define two positions in space for the robot's end-effector
p1 = [620  375 50];
p2 = [620 -375 50];

% Define the rotation matrix for the end-effector's orientation
R = [
    0  0 1;
    0 -1 0;
    1  0 0;
    ];

% Define homogenous transformation matrices for the two positions
H1 = [R p1' ; zeros(1,3) 1];
H2 = [R p2' ; zeros(1,3) 1];

% Calculate the inverse kinematics to find joint angles for these positions
q1 = inverse(H1, kuka);
q2 = inverse(H2, kuka);
 
%% Repulsive tau test
q = [pi/10, pi/12, pi/6, pi/2, pi/2, -pi/6];
tau = rep(q, kuka_forces, preobs{1})';
fprintf("Tau repulsive test for the cylinder: \n");
disp(tau);

%% Repulsive tau test on plane
tau = rep(q1, kuka_forces, preobs{2})';
fprintf("Tau repulsive test for the plane: \n");
disp(tau)

%% Motion Planning

% Perform motion planning between two configurations q1 and q2
qref = motionplan(q1, q2, 0, 10, kuka_forces, preobs, 0.03);

% Set the axis limits for the plot
axis([-100 100 -100 100 0 200]);

% Set the view angle for the plot
view(-32, 50);

% Plot the obstacles in the environment
plotobstacle(obs);

% Hold on to the current plot
hold on;

% Plot the robot's path
plot(kuka, q);

% Release the plot hold
hold off;
