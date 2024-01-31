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

%% Part 4.1 - Motion Planning for KUKA Robot Arm

% Define the grid height
z_grid = 45;

% Define the positions for the robot's path
p0 = [370 -440 150];
p1 = [370 -440 z_grid];
p2 = [750 -220 225];
p3 = [620 350 225];

% Define the rotation matrix for the end-effector's orientation
R = [0 0 1; 0 -1 0; 1 0 0;];

% Create homogenous transformation matrices for each position
H0 = [R p0' ; zeros(1,3) 1];
H1 = [R p1' ; zeros(1,3) 1];
H2 = [R p2' ; zeros(1,3) 1];
H3 = [R p3' ; zeros(1,3) 1];

% Define the robot's home configuration
q_home = [0 1.5708 0 0 1.5708 0];

% Solve inverse kinematics for each position
q0 = inverse(H0, kuka);
q1 = inverse(H1, kuka);
q2 = inverse(H2, kuka);
q3 = inverse(H3, kuka);

% Generate time vector for motion planning
t = linspace(0, 10, 100);

% Motion planning from home to q0
qref_0 = motionplan(q_home, q0, 0, 10, kuka_forces, obs, 0.03);
q_0 = ppval(qref_0, t)';

% Motion planning from q0 to q1
qref_1 = motionplan(q0, q1, 0, 10, kuka_forces, obs, 0.03);
q_1 = ppval(qref_1, t)';

% Motion planning from q1 to q2
qref_2 = motionplan(q1, q2, 0, 10, kuka_forces, obs, 0.03);
q_2 = ppval(qref_2, t)';

% Motion planning from q2 to q3
qref_3 = motionplan(q2, q3, 0, 10, kuka_forces, obs, 0.03);
q_3 = ppval(qref_3, t)';

% Plot obstacles and the robot's path
plotobstacle(obs);
plot(kuka, q_0);
plot(kuka, q_1);
plot(kuka, q_2);
plot(kuka, q_3);
%% Part 4.2 - Initial Motion Planning with Kuka

% Open the gripper
setGripper(0);

% Move the robot through the first trajectory
for i = 1:size(q_0, 1) 
    setAngles(q_0(i, :), 0.04);
end

% Set angles to q1 position and close the gripper
setAngles(q1, 0.04);
setGripper(1); % close gripper

% Continue through the second trajectory
for i = 1:size(q_2, 1) 
    setAngles(q_2(i, :), 0.04);
end

% Complete the third trajectory
for i = 1:size(q_3, 1) 
    setAngles(q_3(i, :), 0.04);
end

% Open the gripper
setGripper(0);

% Answer to the question: Varying the algorithm parameters—such as the 
% repulsive (α_rep) and attractive (α_att) coefficients, 
% the repulsive strength (η), and the ρ0 (influence radius of obstacles)—
% significantly impacts the robot's motion planning. Increasing α_att leads
% to a more aggressive approach towards the goal, enhancing speed but 
% risking overshoots, while increasing α_rep intensifies obstacle avoidance,
% possibly causing longer, more erratic paths. Conversely, decreasing these 
% coefficients results in more cautious movements but may slow down task 
% completion. The ρ0 parameter, defining the obstacle's influence radius, 
% plays a crucial role: a larger ρ0 increases the avoidance area, 
% enhancing safety but potentially complicating the path, 
% while a smaller ρ0 might streamline the route at the risk of 
% closer encounters with obstacles.

%% Part 4.3 - Creative Motion Planning with Kuka

% The Creative Motion Planning with Kuka involves a series of intricate 
% maneuvers for object handling and navigating through obstacles. 
% The Kuka robot first approaches and picks up an object at position q1, 
% then transitions to q3 for object placement. This is followed by a 
% reverse journey to q2, then a movement to a new position q2b for picking 
% up a second object. The robot finally returns to q3 for object placement, 
% showcasing its ability to execute complex paths and precise object 
% manipulation.

% Define the new target position near p2
p2b = [750 -220 50];
H2b = [R p2b'; zeros(1, 3) 1];
q2b = inverse(H2b, kuka);

% Open the gripper
setGripper(0);

% Motion planning from q0 to q3
qref_03 = motionplan(q0, q3, 0, 10, kuka_forces, obs, 0.03);
q_03 = ppval(qref_03, t)';

% Move to q0 position
for i = 1:size(q_0, 1) 
    setAngles(q_0(i, :), 0.04);
end

% Move to q1, pick up the object, and return to q0
setAngles(q1, 0.02);
setGripper(1); % close gripper
setAngles(q0, 0.02);

% Move to q3 and drop the object
for i = 1:size(q_03, 1) 
    setAngles(q_03(i, :), 0.04);
end

% Open the gripper
setGripper(0);

% Reverse movement from q3 to q2
for i = size(q_3, 1):-1:1
    setAngles(q_3(i, :), 0.04);
end

% Move to the new position q2b and close the gripper
setAngles(q2b, 0.04);
setGripper(1); % close gripper

% Return to q2 position
setAngles(q2, 0.04);

% Move back to q3 and open the gripper
for i = 1:size(q_3, 1) 
    setAngles(q_3(i, :), 0.04);
end

% Open the gripper
setGripper(0);

% Set the view for the plot and display the robot's final position
view(-32, 50);
hold on;
plotobstacle(obs);
plot(kuka, q);
hold off;