clc;
clear;
%%
% 4.1 Definition of robot structure
% Define the DH parameters for PUMA 560
DH = [
    0     76     0     pi/2;
    0    -23.65  43.23  0;
    0     0      0     pi/2;
    0     43.18  0    -pi/2;
    0     0      0     pi/2;
    0     20     0     0
];
% Create the robot structure
myrobot = mypuma560(DH);

%%
% 4.2 Plot a sample joint space trajectory
% Initialize the joint angles matrix
q = zeros(200, 6);
% Fill the matrix with joint angles
q(:, 1) = linspace(0, pi, 200);  % For θ1
q(:, 2) = linspace(0, pi/2, 200);  % For θ2
q(:, 3) = linspace(0, pi, 200);  % For θ3
q(:, 4) = linspace(pi/4, 3*pi/4, 200);  % For θ4
q(:, 5) = linspace(-pi/3, pi/3, 200);  % For θ5
q(:, 6) = linspace(0, 2*pi, 200);  % For θ6

% Plot the robot configurations
plot(myrobot, q);

%%
% 4.3 Forward Kinematics
% Initialize variables for forward kinematics
n = 200;
H1 = zeros(4, 4, n);
o = zeros(200,3);

% Loop to calculate the homogeneous transformation matrix and end effector position
for t = 1:n
    H = forward(q(t,:), myrobot); 
    H1(:, :, t) = H;
    o(t,:) = H(1:3,4);
end
% Plot the trajectory of the end effector
figure;
plot3(o(:, 1), o(:, 2), o(:, 3), 'r');
hold on;
plot(myrobot, q);

%%
% 4.4 Inverse Kinematics
% Test the inverse function
H = [cos(pi/4) -sin(pi/4) 0 20; sin(pi/4) cos(pi/4) 0 23; 0 0 1 15; 0 0 0 1];
q_inv = inverse(H, myrobot); 
disp(q_inv);  % Display the calculated joint angles
disp(H);  % Display the given homogeneous transformation matrix

% Initialize variables for inverse kinematics test
n = 100;
d = zeros(n, 3);
% Define a straight-line path for the end-effector
d(:, 1) = linspace(10, 30, n);
d(:, 2) = linspace(23, 30, n);
d(:, 3) = linspace(15, 100, n);

% Define the constant orientation of the end-effector using a rotation matrix
R = rotz(pi/4);

% Initialize matrices for storing inverse kinematics solutions and positions
qs_inv = zeros(n, 6);
os = zeros(n, 3);

% Loop to calculate joint angles and end-effector positions along the path
for i = 1:n  
   H = eye(4, 4);
   H(1:3, 1:3) = R;  % Set orientation
   H(1:3, 4) = d(i, 1:3)';  % Set position
   
   qs_inv(i, :) = inverse(H, myrobot);  % Call the inverse function to find joint angles
   
   H = forward(qs_inv(i, :), myrobot);  % Call the forward function to verify position
   os(i, :) = H(1:3, 4);  % Store the calculated position
end

% Plot the calculated end-effector positions
plot3(os(:, 1), os(:, 2), os(:, 3), 'r');
hold on;

% Plot the robot configurations
plot(myrobot, qs_inv);

