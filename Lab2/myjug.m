function myjug(myrobot)
    % MYJUG Command the robot to follow a trajectory defined by data in an Excel file.
    % 
    % Inputs:
    %   myrobot: Structure containing the robot parameters.
    
    % Read data from 'jug.xlsx' Excel file. 
    % The data is expected to have at least two columns corresponding to x and y coordinates.
    data = xlsread('jug.xlsx');
    
    % Get the number of data points
    N = length(data);   
    
    % Initialize the workspace points in a 3xN matrix
    X_workspace = zeros(3,N);
    
    % Convert the data from the Excel file into the workspace points.
    % Here, the x-coordinates are scaled by a factor of 14 and offset by 550.
    % The y-coordinates are only scaled by a factor of 14.
    % The z-coordinates are set to a constant value of -2.
    X_workspace(1,:) = 550 + 14*data(:, 1);
    X_workspace(2,:) = 14*data(:, 2);
    X_workspace(3,:) = - ones(length(data),1)*2;
     
    % Initialize the points in the robot's base frame in a 3xN matrix
    X_baseframe = zeros(3,N);
    
    % Transform the workspace points to the robot's base frame
    for i = 1:N
        X_baseframe(:,i) = FrameTransformation(X_workspace(:,i));
    end
    
    % Define the desired end-effector orientation relative to the base frame
    R06 = [0 0 1; 0 -1 0; 1 0 0];
    
    % For each point in the trajectory, compute the required joint angles and command the robot to move
    for i = 1:N
         H = [R06 X_baseframe(:,i); zeros(1, 3) 1];  % Construct the homogeneous transformation matrix
         q = inverse_kuka(H,myrobot);               % Compute the joint angles using inverse kinematics
         setAngles(q, 0.03);                         % Command the robot to move to the computed angles with a specified speed
    end
end

    