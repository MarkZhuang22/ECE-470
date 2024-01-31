function mycircle(myrobot)
    % MYCIRCLE Generate a circular trajectory in the robot's workspace and command the robot to follow it.
    % 
    % Inputs:
    %   myrobot: Structure containing the robot parameters.
    
    % Define the number of points in the trajectory
    N = 100;
    
    % Initialize the workspace points in a 3xN matrix
    X_workspace = zeros(3,N);
    
    % Define circle parameters: radius, center (cx, cy), and frequency multiplier (f)
    r = 50;
    cx = 620;
    cy = 0;
    f = 1;
    
    % Compute the circle points in the workspace frame
    for i = 1:N
       X_workspace(:,i) = [r * cos(2*pi*f/(N)*i) + cx ; r * sin(2*pi*f/(N)*i) + cy ; -4];
    end

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
         q = inverse_kuka(H, myrobot);               % Compute the joint angles using inverse kinematics
         setAngles(q, 0.03);                         % Command the robot to move to the computed angles with a specified speed
    end
end
