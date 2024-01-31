function H = forward(joint, myrobot)

    % Extracting DH parameters 
    n = length(myrobot.a);  
    DH = zeros(n,4);        % Initialize DH parameter matrix

    % Populate the DH matrix from the robot model
    DH(:,2) = myrobot.d';  
    DH(:,3) = myrobot.a';  
    DH(:,4) = myrobot.alpha'; 

    % Define rotation matrix around z-axis
    function R = myrotz(theta)
        % myrotz: Creates a rotation matrix for a rotation about the z-axis.
        R = [
            cos(theta) -sin(theta) 0;
            sin(theta)  cos(theta) 0;
            0          0           1;
        ];
    end

    % Define rotation matrix around x-axis
    function R = myrotx(theta)
        R = myrotz(theta);   % Initial rotation about z-axis
        Y = [                 % Matrix for aligning with x-axis
            0 0 1;
            0 1 0;
            -1 0 0;
        ];
        R = Y * R * Y';       % Final rotation about x-axis
    end

    % Convert a rotation matrix to a homogeneous transformation matrix
    function H = rotToTrot(R)
        H = eye(4,4);        % Initialize as 4x4 identity matrix
        H(1:3,1:3) = R;      % Embed the rotation matrix
    end

    % Homogeneous transformation for rotation about z-axis
    function H = mytrotz(theta) 
        % mytrotz: Creates a 4x4 homogeneous transformation matrix for rotation about the z-axis.
        H = rotToTrot(myrotz(theta));
    end

    % Homogeneous transformation for rotation about x-axis
    function H = mytrotx(theta) 
        % mytrotx: Creates a 4x4 homogeneous transformation matrix for rotation about the x-axis.
        H = rotToTrot(myrotx(theta));
    end

    % Create a homogeneous translation matrix
    function H = mytransl(x,y,z)
        H = eye(4,4);        % Initialize as 4x4 identity matrix
        H(1:3,4) = [x y z]'; % Set the translation vector
    end

    % Compute the end-effector pose using DH parameters
    H = eye(4,4);           % Initialize as 4x4 identity matrix
    for i = 1:length(joint)
        % Multiply transformation matrices for each joint
        H = H * mytrotz(joint(i)) * mytransl(0,0,DH(i,2)) * mytransl(DH(i,3),0,0) * mytrotx(DH(i,4));       
    end
end
