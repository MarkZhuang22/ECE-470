function H = forward(joint, myrobot)
   
    % Get DH table
    n = length(myrobot.a);
    DH = zeros(n, 4);
    
    % Populate the DH table from the robot structure
    DH(:, 2) = myrobot.d';
    DH(:, 3) = myrobot.a';
    DH(:, 4) = myrobot.alpha';
    
    % Define a nested function for Z-axis rotation matrix
    function R = myrotz(theta)
        R = [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1];
    end
    
    % Define a nested function for X-axis rotation matrix
    function R = myrotx(theta)
        R = myrotz(theta);
        Y = [0, 0, 1; 0, 1, 0; -1, 0, 0];
        R = Y * R * Y';
    end
    
    % Convert a rotation matrix to a homogeneous transformation matrix
    function H = rottotrot(R)
        H = eye(4, 4);
        H(1:3, 1:3) = R;
    end
    
    % Create a homogeneous transformation matrix for Z-axis rotation
    function H = mytrotz(theta)
        H = rottotrot(myrotz(theta));
    end
    
    % Create a homogeneous transformation matrix for X-axis rotation
    function H = mytrotx(theta)
        H = rottotrot(myrotx(theta));
    end
    
    % Create a homogeneous transformation matrix for translation
    function H = mytransl(x, y, z)
        H = eye(4, 4);
        H(1:3, 4) = [x, y, z]';
    end
    
    % Initialize the homogeneous transformation matrix as identity
    H = eye(4, 4);
    
    % Loop through each joint to compute the final transformation matrix
    for i = 1:length(joint)
        H = H * mytrotz(joint(i)) * mytransl(0, 0, DH(i, 2)) * mytransl(DH(i, 3), 0, 0) * mytrotx(DH(i, 4));
    end
end
