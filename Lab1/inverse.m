function q = inverse(H, myrobot)
    
    % Get DH table
    n = length(myrobot.a);
    DH = zeros(n, 4);
    
    % Populate the DH table from the robot structure
    DH(:, 2) = myrobot.d';
    DH(:, 3) = myrobot.a';
    DH(:, 4) = myrobot.alpha';
    
    % Extract the desired end-effector position (o_d0) and orientation (R_d) from H
    o_d0 = H(1:3, 4);
    R_d = H(1:3, 1:3);
    
    % Extract DH parameters needed for inverse kinematics
    d1 = DH(1, 2);
    d2 = DH(2, 2);
    a2 = DH(2, 3);
    d4 = DH(4, 2);
    d6 = DH(6, 2);
    
    % Compute the wrist center position in the base frame (o_c0)
    o_c0 = o_d0 - R_d * [0, 0, d6]';
    ocx = o_c0(1);
    ocy = o_c0(2);
    ocz = o_c0(3);
    
    % Calculate joint angles theta1, theta2, and theta3 based on wrist center
    theta1 = atan2(ocy, ocx) - asin(-d2 / sqrt(ocx^2 + ocy^2));
    theta3 = -pi / 2 + acos((a2^2 + d4^2 - (ocx^2 + ocy^2 - d2^2 + (ocz - d1)^2)) / (2 * a2 * d4));
    theta2 = atan2(ocz - d1, sqrt(ocx^2 + ocy^2 - d2^2)) - atan2(sin(theta3 - pi / 2) * d4, a2 + cos(theta3 - pi / 2) * d4);
    
    % Calculate the transformation matrix H_03 for the first three joints
    H_03 = myrobot.A(1:3, [theta1, theta2, theta3]);
    
    % Extract the rotation matrix R_03 from H_03
    R_03 = H_03.R;
    
    % Compute the rotation matrix R_36 for the last three joints
    R_36 = R_03' * R_d;
    
    % Extract Euler angles theta4, theta5, theta6 from R_36
    theta456 = tr2eul(R_36);
    
    % Combine all joint angles to return as output
    q = [theta1, theta2, theta3, theta456(1), theta456(2), theta456(3)];
end
