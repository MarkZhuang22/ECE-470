%% Function that creates q to solve inverse kinematics problem
function q = inverse(H, myrobot)

    % Extract(DH) parameters from the robot model
    d = myrobot.d;
    a = myrobot.a; 
    
    % Extract the rotational part of H
    Rd = H(1:3,1:3);
    
    % Calculate the position vector for the wrist center
    c = H(1:3,4) - Rd*[a(6); 0; d(6);];
    xc = c(1); 
    yc = c(2); 
    zc = c(3); 
    
    % Calculate the distance from the base to the wrist center projected in xy-plane
    l = sqrt(xc.^2 + yc.^2 - d(2).^2);
    
    % Calculate the first joint angle (theta 1)
    th1 = atan2(yc, xc) - atan2(-d(2), l);
    
    % Compute cos(theta 3) using the cosine law
    D = (((zc - d(1))^2 + l.^2 - d(4).^2 - a(2).^2) / (2 * a(2) * d(4)));
    th3 = asin(D); % theta 3
    
    % Calculate the second joint angle (theta 2)
    th2 = atan2(zc - d(1), l) - atan2(d(4) * sin(th3 - pi/2), a(2) + d(4) * cos(th3 - pi/2));
    
    % Compute the transformation matrix for the first three joints
    joint = [th1 th2 th3];
    H3 = myrobot.A(1:3, joint);
    R3 = H3.R; % Extract the rotation part of H3
    
    % Determine the orientation of the end effector relative to the third joint
    R = transpose(R3) * Rd;
    
    % Calculate the Euler angles for the last three joints
    th456 = tr2eul(R);
    
    % Concatenate all joint angles to form the inverse kinematics solution
    q = [th1 th2 th3 th456(1) th456(2) th456(3)];

end
