function q = inverse_kuka(H, myrobot)
    % INVERSE_KUKA Calculate the inverse kinematics for a Kuka robot.
    % 
    % Inputs:
    %   H: Homogeneous transformation matrix representing the desired pose of the robot end-effector.
    %   myrobot: Structure containing the robot parameters, specifically the Denavit-Hartenberg (DH) parameters.
    % 
    % Output:
    %   q: 1x6 vector containing the joint angles required to achieve the given end-effector pose.

    % Extract DH table parameters
    d = myrobot.d;
    a = myrobot.a;

    % Compute the desired end-effector orientation and position
    Rd = H(1:3,1:3);
    od = H(1:3,4);

    % Calculate the center of the wrist based on the end effector position and orientation.
    % Note: The values [-a(6); 0; d(6)] are the displacements from the wrist center to the end-effector.
    c = od - Rd*[a(6); 0; d(6)];
    xc = c(1);
    yc = c(2);
    zc = c(3);

    % Calculate theta1 based on the wrist's x and y coordinates.
    th1 = atan2(yc,xc);

    % Define helper constants for further calculations
    s = zc - d(1);  % Height of wrist from the base
    r = sqrt(xc.^2 + yc.^2) - a(1);  % Radial distance of the wrist from the robot's vertical axis
    l = sqrt(d(4).^2 + a(3).^2);  % Distance from joint 3 to joint 5 (along the arm)

    % Calculate theta3
    g = atan2(a(3),d(4));
    D = ((s.^2 + r.^2 - l.^2 - a(2).^2) / (2*l*a(2)));
    th3 = asin(D) - g;

    % Calculate theta2 using the known s, r, l, and th3 values.
    th2 = atan2(s,r) - atan2(l*sin(th3+g-pi/2), a(2) + l*cos(th3+g-pi/2));

    % Compute the transformation matrix from the base to the wrist (H30) using the first three joint angles.
    joint = [th1 th2 th3];
    H3 = myrobot.A(1:3, joint);
    R3 = H3.R;

    % Calculate the orientation of the end-effector relative to the wrist (joint 3).
    R = transpose(R3)*Rd;
    th456 = tr2eul(R);  % Extract Euler angles from the rotation matrix

    % Compile all the joint angles into a single vector
    q = [th1 th2 th3 th456(1) th456(2) th456(3)];

end
