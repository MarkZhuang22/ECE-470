function tau = att(q, q_f, myrobot)

    tau = zeros(6,1);       % Initialize torque 
    Fatts = zeros(3,6);     % Initialize a matrix to store attractive forces for each joint

    % Loop through each joint to calculate forces and torques
    for i = 1:6
        Jv_i = zeros(3,6);  % Initialize Jacobian matrix for current joint

        % Compute current position of joint i
        Hi = myrobot.A(1:i, q);   % Transformation matrix up to joint i
        oi = Hi(1:3,4);           % Current position of joint i

        % Compute Jacobian up to joint i
        for j = 1:i
            Hj_1 = myrobot.A(1:j-1,q); % Transformation matrix up to joint j-1
            Rj_1 = Hj_1(1:3,1:3);      % Rotation part of Hj_1
            zj_1 = Rj_1(:,3);          % z-axis of joint j-1
            oj_1 = Hj_1(1:3,4);        % Position of joint j-1
            Jv_i(:,j) = cross(zj_1, (oi - oj_1)); % Compute jth column of Jacobian
        end

        % Compute desired position of joint i
        Ai_f = myrobot.A(1:i, q_f); % Transformation matrix for desired position
        oi_f = Ai_f(1:3,4);         % Desired position of joint i

        % Compute attractive force for joint i
        Fatti = -(oi - oi_f);       % Force vector pointing towards the goal
        tau = tau + Jv_i' * Fatti;  % Update torque based on Jacobian and force

        % Store computed attractive force
        Fatts(:,i) = Fatti;
    end
    
    % Normalize tau if it is not zero
    if norm(tau) ~= 0
        tau = tau/norm(tau);
    end
end

