function tau = rep(q, myrobot, ob)

    tau = zeros(6,1);       % Initialize torque 
    Freps = zeros(3,6);     % Initialize a matrix to store repulsive forces for each joint

    % Loop through each joint to calculate forces and torques
    for i = 1:6
        Jv_i = zeros(3,6);  % Initialize Jacobian matrix for current joint

        % Compute current position of joint i
        Hi = myrobot.A(1:i, q);  % Transformation matrix up to joint i
        oi = Hi(1:3,4);          % Current position of joint i

        % Compute Jacobian up to joint i
        for j = 1:i
            Hj_1 = myrobot.A(1:j-1, q); % Transformation matrix up to joint j-1
            Rj_1 = Hj_1(1:3,1:3);      % Rotation part of Hj_1
            zj_1 = Rj_1(:,3);          % z-axis of joint j-1
            oj_1 = Hj_1(1:3,4);        % Position of joint j-1
            Jv_i(:,j) = cross(zj_1, (oi - oj_1)); % Compute jth column of Jacobian
        end

        % Initialize repulsive force vector for joint i
        Frepi = zeros(3,1);

        % Compute repulsive forces depending on obstacle type
        if strcmp(ob.type, 'cyl')  % For cylindrical obstacles
            oixy = oi(1:2);        % XY position of joint i
            dist = norm(oixy - ob.c) - ob.R;  % Distance to obstacle
            % Calculate force based on distance to obstacle
            if dist < 0
                fprintf("Already in collision \n");
            elseif dist > ob.rho0
                Frepi = zeros(3,1);
            else           
                r2d = oixy - ob.c;  
                r = [r2d(1) r2d(2) 0]';
                r = r / norm(r);
                Frepi = 1 * (1/dist - 1/ob.rho0) / dist^2 * r;
            end

        elseif strcmp(ob.type, 'sph')  % For spherical obstacles
            dist = norm(oi - ob.c) - ob.R;  % Distance to obstacle
            % Calculate force based on distance to obstacle
            if dist < 0
                fprintf("Already in collision \n");
            elseif dist > ob.rho0
                Frepi = zeros(3,1);
            else           
                r = oi - ob.c;
                r = r / norm(r);
                Frepi = 1 * (1/dist - 1/ob.rho0) / dist^2 * r;
            end

        else
            fprintf("Strange object \n"); 
        end
       
        % Update the total torque based on the repulsive force
        tau = tau + Jv_i' * Frepi;    
        Freps(:,i) = Frepi;  % Store computed repulsive force
    end

    % Normalize tau if it is not zero
    if norm(tau) ~= 0
        tau = tau/norm(tau);
    end
end
