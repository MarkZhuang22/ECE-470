function qref = motionplan(q0, q_f, t1, t2, myrobot, obs, tol)

    q = q0; % Initialize current position as the initial position

    % Loop until the robot position is close enough to the final position
    while norm(mod(q(end,1:5)'-q_f(1:5)'+pi,2*pi)-pi) >= tol
        qi = q(end, 1:6)'; % Current joint configuration

        % Compute torques due to attractive forces
        tau = att(qi, q_f, myrobot);

        % Initialize array for close objects
        cobjs = [];
        % Loop through each obstacle
        for c = 1:size(obs,2)
            % Compute torques due to repulsive forces for each obstacle
            rep_c = rep(qi, myrobot, obs{c});
            tau = tau + rep_c; % Add repulsive forces to total torque
            if norm(rep_c) ~= 0
                cobjs(end+1) = c; % Add obstacle to close objects if force is non-zero
            end
        end

        % Display information about proximity to objects
        if size(cobjs,2) ~= 0
            fprintf("Close to objects:");
            disp(cobjs);
        else
            fprintf("Close to no objects \n\n");
        end

        % Update the joint configuration
        alpha = 0.01; % Step size
        qii = qi + alpha * tau; % New configuration
        q(end+1, 1:6) = qii'; % Append new configuration to trajectory

        % Calculate and display error
        error = norm(q(end,1:5)'-q_f(1:5)');
        fprintf("Step " + (size(q,1)-1)  + ", error of " + error + "\n");
        disp(qii');
        fprintf("\n \n");
    end

    % Interpolate the last joint variable with linspace
    q(:,6) = linspace(q0(6), q_f(6), size(q,1));

    % spline
    t = linspace(t1, t2, size(q,1));
    qref = spline(t, q');
end

    