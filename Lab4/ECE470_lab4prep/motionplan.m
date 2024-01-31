function qref = motionplan(q0, q_f, t1, t2, myrobot, obs, tol)

% Initialize the starting configuration
q = q0;

% Set parameters for attractive and repulsive forces
alpha_att = 0.013;
alpha_rep = 0.01;

% Loop until the current configuration is close enough to the final configuration
while norm(q(end,1:5)'-q_f(1:5)') >= tol

    % Get the last row of q, representing the current configuration
    qi = q(end, 1:6)';

    % Calculate attractive force
    tau_att = att(qi, q_f, myrobot); % vector 6x1

    % Initialize repulsive force vector
    tau_rep = zeros(6,1);

    % Compute repulsive forces for each obstacle
    cobjs = []; % initialize array to store indices of close objects
    for c = 1:size(obs,2)
        rep_c = rep(qi, myrobot, obs{c});
        tau_rep = tau_rep + rep_c;
        if norm(rep_c) ~= 0
            cobjs(end+1) = c; % store index of close object
        end
    end

    % Display information about proximity to objects
    if size(cobjs,2) ~= 0
        fprintf("Close to objects:");
        disp(cobjs);
    else
        fprintf("Close to no objects \n\n");
    end

    % Compute the next configuration
    tau = alpha_att * tau_att + alpha_rep * tau_rep;
    qii = qi + 1 * tau;

    % Append the new configuration to the matrix q
    q(end+1, 1:6) = qii';

    % Display current step and error
    error = norm(q(end,1:5)'-q_f(1:5)');
    fprintf("Step " + (size(q,1)-1)  + ", error of " + error + "\n");
    disp(qii');
    fprintf("\n \n");

end

% Interpolate for the 6th joint variable
q(:,6) = linspace(q0(6), q_f(6), size(q,1));

% Generate a spline reference trajectory
disp(size(q,1))
t = linspace(t1, t2, size(q,1));
qref = spline(t, q');

end
