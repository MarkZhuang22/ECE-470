function tau = rep(q, myrobot, ob)

% Initialize the tau vector (6x1)
tau = zeros(6,1);

% Initialize the matrix for repulsive forces for each joint
Freps = zeros(3,6);

% Loop through all the links of the robot
for i = 1:6

    % Initialize velocity Jacobian for link i
    Jv_i = zeros(3,6);

    % Calculate the transformation matrix of link i
    Hi = myrobot.A(1:i, q);
    oi = Hi.t;

    % Fill the columns of the velocity Jacobian until column i
    for j = 1:i
        Hj_1 = myrobot.A(1:j-1, q);
        Rj_1 = Hj_1.R;
        zj_1 = Rj_1(:,3);
        oj_1 = Hj_1.t;
        Jv_i(:,j) = cross(zj_1, (oi - oj_1));
    end

    % Initialize repulsive force for link i
    Frepi = zeros(3,1);

    % Handling cylinder obstacle
    if strcmp(ob.type, 'cyl')

        % Calculate the vector for the cylinder's influence
        a = ob.c;
        a(3) = ob.h;
        n = [0; 0; 1];
        v_parr = (oi - a)-(oi - a)'*n*n;

        % Calculate perpendicular and parallel distances
        d_perp = max(0, (oi - a)' * n);
        d_parr = max(0, norm(v_parr) - ob.R);
        dist = sqrt(d_perp^2 + d_parr^2);
        v_dist = d_perp * n + d_parr * (v_parr) / norm(v_parr);

        % Calculate repulsive force based on distance
        if dist == 0
            fprintf("Already in collision \n");
        elseif dist > ob.rho0
            Frepi = zeros(3,1);
        else
            Frepi = 1 * (1/dist - 1/ob.rho0) / dist^3 * v_dist;
        end

    % Handling plane obstacle
    elseif strcmp(ob.type, 'plane')
        dist = max(0, (oi - ob.p)' * ob.n);
        if dist == 0
            fprintf("Already in collision \n");
        elseif dist > ob.rho0
            Frepi = zeros(3,1);
        else
            Frepi = 1 * (1/dist - 1/ob.rho0) / dist^2 * ob.n;
        end

    % Handling sphere obstacle
    elseif strcmp(ob.type, 'sph')
        dist = norm(oi - ob.c) - ob.R;
        if dist < ob.R
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

    % Sum the forces applied to all joints
    tau = tau + Jv_i' * Frepi;

    % Store the repulsive force for current link
    Freps(:,i) = Frepi;
end

% Normalize the tau vector if it is not zero
if norm(tau) ~= 0
    tau = tau / norm(tau);
end

end
