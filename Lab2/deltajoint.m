function q = deltajoint(delta)
    % TODO 1/2: Add proper documentation for this function.

    kuka = mykuka_search(delta);

    %-------------------------- Calibration ----------------------------%
    % TODO 2/2: Fill in values Xi and Qi for i = {1, 2, 3}. Xi are 3 by 1
    % column vectors, while Qi are 1 by 6 row vectors.
    X1 = [598.91 60.58 25.22];
    X2 = [266.48 -532.49 32.88];
    X3 = [391.92 443.36 30.67];
    Q1 = [0.1145    0.8222   -0.3513    0.1279    1.1118   -0.0572];
    Q2 = [-1.0428    0.8029   -0.5611       0    1.5708   -0.1637];
    Q3 = [0.9112    0.8027   -0.5672        0    1.5708   -0.1637];
    
    %-------------------------------------------------------------------%

    H1=forward_kuka(Q1, kuka);
    H2=forward_kuka(Q2, kuka);
    H3=forward_kuka(Q3, kuka);
    
    q=norm(H1.t - X1')+norm(H2.t - X2')+norm(H3.t - X3');
end