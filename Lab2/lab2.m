DH = [0 400.0 25.0 pi/2;
0 0 315.0 0;
0 0 35.0 pi/2;
0 365.0 0 -pi/2;
0 0 0 pi/2;
0 161.44 -296.23 0];
% Creating robot structure
kuka = mykuka(DH);
q = [pi/5 pi/3 -pi/4 pi/4 pi/3 pi/4];
H = forward_kuka(q,kuka);
fprintf("forward_kuka H: \n")
disp(H)
%Check Inverse
Hr = [0.1173 -0.3109 0.9432 368.9562;
-0.8419 -0.5349 -0.0717 420.4832;
0.5268 -0.7856 -0.3245 120.8570;
0 0 0 1.000;];
qc = inverse_kuka(H,kuka);
fprintf("inverse_kuka q:")
disp(qc)
delta = fminunc(@deltajoint,[0 0]);
myrobot = mykuka_search(delta)


H = [0 0 1 592.55;
    0 -1 0 49.99;
    1 0 0 32.37;
    0 0 0 1];
q = [0.1145    0.8222   -0.3513    0.1279    1.1118   -0.0572];
p_workspace = [600; 100; -3];
p_baseframe = FrameTransformation(p_workspace);


R06 = [0 0 1; 0 -1 0; 1 0 0];
H = [R06 p_baseframe; zeros(1, 3) 1];
q = inverse_kuka(H,myrobot);

function structure = mykuka(DH)
structure = SerialLink(DH);

end

