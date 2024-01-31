function H = forward_kuka(joint, myrobot)
H = myrobot.A(1:6, joint); %to get link transformation matrices H
end