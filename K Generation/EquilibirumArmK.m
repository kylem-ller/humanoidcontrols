freq = 20 / 1000; % s to ms
LoadAcrobatDynamics(@LoadRobotKinematics, false, freq);

R = 1/(1^2);
Q = diag([1/(0.1)^2, 1/(pi/8)^2, 1/(r*wm)^2, 1/(pi/8)^2]);

A_now = double(A(0,0,0,0,0));
B_now = double(B(0,0,0,0,0));

[K,~,~] = dlqr(A_now, B_now, Q, R);
K3 = zeros(2,6);
K3(1,1:2) = K(1:2);
K3(1,4:5) = K(3:4);

uploadToArduino(K3, 0, 0);