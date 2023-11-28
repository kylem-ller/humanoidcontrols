freq = 20 / 1000; % s to ms
LoadFullDynamics(@LoadRobotKinematics, false, freq);

R = diag([1/(0.5^2) 1/(1^2)]);
Q = diag([1/(0.2)^2, 1/(pi/16)^2, 1/(pi/4)^2, 1/(wm*r/Gw)^2, 1/(pi/32)^2, 1/(pi/4)^2]);

A_now = double(A(0,0,0,0,0,0,0,0));
B_now = double(B(0,0,0,0,0,0,0,0));

[K,~,~] = dlqr(A_now, B_now, Q, R);

uploadToArduino(K, 0, 0);