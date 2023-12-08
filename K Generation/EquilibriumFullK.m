freq = 20 / 1000; % s to ms
LoadFullDynamics(@LoadRobotKinematics, true, freq);

% R = diag([1/(0.25^2) 1/(4^2)]);
% Q = diag([1/(0.3)^2, 1/(pi/8)^2, 1/(pi/4)^2, 1/(1.5*wm*r/Gw)^2, 1/(pi/120)^2, 1/(pi/3)^2]);
R = diag([1/(0.5^2) 1/(8^2)]);
Q = diag([1/(0.3)^2, 1/(pi/18)^2, 1/(2*pi/2)^2, 1/(15*wm*r/Gw)^2, 1/(pi/120)^2, 1/(2*pi)^2]);

A_now = double(A(0,0,0,0,0,0,0,0));
B_now = double(B(0,0,0,0,0,0,0,0));
H_now = double(H(0,0,0,0,0,0,0,0));

[K,~,~] = dlqr(A_now, B_now, Q, R);

uploadToArduino(K, H_now);