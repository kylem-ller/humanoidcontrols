% Physics Variables
L1 = 5 * 0.0254; % m
Lc1 = 0.1 * L1;
L2 = 3.5 / 5 * L1;
g = 9.81;
freq = 50/1000; % ms
mw = 0.05;
m1 = 1.1; %1.1;
wm = 2 * 3.1415 * 6000 / 60 / 40;
Tm = 0.0025 * 40;
r = 40 / 1000;

% State & Input Variables
x0 = sym('x',[4 1]);
v0 = sym('v',[1 1]);
s0 = [x0; v0];

% Equations Of Motion
% M = [r*(mw+m1) -r*(mw+m1)-m1*Lc1*cos(x0(3)); -r*m1*Lc1*cos(x0(3)) r*m1*Lc1*cos(x0(3))+m1*Lc1^2];
M = [mw+m1 -m1*Lc1*cos(x0(3)); -m1*Lc1*cos(x0(3)) m1*Lc1^2];
V = [m1*Lc1*x0(4)^2*sin(x0(3)); 0];
G = [0; -m1*g*Lc1*sin(x0(3))];
T = [Tm/r*(v0(1)-(x0(2)/r+x0(4))/wm); 0];
% M = [m1*Lc1^2+m2*(L1^2+L2^2-2*L1*L2*cos(x0(3))) 0; 0 m2*L2^2];
% V = [m2*L1*L2*x0(4)*(2*x0(2)+x0(4))*sin(x0(3)); -m2*L1*L2*x0(2)^2*sin(x0(3))];
% G = [-m1*g*Lc1*sin(x0(1))+m2*g*(-L1*sin(x0(1))+L2*sin(x0(1)+x0(3))); m2*g*L2*sin(x0(1)+x0(3))];

% Control System
f = [x0(end/2+1:end); M^(-1)*(T-V-G)];
A(s0) = freq * jacobian(f, x0) + eye(4);
B(s0) = freq * jacobian(f, v0);
% A(s0) = jacobian(f, x0);
% B(s0) = jacobian(f, v0);

R = 1/(1^2);
Q = diag([1/(0.1)^2, 1/(pi/8)^2, 1/(r*wm)^2, 1/(pi/8)^2]);

A_now = double(A(0,0,0,0,0));
B_now = double(B(0,0,0,0,0));

[K,~,~] = dlqr(A_now, B_now, Q, R);
% K = lqr(A_now, B_now, Q, R);
K = K

file = fullfile(pwd,'Arduino','Main','K.h');
fileID = fopen(file,'w');

K3 = zeros(2,6);
K3(1,1:2) = K(1:2);
K3(1,4:5) = K(3:4);
K2 = strrep( regexprep( mat2str(K3), {'\[', '\]', '\s+'}, {'', '', ', '}), ';', '}, {');
fprintf(fileID,'const float k[%d][%d] = {{%s}};', length(K3(:,1)), length(K3(1,:)), K2);
fclose(fileID);