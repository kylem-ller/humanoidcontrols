% Physics Variables
L1 = 5 * 0.0254; % m
Lc1 = 0.5 * L1;
Lc2 = 3.5 * 0.0254;
g = 9.81;
freq = 30/1000; % ms
mw = 0.05;
m1 = 0.9;
m2 = 0.2;
wm = 2 * 3.1415 * 6000 / 60;
Tm = 0.0025;
Gw = 40;
G2 = 45;
r = 80 / 1000;
Ic1 = 0;
Ic2 = 0;

% State & Input Variables
x0 = sym('x',[6 1]);
v0 = sym('v',[2 1]);
s0 = [x0; v0];

% Equations Of Motion
sigma1 = -m1*Lc1*cos(x0(3))+m2*(-L1*cos(x0(3))+Lc2*cos(x0(3)+x0(5)));
sigma2 = m2*Lc2*cos(x0(3)+x0(5));
sigma3 = Ic1+Ic2+m1*Lc1^2+m2*(L1^2+Lc2^2-2*L1*Lc2*cos(x0(5)));
sigma4 = Ic2+m2*Lc2*(Lc2-L1*cos(x0(5)));
M = [mw+m1+m2 sigma1 sigma2;...
     sigma1 sigma3 sigma4;...
     sigma2 sigma4 Ic2+m2*Lc2^2];

V = [m1*Lc1*x0(4)^2*sin(x0(3))+m2*(L1*x0(4)^2*sin(x0(3))-Lc2*(x0(4)+x0(6))^2*sin(x0(3)+x0(5)));...
     m2*L1*Lc2*x0(6)*(2*x0(4)+x0(6))*sin(x0(5));...
     -m2*L1*Lc2*x0(4)^2*sin(x0(5))];

G = [0;...
     -m1*g*Lc1*sin(x0(3))+m2*g*(-L1*sin(x0(3)+Lc2*sin(x0(3)+x0(5))));...
     m2*g*Lc2*sin(x0(3)+x0(5))];

T = [Gw*Tm/r*(v0(1)-(x0(2)/r+x0(4))/wm*Gw);...
     0;...
     G2*Tm*(v0(2)-x0(6)/wm*G2)];

% Control System
f = [x0(end/2+1:end); M^(-1)*(T-V-G)];
A(s0) = freq * jacobian(f, x0) + eye(6);
B(s0) = freq * jacobian(f, v0);
% A(s0) = jacobian(f, x0);
% B(s0) = jacobian(f, v0);

R = diag([1/(1^2) 1/(1^2)]);
Q = diag([1/(0.1)^2, 1/(pi/4)^2, 1/(pi/4)^2, 1/wm^2, 1/(pi/4)^2, 1/(pi/4)^2]);

A_now = double(A(0,0,0,0,pi,0,0,0));
B_now = double(B(0,0,0,0,pi,0,0,0));

[K,~,~] = dlqr(A_now, B_now, Q, R);
% K = lqr(A_now, B_now, Q, R);
K = K

file = fullfile(pwd,'Arduino','Main','K.h');
fileID = fopen(file,'w');

K2 = strrep( regexprep( mat2str(K), {'\[', '\]', '\s+'}, {'', '', ', '}), ';', '}, {');
fprintf(fileID,'const float k[%d][%d] = {{%s}};', length(K(:,1)), length(K(1,:)), K2);
fclose(fileID);