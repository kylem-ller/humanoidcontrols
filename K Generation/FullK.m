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

% Equations Of Motion
x0 = sym('x',[6 1]);

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

T = [-Tm/r*(x0(2)/r+x0(4))/wm*Gw^2;...
     0;...
     -Tm*x0(6)/wm*G2^2];

f1(x0) = M^(-1)*(-V-G+T);
f2(x0) = M^(-1)*([Tm*Gw/r 0 0]');
f3(x0) = M^(-1)*([0 0 Tm*G2]');
f2v = f2(x0(1),x0(2),x0(3),x0(4),x0(5),x0(6));
f3v = f3(x0(1),x0(2),x0(3),x0(4),x0(5),x0(6));

d1 = diff(f1,x0(1));
d1 = d1(x0(1),x0(2),x0(3),x0(4),x0(5),x0(6));
d2 = diff(f1,x0(2));
d2 = d2(x0(1),x0(2),x0(3),x0(4),x0(5),x0(6));
d3 = diff(f1,x0(3));
d3 = d3(x0(1),x0(2),x0(3),x0(4),x0(5),x0(6));
d4 = diff(f1,x0(4));
d4 = d4(x0(1),x0(2),x0(3),x0(4),x0(5),x0(6));
d5 = diff(f1,x0(5));
d5 = d5(x0(1),x0(2),x0(3),x0(4),x0(5),x0(6));
d6 = diff(f1,x0(6));
d6 = d6(x0(1),x0(2),x0(3),x0(4),x0(5),x0(6));

% Control System
A(x0) = freq * [0     1     0     0     0     0; ...
                d1(1) d2(1) d3(1) d4(1) d5(1) d6(1); ...
                0     0     0     1     0     0; ... 
                d1(2) d2(2) d3(2) d4(2) d5(2) d6(2); ...
                0     0     0     0     0     1; ...
                d1(3) d2(3) d3(3) d4(3) d5(3) d6(3)] + eye(6);
B(x0) = freq * [0      0; ...
                f2v(1) f3v(1); ...
                0      0; ... 
                f2v(2) f3v(2); ...
                0      0; ...
                f2v(3) f3v(3)];
% A(x0) = [0     1     0     0; ...
%          d1(1) d2(1) d3(1) d4(1); ...
%          0     0     0     1; ... 
%          d1(2) d2(2) d3(2) d4(2)];
% B(x0) = [0 f2v(1) 0 f2v(2)]';

R = diag([1/(1^2) 1/(1^2)]);
Q = diag([1/(0.1)^2, 1/wm^2, 1/(pi/4)^2, 1/(pi/4)^2,  1/(pi/4)^2, 1/(pi/4)^2]);

A_now = double(A(0,0,0,0,pi,0));
B_now = double(B(0,0,0,0,pi,0));

[K,~,~] = dlqr(A_now, B_now, Q, R);
% K = lqr(A_now, B_now, Q, R);

file = fullfile(pwd,'Arduino','Main','K.h');
fileID = fopen(file,'w');

K2 = strrep( regexprep( mat2str(K), {'\[', '\]', '\s+'}, {'', '', ', '}), ';', '}, {');
fprintf(fileID,'const float k[%d][%d] = {{%s}};', length(K(:,1)), length(K(1,:)), K2);
fclose(fileID);