% Physics Variables
L1 = 5 * 0.0254; % m
Lc1 = 0.3 * L1;
L2 = 3.5 / 5 * L1;
g = 9.81;
freq = 10/1000; % ms
mw = 0.05;
m1 = 1.1;
wm = 2 * 3.1415 * 6000 / 60 / 40;
Tm = 0.0025 * 40;
r = 40 / 1000;

% Equations Of Motion
x0 = sym('x',[4 1]); % m2*L2*(L2-L1*cos(x0(3)))
% M = [r*(mw+m1) -r*(mw+m1)-m1*Lc1*cos(x0(3)); -r*m1*Lc1*cos(x0(3)) r*m1*Lc1*cos(x0(3))+m1*Lc1^2];
M = [mw+m1 -m1*Lc1*cos(x0(3)); -m1*Lc1*cos(x0(3)) m1*Lc1^2];
V = [m1*Lc1*x0(4)*sin(x0(3)); 0];
G = [0; -m1*g*Lc1*sin(x0(3))];
T = [-Tm/r*(x0(2)/r+x0(4))/wm; 0];
% M = [m1*Lc1^2+m2*(L1^2+L2^2-2*L1*L2*cos(x0(3))) 0; 0 m2*L2^2];
% V = [m2*L1*L2*x0(4)*(2*x0(2)+x0(4))*sin(x0(3)); -m2*L1*L2*x0(2)^2*sin(x0(3))];
% G = [-m1*g*Lc1*sin(x0(1))+m2*g*(-L1*sin(x0(1))+L2*sin(x0(1)+x0(3))); m2*g*L2*sin(x0(1)+x0(3))];

f1(x0) = M^(-1)*(-V-G+T);
f2(x0) = M^(-1)*([Tm/r 0]');
f2v = f2(x0(1),x0(2),x0(3),x0(4));

d1 = diff(f1,x0(1));
d1 = d1(x0(1),x0(2),x0(3),x0(4));
d2 = diff(f1,x0(2));
d2 = d2(x0(1),x0(2),x0(3),x0(4));
d3 = diff(f1,x0(3));
d3 = d3(x0(1),x0(2),x0(3),x0(4));
d4 = diff(f1,x0(4));
d4 = d4(x0(1),x0(2),x0(3),x0(4));

% Control System
A(x0) = freq * [0     1     0     0; ...
                d1(1) d2(1) d3(1) d4(1); ...
                0     0     0     1; ... 
                d1(2) d2(2) d3(2) d4(2)] + eye(4);
B(x0) = freq * [0 f2v(1) 0 f2v(2)]';
% A(x0) = [0     1     0     0; ...
%          d1(1) d2(1) d3(1) d4(1); ...
%          0     0     0     1; ... 
%          d1(2) d2(2) d3(2) d4(2)];
% B(x0) = [0 f2v(1) 0 f2v(2)]';

R = 1/(1^2);
Q = [1/(0.1)^2 0     0          0;...
         0          1/(r*wm)^2 0          0;...
         0          0     1/(pi/8)^2 0;...%1/(pi/2)^2 0;...
         0          0     0          1/(pi/8)^2];

A_now = double(A(0,0,0,0));
B_now = double(B(0,0,0,0));

[K,~,~] = dlqr(A_now, B_now, Q, R);
% K = lqr(A_now, B_now, Q, R);
K = K

file = fullfile(pwd,'Arduino','Main','K.h');
fileID = fopen(file,'w');

K3 = zeros(2,6);
K3(1,1:4) = K;
K2 = strrep( regexprep( mat2str(K3), {'\[', '\]', '\s+'}, {'', '', ', '}), ';', '}, {');
fprintf(fileID,'const float k[%d][%d] = {{%s}};', length(K3(:,1)), length(K3(1,:)), K2);
fclose(fileID);