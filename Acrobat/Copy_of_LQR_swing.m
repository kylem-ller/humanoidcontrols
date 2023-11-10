% Physics Setup
L1 = 5 * 0.0254; % m
Lc1 = 0.5 * L1;
L2 = 3.5 / 5 * L1;
g = 9.81;
freq = 10/1000; % ms
friction = 0; % percent max grav acc
m1 = 0.2; % kg
m2 = 0.2;

% Initial States
theta1 = 0;
theta2 = pi;
w1 = 0;
w2 = 0;
tau = 0;

% Targets
thetaR = 0;
theta2R = pi/6;
wSwing = 15;

% Ranges
thetaSwing = pi/4;
theta2Min = pi/8;
theta2Max = 2*pi-theta2Min;
aMax = 60;

% Equations Of Motion
x0 = sym('x',[4 1]); % m2*L2*(L2-L1*cos(x0(3)))
M = [m1*Lc1^2+m2*(L1^2+L2^2-2*L1*L2*cos(x0(3))) 0; 0 m2*L2^2];
V = [m2*L1*L2*x0(4)*(2*x0(2)+x0(4))*sin(x0(3)); -m2*L1*L2*x0(2)^2*sin(x0(3))];
G = [-m1*g*Lc1*sin(x0(1))+m2*g*(-L1*sin(x0(1))+L2*sin(x0(1)+x0(3))); m2*g*L2*sin(x0(1)+x0(3))];

f1(x0) = M^(-1)*(-V-G);
f2(x0) = M^(-1)*([0 1]');
f2v = f2(x0(1),x0(2),x0(3),x0(4));

d1 = diff(f1,x0(1));
d1 = d1(x0(1),x0(2),x0(3),x0(4));
d2 = diff(f1,x0(1));
d2 = d2(x0(1),x0(2),x0(3),x0(4));
d3 = diff(f1,x0(1));
d3 = d3(x0(1),x0(2),x0(3),x0(4));
d4 = diff(f1,x0(1));
d4 = d4(x0(1),x0(2),x0(3),x0(4));

% Control System
A(x0) = freq * [0     1     0     0; ...
                d1(1) d2(1) d3(1) d4(1); ...
                0     0     0     1; ... 
                d1(2) d2(2) d3(2) d4(2)] + eye(4);
B(x0) = freq * [0 f2v(1) 0 f2v(2)]';
R = [0 0 pi 0]';
% C = [1 0 0 0; 0 0 1 0];
% D = [0 0]';
LQR_R = 1/(0.1^2);
% wSwing2 = -double(int(aG(theta0,pi),pi/2,0));
% LQR_R = 1/(1^2);
x = [theta1 w1 theta2 w2]';
x = [x x];
tau = 0;

% Transfer Functions
% s = tf('s');

% Initial Figure
f = figure();
xlim((2.1*L1)*[-1 1]);
ylim((2.1*L1)*[-1 1]);
theta1R = get_theta1(thetaR, theta2R);
[gx1,y1,gx2,y2] = coords(L1,L2,theta1,theta2);
[xR1,yR1,xR2,yR2] = coords(L1,L2,theta1R,theta2R);
hold on
l1 = plot([0 gx1],[0 y1],'linewidth',2);
l2 = plot([gx1 gx2],[y1 y2],'linewidth',2);
ball = plot(gx2,y2,'.','MarkerSize',40);
%goal1 = plot(xR1,yR1,'.','MarkerSize',20);
%goal2 = plot(xR2,yR2,'.','MarkerSize',20);
%kb = HebiKeyboard();
pause(1)

i = 0;
while (true)
    % Update A
    %A_now = double(A(x(1,2),x(2,2),x(3,2),x(4,2)));
    %B_now = double(B(x(1,2),x(2,2),x(3,2),x(4,2)));
    A_now = double(A(0,0,pi,0));
    B_now = double(B(0,0,pi,0));

    % Update Physics
    % x = shift(x, A_now*(x(:,1)-x(:,2)) + B_now*tau + x(:,2) + freq*[0 f1_now(1) 0 f1_now(2)]');
    a = double(f1(x(1,1),x(2,1),x(3,1),x(4,1))) + double(f2(x(1,1),x(2,1),x(3,1),x(4,1)))*tau;
    x = shift(x, (freq*[0 1 0 0; 0 0 0 0; 0 0 0 1; 0 0 0 0]+eye(4))*x(:,1) + freq*[0 a(1) 0 a(2)]');

    % Update Motor Acc
    Q = [1/(pi/4)^2 0     0          0;...
         0          1/8^2 0          0;...
         0          0     1/(pi/4)^2 0;...%1/(pi/2)^2 0;...
         0          0     0          1/8^2];
    R = [0 0 pi 0]';
%     if (abs(theta(1)) > 3*pi/5)
%         Q = [0 0          0 0;...
%              0 1/wSwing^2 0 0;...
%              0 0          0 0;...
%              0 0          0 0];
%         R = [0 min(30,5*w(2)) pi 0]';
%         %R = [0 1.5*abs(theta(2))*w(2) pi 0]';
%     else
%         Q = [2/(pi/4)^2 0 0      0;...
%              0          0 0          0;...
%              0          0 2/(pi/4)^2 0;...%1/(pi/2)^2 0;...
%              0          0 0 0];
%         R = [0 0 pi 0]';
%     end

    % [K,~,~] = lqr(A_now, B, Q, LQR_R);
    [K,~,~] = dlqr(A_now, B_now, Q, LQR_R);

    tau = K*(R-x(:,1))
    
    i = i + 1;
    if (mod(i, cast(1/100 / freq,"uint8")) == 0)
        % Edit Figure
        theta1R = get_theta1(R(1), R(3));
        [gx1,y1,gx2,y2] = coords(L1,L2,x(1,1),x(3,1));
        [xR1,yR1,xR2,yR2] = coords(L1,L2,theta1R,R(3));
        set(l1,'XData',[0 gx1],'YData',[0 y1]);
        set(l2,'XData',[gx1 gx2],'YData',[y1 y2]);
        set(ball,'XData',gx2,'YData',y2);
        %set(goal2,'XData',xR2,'YData',yR2);
        %set(goal1,'XData',xR1,'YData',yR1);
    end
    pause(freq)
end

function [x1,y1,x2,y2] = coords(L1,L2,theta,theta_2)
    x1 = L1*cos(theta+pi/2);
    y1 = L1*sin(theta+pi/2);
    x2 = x1 + L2*cos(theta+theta_2+3*pi/2);
    y2 = y1 + L2*sin(theta+theta_2+3*pi/2);
end

function ang = dist(theta, theta_2)
    ang = mod(theta-theta_2, 2*pi);
    if (ang > mod(theta_2-theta, 2*pi))
        ang = -mod(theta_2-theta, 2*pi);
    end
end

function y = z_transfer(y, u, tf, freq)
    y_new = y(1);
    y = shift(y, y_new);
    for i=1:length(tf)
        [z_u, z_y] = tfdata(c2d(tf(i), freq, 'tustin'));
        z_u = cell2mat(z_u);
        z_y = cell2mat(z_y);
    
        y_new = y_new - y(1) + sum(z_u.*u(i,1:length(z_u))) - sum(z_y(2:end).*y(1:length(z_y)-1));
    end
    y = shift(y, y_new);
end

function y = shift(y, x)
    y = [x y(:,1:end-1)];
end

function theta1 = get_theta1(theta, theta2)
    theta1 = theta + pi/2 - theta2/2;
end

function theta = get_theta(theta1, theta2)
    theta = theta1 - pi/2 + theta2/2;
end