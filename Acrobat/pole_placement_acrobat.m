% Physics Setup
l = 100/1000; % m
g = 9.81;
freq = 5/1000; % ms
friction = 5; % percent max grav acc

% Initial States
theta = 0;
theta2 = pi/2;
w = 0;
w2 = 0;

% Targets
thetaR = 0;
theta2R = pi/2;

% Ranges
theta2Min = pi/16;
theta2Max = 11*pi/6;
aMax = 40;

% Equations Of Motion
syms theta0 theta20
aG(theta0, theta20) = g*sin(theta0)/(2*l*sin(theta20/2));
a2G(theta0, theta20) = g*sin(theta0+theta20/2-pi/2)/l;

% Control System
C1 = diff(aG,theta0);
C2 = diff(aG,theta20);
C3 = diff(a2G,theta0);
C4 = diff(a2G,theta20);

A(theta0, theta20) = [0       1 0       0; ...
                      C1+C3/2 0 C2+C4/2 0; ...
                      0       0 0       1; ... 
                      C3      0 C4      0];
B = [0 1/2 0 1]';
R = [thetaR 0 theta2R 0]';
C = [1 0 0 0; 0 0 1 0];
D = [0 0]';
poles = [-20+8i -20-8i -10+5i -10-5i ];

% Transfer Functions
s = tf('s');

% Initial States
theta = [theta 0 0 0];
theta2 = [theta2 0 0 0];
w = [w 0 0 0];
w2 = [w2 0 0 0];
a = [0 0 0 0];
a2 = [0 0 0 0];
aM = [0 0 0 0];
int = [0 0 0 0; 0 0 0 0];
int1 = [0 0 0 0];
int2 = [0 0 0 0];

% Initial Figure
f = figure();
xlim((2.1*l)*[-1 1]);
ylim((2.1*l)*[-1 1]);
theta1 = get_theta1(theta(1), theta2(1));
theta1R = get_theta1(thetaR, theta2R);
[x1,y1,x2,y2] = coords(l,theta1,theta2(1));
[xR1,yR1,xR2,yR2] = coords(l,theta1R,theta2R);
hold on
l1 = plot([0 x1],[0 y1],'linewidth',2);
l2 = plot([x1 x2],[y1 y2],'linewidth',2);
ball = plot(x2,y2,'.','MarkerSize',40);
goal1 = plot(xR1,yR1,'.','MarkerSize',20);
goal2 = plot(xR2,yR2,'.','MarkerSize',20);
pause(1)

i = 0;
while (true)
    % Upate Goal
    R(3) = -pi/4*sin(i*freq*5)+pi/2;

    % Update K
    K = place(double(A(theta(1),theta2(1))), B, poles);

    % Update Physics
    a2 = shift(a2, aM(1) + double(a2G(theta(1), theta2(1))));
    a = shift(a, a2(1)/2 + double(aG(theta(1), theta2(1))));
    if (w ~= 0)
        a(1) = a(1) - abs(w(1))/w(1) * friction/100 *g/(2*l);
    end
    if (w2 ~= 0)
        a2(1) = a2(1) - abs(w2(1))/w2(1) * friction/100 *g/l;
    end
    
    theta = z_transfer(theta, [w; a], [1/s; 1/s^2], freq);
    theta2 = z_transfer(theta2, [w2; a2], [1/s; 1/s^2], freq);
    theta2(1) = max(theta2Min,min(theta2Max,theta2(1)));

    w = z_transfer(w, a, 1/s, freq);
    w2 = z_transfer(w2, a2, 1/s, freq);

    % Update Motor Acc
    state = [theta(1) w(1) theta2(1) w2(1)]';
    aM = shift(aM, K*(R-state));
    disp(aM(1))

    i = i + 1;
    if (mod(i, cast(1/100 / freq,"uint8")) == 0)
        % Edit Figure
        theta1 = get_theta1(theta(1), theta2(1));
        theta1R = get_theta1(R(1), R(3));
        [x1,y1,x2,y2] = coords(l,theta1,theta2(1));
        [xR1,yR1,xR2,yR2] = coords(l,theta1R,R(3));
        set(l1,'XData',[0 x1],'YData',[0 y1]);
        set(l2,'XData',[x1 x2],'YData',[y1 y2]);
        set(ball,'XData',x2,'YData',y2);
        set(goal2,'XData',xR2,'YData',yR2);
        set(goal1,'XData',xR1,'YData',yR1);
    end
    pause(freq)
end

function [x1,y1,x2,y2] = coords(l,theta,theta_2)
    x1 = l*cos(theta+pi/2);
    y1 = l*sin(theta+pi/2);
    x2 = x1 + l*cos(theta+theta_2+3*pi/2);
    y2 = y1 + l*sin(theta+theta_2+3*pi/2);
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