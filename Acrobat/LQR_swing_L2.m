% Physics Setup
L1 = 5 * 0.0254;% 100/1000; % m
L2 = 4/5 * L1; % m
g = 9.81;
freq = 10/1000; % ms
friction = 0; % percent max grav acc

% Initial States
theta = pi;
theta2 = pi;
w = 0;
w2 = 0;

% Targets
thetaR = 0;
theta2R = pi/2;
wSwing = 15;

% Ranges
thetaSwing = pi/4;
theta2Min = pi/8;
theta2Max = 2*pi-theta2Min;
aMax = 40;

% Equations Of Motion
syms L3 theta1 theta3 theta4 theta0 theta20 w20 a20
L3(theta0, theta20, w20, a20) = sqrt(L1^2 + L2^2 - 2*L1*L2*cos(theta20));
theta3(theta0, theta20, w20, a20) = -asin(L2/L3*sin(theta20));
theta1(theta0, theta20, w20, a20) = theta0 + theta3;
theta4(theta0, theta20, w20, a20) = theta0 + theta1 - pi;
aG(theta0, theta20, w20, a20) = g*sin(theta0)/L3;
a2G(theta0, theta20, w20, a20) = g*sin(theta4)/L2;

% Control System
C1 = diff(aG,theta0);
C2 = diff(aG,theta20);
C3 = diff(a2G,theta0);
C4 = diff(a2G,theta20);
C5(theta0,theta20,w20,a20) = 0.5+0*a20;%diff(theta3,theta20,3)*1/2*freq^2*w20^2 + diff(theta3,theta20,2)*(1/2*freq^2*a20+2*w20*freq) + diff(theta3,theta20,1);
%C5(theta0,theta20,w20,a20) = -diff(theta3,theta20,1);

%A(theta0, theta20, w20, a20) = [0 1 0 0]'*[C1 0 C2 0] + [0 C5 0 1]'*[C3 0 C4 0];
A(theta0, theta20, w20, a20) = [0        1 0        0; ...
                                C1+C3*C5 0 C2+C4*C5 0; ...
                                0        0 0        1; ... 
                                C3       0 C4       0];
B(theta0, theta20, w20, a20) = [0 C5 0 1]';
R = [thetaR 0 theta2R 0]';
C = [1 0 0 0; 0 0 1 0];
D = [0 0]';
LQR_R = 1/100^2/2;
wSwing2 = -double(int(aG(theta0,pi,0,0),pi/2,0));

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

% Initial Figure
f = figure();
xlim((1.05*(L1+L2))*[-1 1]);
ylim((1.05*(L1+L2))*[-1 1]);
[x1,y1,x2,y2] = coords(L1,L2,double(theta1(theta(1), theta2(1), w2(1), a2(1))),theta2(1));
[xR1,yR1,xR2,yR2] = coords(L1,L2,double(theta1(thetaR, theta2R, w2(1), a2(1))),theta2R);
hold on
l1 = plot([0 x1],[0 y1],'linewidth',2);
l2 = plot([x1 x2],[y1 y2],'linewidth',2);
ball = plot(x2,y2,'.','MarkerSize',40);
%goal1 = plot(xR1,yR1,'.','MarkerSize',20);
%goal2 = plot(xR2,yR2,'.','MarkerSize',20);
%kb = HebiKeyboard();
pause(1)

i = 0;
while (true)
    % Update Controls
    A_now = double(A(theta(1),theta2(1),w2(1), a2(1)));
    B_now = double(B(theta(1),theta2(1),w2(1), a2(1)));
    C5_now = double(C5(theta(1),theta2(1),w2(1), a2(1)));

    % Update Physics
    a2 = shift(a2, aM(1) + double( a2G(theta(1), theta2(1), w2(1), a2(1) )));
    a = shift(a, a2(1)*C5_now + double(aG(theta(1), theta2(1), w2(1), a2(1))));
%     if (w ~= 0)
%         a(1) = a(1) - abs(w(1))/w(1) * friction/100 *g/(2*l);
%     end
%     if (w2 ~= 0)
%         a2(1) = a2(1) - abs(w2(1))/w2(1) * friction/100 *g/l;
%     end

    % Button Press Disturbance
%     if all(state.keys('a'))
%         a(1) = a(1) + 100;
%     elseif all(state.keys('d'))
%         a(1) = a(1) -100;
%     end
    
    theta = z_transfer(theta, [w; a], [1/s; 1/s^2], freq);
    theta2 = z_transfer(theta2, [w2; a2], [1/s; 1/s^2], freq);
    %theta2(1) = max(theta2Min,min(theta2Max,theta2(1)));
    theta = theta - 2*pi*floor((theta(1)+pi)/(2*pi));
    theta2 = theta2 - 2*pi*floor(theta2(1)/(2*pi));

    w = z_transfer(w, a, 1/s, freq);
    w2 = z_transfer(w2, a2, 1/s, freq);

    % Update Motor Acc
    if (abs(theta(1)) > thetaSwing)
        Q = [0 0          0 0;...
             0 10/wSwing^2 0 0;...
             0 0          0 0;...
             0 0          0 1/wSwing^2];
        R = [0 abs(theta(1))*(w(1)) pi 0]';
    else
        Q = [1/(pi/2)^2 0 0          0;...
             0          0 0          0;...
             0          0 1/(pi)^2 0;...%1/(pi/2)^2 0;...
             0          0 0          0];
        R = [dist(0,theta(1))+theta(1) 0 pi 0]';
    end

    [K,~,~] = lqr(A_now, B_now, Q, LQR_R);
    %[K,~,~] = dlqr(A_now, B_now, Q, LQR_R);

    state = [theta(1) w(1) theta2(1) w2(1)]';
    aM = shift(aM, K*(R-state));
    aM = max(-aMax, min(aMax, aM));
%     disp(aM(1))

    i = i + 1;
    if (mod(i, cast(1/100 / freq,"uint8")) == 0)
        % Edit Figure
        [x1,y1,x2,y2] = coords(L1,L2,double(theta1(theta(1), theta2(1), w2(1), a2(1))),theta2(1));
        [xR1,yR1,xR2,yR2] = coords(L1,L2,double(theta1(thetaR, theta2R, w2(1), a2(1))),R(3));
        set(l1,'XData',[0 x1],'YData',[0 y1]);
        set(l2,'XData',[x1 x2],'YData',[y1 y2]);
        set(ball,'XData',x2,'YData',y2);
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