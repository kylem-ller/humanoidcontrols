% Physics Setup
freq = 10 / 1000; % s to ms
LoadAcrobatDynamics(@LoadRobotKinematics, true, freq);

% Initial States
theta1 = 0.1;
theta2 = 0;
w1 = 0;
w2 = 0;

% Targets
thetaR = 0;
theta2R = pi/6;
wSwing = 15;

% Ranges
thetaSwing = pi/4;

R = 1/(1^2);

x = [theta1 theta2 w1 w2]';
u = 0;

% Initial Figure
fig = figure();
xlim((2.1*L1)*[-1 1]);
ylim((2.1*L1)*[-1 1]);
[gx1,y1,gx2,y2] = coords(L1,Lc2,theta1,theta2);
hold on
l1 = plot([0 gx1],[0 y1],'linewidth',2);
l2 = plot([gx1 gx2],[y1 y2],'linewidth',2);
ball = plot(gx2,y2,'.','MarkerSize',20);
 
pause(1)
i = 0;
while (true)
    % Update A
    A_now = double(A(x(1),x(2),x(3),x(4),u));
    B_now = double(B(x(1),x(2),x(3),x(4),u));
    H_now = double(H(x(1),x(2),x(3),x(4),u));

    % Update Physics
    x = (freq*[zeros(2,2) eye(2); zeros(2,4)]+eye(4))*x + freq*double(f(x(1),x(2),x(3),x(4),u));
    x(1:2) = x(1:2) - 2*pi*floor((x(1:2)+pi)/(2*pi));

    q = H_now*x;
    q(1:2) = q(1:2) - 2*pi*floor((x(1:2)+pi)/(2*pi));
    
    % Update Motor Acc
    Q = [1/(pi/32)^2 0     0          0;...
         0          1/(pi/32)^2 0          0;...
         0          0     1/(pi/4)^2 0;...%1/(pi/2)^2 0;...
         0          0     0          1/(pi/8)^2];
    q_ref = [0 0 0 0]';
%     if (abs(theta(1)) > 3*pi/5)
%         Q = [0 0          0 0;...
%              0 1/wSwing^2 0 0;...
%              0 0          0 0;...
%              0 0          0 0];
%         q_ref = [0 min(30,5*w(2)) pi 0]';
%         %q_ref = [0 1.5*abs(theta(2))*w(2) pi 0]';
%     else
%         Q = [2/(pi/4)^2 0 0      0;...
%              0          0 0          0;...
%              0          0 2/(pi/4)^2 0;...%1/(pi/2)^2 0;...
%              0          0 0 0];
%         q_ref = [0 0 pi 0]';
%     end

    % [K,~,~] = dlqr(A_now, B_now, Q, R);
    [K,~,~] = dlqr(A_now, B_now, Q, R);

    % u = u + K*(q_ref-x)
    u = K*(q_ref-q);
    u = max(min(1,u), -1);
    
    i = i + 1;
    if (mod(i, cast(1/100 / freq,"uint8")) == 0)
        % Edit Figure
        [gx1,y1,gx2,y2] = coords(L1,Lc2,x(1),x(2));
        set(l1,'XData',[0 gx1],'YData',[0 y1]);
        set(l2,'XData',[gx1 gx2],'YData',[y1 y2]);
        set(ball,'XData',gx2,'YData',y2);
    end
    pause(freq)
end

function [x1,y1,x2,y2] = coords(L1,L2,theta,theta_2)
    x1 = L1*cos(theta+pi/2);
    y1 = L1*sin(theta+pi/2);
    x2 = x1 + L2*cos(theta+theta_2+pi/2);
    y2 = y1 + L2*sin(theta+theta_2+pi/2);
end

function ang = dist(theta, theta_2)
    ang = mod(theta-theta_2, 2*pi);
    if (ang > mod(theta_2-theta, 2*pi))
        ang = -mod(theta_2-theta, 2*pi);
    end
end