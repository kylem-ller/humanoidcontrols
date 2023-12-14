% Physics Setup
freq = 20 / 1000; % s to ms
LoadFullDynamics(@LoadRobotKinematics, true, freq);

% Initial States & Targets
% pos theta1 theta2 vel w1 w2
x = [0; 0.15; 0; 0; 0; 0];
q_ref = [0; 0; 0; 0; 0; 0];
u = [0; 0];

% Cost Matrices
% R = diag([1/(1^2) 1/(1^2)]);
% Q = diag([1/(0.2)^2, 1/(pi/16)^2, 1/(pi/4)^2, 1/(wm*r/Gw)^2, 1/(pi/32)^2, 1/(pi/4)^2]);
% R = diag([1/(0.5^2) 1/(3^2)]);
% Q = diag([1/(0.25)^2, 1/(pi/6)^2, 1/(pi/2)^2, 1/(1.5*wm*r/Gw)^2, 1/(pi/100)^2, 1/(2*pi)^2]);
R = diag([1/(0.4^2) 1/(8^2)]);
Q = diag([1/(0.2)^2, 3*1/(pi/32)^2, 1/(pi/8)^2, 1/(0.4*wm*r/Gw)^2, 1/(pi/80)^2, 1/(pi/8)^2]);

% Initial Figure
fig = figure();
xlim((2.1*L1)*[-1 1]);
ylim((2.1*L1)*[-1 1]);
axis square
[gx1,y1,gx2,y2] = coords(x(1),L1,Lc2,x(2),x(3));
hold on
l1 = plot([x(1) gx1],[0 y1],'linewidth',2);
l2 = plot([gx1 gx2],[y1 y2],'linewidth',2);
ball = plot(gx2,y2,'.','MarkerSize',20);
circle = rectangle('Position',[x(1)-r -r 2*r 2*r],'Curvature',[1,1]);

pause(1)
i = 0;
while (true)
    % Update Physics
    %acc = double(f(x(1),x(2),x(3),x(4),x(5),x(6),u(1),u(2)));
    x = double(f(x(1),x(2),x(3),x(4),x(5),x(6),u(1),u(2))); %x + freq*[zeros(3,3) eye(3); zeros(3,6)]*x + [.5*freq^2*ones(3,1); freq*ones(3,1)].*[acc; acc];
%     x = (freq*[zeros(3,3) eye(3); zeros(3,6)]+eye(6))*x + ...
%         freq*double(f(x(1),x(2),x(3),x(4),x(5),x(6),u(1),u(2))) + ...
%         double(f2(x(1),x(2),x(3),x(4),x(5),x(6),u(1),u(2))*u);
    x(2:3) = x(2:3) - 2*pi*floor((x(1:2)+pi)/(2*pi));

    % Update Linearization
    A_now = double(A(x(1),x(2),x(3),x(4),x(5),x(6),u(1),u(2)));
    B_now = double(B(x(1),x(2),x(3),x(4),x(5),x(6),u(1),u(2)));
    H_now = double(H(x(1),x(2),x(3),x(4),x(5),x(6),u(1),u(2)));
%     A_now = double(A(0,0,0,0,0,0,0,0));
%     B_now = double(B(0,0,0,0,0,0,0,0));
%     H_now = double(H(0,0,0,0,0,0,0,0));

    q = H_now*x;
    q(2:3) = q(2:3) - 2*pi*floor((x(1:2)+pi)/(2*pi));
    [K,~,~] = dlqr(A_now, B_now, Q, R);

    % u = u + K*(q_ref-x)
    u = K*(q_ref-q);
    u(1) = max(min(1,u(1)), -1);
    u(2) = max(min(1,u(2)), -1);
    
    i = i + 1;
    if (mod(i, cast(1/50 / freq,"uint8")) == 0)
        % Edit Figure
        [gx1,y1,gx2,y2] = coords(x(1),L1,Lc2,x(2),x(3));
        set(l1,'XData',[x(1) gx1],'YData',[0 y1]);
        set(l2,'XData',[gx1 gx2],'YData',[y1 y2]);
        set(ball,'XData',gx2,'YData',y2);
        set(circle,"Position",[x(1)-r -r 2*r 2*r]);
    end
    pause(freq)
end

function [x1,y1,x2,y2] = coords(x,L1,L2,theta,theta_2)
    x1 = x + L1*cos(theta+pi/2);
    y1 = L1*sin(theta+pi/2);
    x2 = x1 + L2*cos(theta+theta_2+pi/2);
    y2 = y1 + L2*sin(theta+theta_2+pi/2);
end
