% Physics Setup
l = 1/1000; %m
g = 9.81;
freq = 1/5000; %ms

% Initial States
theta_1 = -pi/4;
theta_2 = pi/2;
theta = get_theta(theta_1, theta_2);
w = 0;
w_2 = 0;

theta_1_goal = -pi/2;
theta_2_goal = 3*pi/4;
theta_goal = get_theta(theta_1_goal, theta_2_goal);

% Ranges
theta_2_min = pi/6;
theta_2_max = pi;
max_a = 50;

% Control System
% R = [3*pi/4 0 pi/2 0]';
R = [theta_goal 0 theta_2_goal 0]';
B = [0 0 0 1]';
C = [1 0 0 0; 0 0 1 0];
D = [0 0]';
dF = [0 0 0 0]';

% Transfer Functions
s = tf('s');
Kp = 5;
Ki = 5;
Kd = 5;
PID = Kp + Ki/s + Kd*s;

% Initial States
theta = [theta 0 0];
theta_2 = [theta_2 0 0];
w = [w 0 0];
w_2 = [w_2 0 0];
a = [0 0 0];
a_2 = [0 0 0];

state_fb = [0 0 0 0]';
input_fb = [0 0 0 0]';
x_est = [0 0 0 0]';
y_est = [0 0]';

KPID = [0 0 0];
KUPID = [0 0 0];

% Initial Figure
f = figure();
xlim((2.1*l)*[-1 1]);
ylim((2.1*l)*[-1 1]);
[x1,y1,x2,y2] = coords(l,theta(1),theta_2(1));
[x_goal1,y_goal1,x_goal2,y_goal2] = coords(l,theta_goal,theta_2_goal);
hold on
l1 = plot([0 x1],[0 y1],'linewidth',2);
l2 = plot([x1 x2],[y1 y2],'linewidth',2);
ball = plot(x2,y2,'.','MarkerSize',40);
goal1 = plot(x_goal1,y_goal1,'.','MarkerSize',20);
goal2 = plot(x_goal2,y_goal2,'.','MarkerSize',20);

errw = 0;
errt = 0;
i = 0;
while (true)
    % Update Physics
    a = shift(a, -g*cos(theta(1)-(pi/2-theta_2(1)/2))/(2*l*sin(theta_2(1)/2)));
% 
    %theta = z_transfer(theta, [w; a], [1/s; 1/s^2], freq);
    %theta = z_transfer(theta, w, 1/s, freq);
%     theta(1) = mod(theta(1), 2*pi);
% 
%     theta_2 = z_transfer(theta_2, [w_2; a_2], [1/s; 1/s^2], freq);

   %w = z_transfer(w, a, 1/s, freq);
%     w_2 = z_transfer(w_2, a_2, 1/s, freq);

   theta = shift(theta, mod(theta(1) + w(1)*freq + a(1)*freq^2/2, 2*pi));
   theta_2 = shift(theta_2, mod(theta_2 + w_2*freq + a_2*freq^2/2, 2*pi));
   theta_2(1) = max(theta_2_min,min(theta_2_max,theta_2(1)));

   w = shift(w, w(1) + a(1)*freq);
   w_2 = shift(w_2, w_2(1) + a_2(1)*freq);
    
    % Update Controls System
%     C1 = g*sin(theta-theta_2/2)/(2*l*sin(theta_2/2));
%     C2 = g*cos(theta-theta_2/2)/(2*l*sin(theta_2/2));
%     C3 = g*sin(theta)/(2*l*(cos(theta_2/2)-1));
% 
%     A = [0 1 0 0; C2 0 C3 0; 0 0 0 1; 0 0 0 0];
%     dF = [0 C1-C2*theta-C3*theta_2 0 0]'; %- dF;
%     poles = [-3+5*1i -3-5*1i -1.6+2*1i -1.6-1*2i];
%     eig = [-7+7i -7-7i -12+12i -12-12i];
%     K = acker(A, B, poles);
%     L = place(A',C', eig)';

%     % Observer
%     err_fb = L*[dist(theta,y_est(1)) dist(theta_2,y_est(2))]';
%     x_est = x_est + freq*(state_fb+input_fb+err_fb) + dF; %%% integral
%     state_fb = A*x_est;
%     y_est = C*x_est;
%     a_2 = K*mod(R-x_est,2*pi);
%     a_2 = max(-max_a,min(max_a,a_2));
%     input_fb = B*a_2;
%     fprintf("theta: %.2f, theta2: %.2f\nest:   %.2f, est:    %.2f\ngoal:  %.2f, goal:   %.2f\na_2: %.3f\n\n", mod([theta theta_2 y_est(1) y_est(2) theta_goal theta_2_goal],2*pi),a_2);

    % Derviative Feedback
%     Ak = A - B*K;
%     [num, den] = ss2tf(Ak, B*K, C, zeros(1,4), 1);
%     G = tf(num(1,:), den);
%     G2 = tf(num(2,:), den);
%     a_2 = K*[dist(R(1),theta) R(2)-w dist(R(3),theta_2) R(4)-w_2]';

%     sys = ss(A,B,[0 0 1 0],0);
%     K = pid(sys);

    
    err = dist(R(3),theta_2);
%     KUPID = [err KUPID(1) KUPID(2)];
%     KPID = [(2012*KUPID(1)-4000*KUPID(2)+1988*KUPID(3)+4E-6*KPID(1)-KPID(2)) KPID(1) KPID(2)];
% 
%     K = KPID(1);
%     a_2 = K*err;
%     a_2 = max(-max_a,min(max_a,a_2));
%     fprintf("err: %.2f, a_2: %.2f\n", [err a_2])
    
    i = i + 1;
    if (mod(i, cast(1/60/10 / freq,"uint8")) == 0)
        % Edit Figure
        [x1,y1,x2,y2] = coords(l,theta(1),theta_2(1));
        set(l1,'XData',[0 x1],'YData',[0 y1]);
        set(l2,'XData',[x1 x2],'YData',[y1 y2]);
        set(ball,'XData',x2,'YData',y2);
        set(goal2,'XData',x_goal2,'YData',y_goal2);
        set(goal1,'XData',x_goal1,'YData',y_goal1);
    end
    pause(freq*10)
end

function [x1,y1,x2,y2] = coords(l,theta,theta_2)
    x1 = l*cos(theta);
    y1 = l*sin(theta);
    x2 = x1 + l*cos(theta+theta_2+pi);
    y2 = y1 + l*sin(theta+theta_2+pi);
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
%     [z_u, z_y] = tfdata(c2d(tf, freq, 'tustin'));
%     z_u = cell2mat(z_u);
%     z_y = cell2mat(z_y(2:end));
% 
%     %y_0 = shift(y, 0)
%     y_new = sum(z_u.*u(1:length(z_u))) - sum(z_y.*y(2:length(z_y)+1));
%     y = shift(y, y_new);
end

function y = shift(y, x)
    y = [x y(1:end-1)];
end

function theta = get_theta(theta_1, theta_2)
    theta = theta_1 + pi/2 - theta_2/2;
end