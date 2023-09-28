% Initial States
L = 5/1000; %m
g = 9.81;
t = 0; %s
t2 = 0;
freq = 1/5000; %ms

theta_2 = 90;
theta = 0;
w_2 = 0;
w = 0;
a_2 = 0;
a = -g*cosd(theta+(90-theta_2/2))/(2*L*sind(theta_2/2));

% Ranges
theta_2_min = 30;
theta_2_max = 180;

% theta 2 = 30-180
% d/dt*d/dt theta = 1/(2*L*sin(theta_2/2))*g*cos(theta-(90-theta_2/2))

% Initial Figure
f = figure();
xlim((2.1*L)*[-1 1]);
ylim((2.1*L)*[-1 1]);
[x1,y1,x2,y2] = coords(L,theta,theta_2);
hold on
l1 = plot([0 x1],[0 y1],'linewidth',2);
l2 = plot([x1 x2],[y1 y2],'linewidth',2);
ball = plot(x2,y2,'.','MarkerSize',40);
kb = HebiKeyboard();

while (true)
    state = read(kb);

    % Update Pos
    theta = theta + w*freq + a*freq^2/2;
    theta_2 = theta_2 + w_2*freq;
    if theta_2 < theta_2_min
        theta_2 = theta_2_min;
    elseif theta_2 > theta_2_max
        theta_2 = theta_2_max;
    end

    % Upate Vel
    w = w + a*freq;
    if all(state.keys('a'))
        w_2 = 100;
    elseif all(state.keys('d'))
        w_2 = -100;
    else
        w_2 = 0;
    end
    
    % Update Acc
    a = -g*cosd(theta-(90-theta_2/2))/(2*L*sind(theta_2/2));
    a = a-abs(w)/w*0.02*g/(2*L);
    
    t = t + freq;
    t2 = t2 + 1;
    if (mod(t2, 30) == 0)
        % Edit Figure
        [x1,y1,x2,y2] = coords(L,theta,theta_2);
        set(l1,'XData',[0 x1],'YData',[0 y1]);
        set(l2,'XData',[x1 x2],'YData',[y1 y2]);
        set(ball,'XData',x2,'YData',y2);
    end
    pause(freq)
end

function [x1,y1,x2,y2] = coords(L,theta,theta_2)
    x1 = L*cosd(theta);
    y1 = L*sind(theta);
    x2 = x1 + L*cosd(theta+theta_2+180);
    y2 = y1 + L*sind(theta+theta_2+180);
end