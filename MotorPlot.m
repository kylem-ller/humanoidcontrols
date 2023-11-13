% l = 3.5 * 0.0254;
% m = 0.25;
% 
% wrpm = 60/2/pi*w_log;
% check = int8(abs(wrpm) < 100);
% ta = m*l^2*a_log;
% 
% 
% 
% hold on
% %plot(abs(wrpm(1:564)), abs(ta(1:564)))
% %plot([0 130], [1.2/10*2 0])
% plot([95 95], [0 0.0697])
% plot([0 150], [0.95/10*2 0])
% plot([0 110.05], [.05 .05])
% ylabel("Torque (Nm)")
% xlabel("Velocity (rpm)")
% title(" Drive Motor Torque & Velocity")
% %title("Simulation vs Motor Torque & Velocity")
% legend("Reaction Torque","Motor","Target Vel")

I = 0.3177732654;
w_s1 = 6000*0.10472;
wm_s2 = 8400*0.10472;
Pm_s12 = 2*15*1000;

syms Pm wm w
assume(Pm,"real")
assume(wm,"real")
assume(w,"real")
assume(Pm,"positive")
assume(wm,"positive")
assume(w,"positive")

c(wm,w) = 4*Pm_s12/(I*wm^2*w^2)*(wm-w);

a_s1(Pm,wm,w) = 4*Pm/(I*wm^2)*(wm-w);
a_s2(Pm,wm,w) = a_s1(Pm,wm,w) + c(Pm,wm,w)*w^2;

inv_a_s1(Pm,wm,w) = 1/a_s1(Pm,wm,w);
inv_a_s2(wm,w) = 1/a_s2(Pm,wm,w);

t_s1(Pm,wm,w) = integral(inv_a_s1,0,w);
t_s2(Pm,wm,w) = integral(inv_a_s2,0,w);

t = linspace(0.01,10,100);
for i = 1:length(t)
    Pm_t = solve t[i] = t_s1(Pm, wm) for Pm @ w
    min = diff(Pm_t, wm) @ w
    wm_t = solve 0 = min for wm
    Pm_twm[i] = Pm_t(wm_t)
end

w_arr1 = linspace(0.01,wm*0.995,100);
for i = 1:length(t)
    min = diff(Pm_s12, wm) @ w_arr1
    wm_t = solve 0 = min for wm
    t_wm2[i] = t_s1(Pm_s12)
end
w_arr2 = linspace(0.01,600,100);




% w_1 = 6000*0.10472;
% wm_2 = 8400*0.10472;
% I = 0.3177732654;
% Pm = 15*1000*2;
% x = linspace(0.01,10,100);
% y = I*wm_2^2./x/4*(log(wm_2) - log(wm_2 - w_1))/1000;
% plot(x, y)
% xlim([0 10]);
% ylim([0 50]);
% xlabel("Acceleration Time (s)")
% ylabel("Max Motor Power (kW)")
% title("Motor Power Needed @ 6000 rpm target")

% y = linspace(0.01,wm*0.995,100);
% x = I*wm^2./Pm*(log1p(wm) - log1p(wm - y))/4;
% plot(x, y/0.10472, [0 10], [w w]/0.10472)
% xlim([0 10]);
% xlabel("Acceleration Time (s)")
% ylabel("Velocity (rpm)")
% title("Weapon Acceleration")