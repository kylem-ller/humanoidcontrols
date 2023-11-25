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

Pm_c = 2*15*1000;
wm_c = 150 * 2.21817385041;
w_c = 130 * 2.21817385041;

syms Pm wm w Pm_t
assume(Pm,"real")
assume(wm,"real")
assume(w,"real")
assume(Pm,"positive")
assume(wm,"positive")
assume(w,"positive")

c = 4*Pm_c/(I*wm_c^2*w_c^2)*(wm_c-w_c);

% a_s1(Pm,wm,w) = 4*Pm/(I*wm^2)*(wm-w);
% a_s2(Pm,wm,w) = a_s1(Pm,wm,w) + c*w^2;
% 
% inv_a_s1(Pm,wm,w) = 1/a_s1(Pm,wm,w);
% inv_a_s2(wm,w) = 1/a_s2(Pm,wm,w);

t_s1(Pm,wm,w) = I*wm^2/(4*Pm)*(log(wm) - log(wm - w));%int(1/a_s1(Pm,wm,w),0,w);
A(Pm,wm,w) = (4*Pm)/(I*wm^2);
t_s2222(Pm,wm,w) = sqrt(A(Pm,wm,w))*sqrt(A(Pm,wm,w) - 4*wm*c);
t_s222(Pm,wm,w) = 2*atanh((A(Pm,wm,w) - 2*c*w)/t_s2222(Pm,wm,w))/t_s2222(Pm,wm,w);
t_s2(Pm,wm,w) = t_s222(Pm,wm,w) - t_s222(Pm,wm,0);
% t_s2(Pm,wm,w) = int(inv_a_s2,w,0,w);
hold on
fplot(t_s1(Pm_c,wm_s2,w), w/0.10472, [0, 1000]);
fplot(t_s2(Pm_c,wm_s2,w), w/0.10472, [0, 1000]);
xlim([0,10]);
ylim([0,9000]);
% min1 = 0 == 2*log(wm) - 2*log(wm-w_arr1(i)) - w_arr1(i)/(wm-w_arr1(i));

% t = linspace(0.01,10,100);
% Pm_twm = linspace(0.01,10,100);
% for i = 1:length(t)
%     eqn1 = t(i) == t_s1(Pm,wm,w_s1);
%     Pm_t(wm) = solve(eqn1, Pm);
%     min = diff(Pm_t, wm);
%     wm_t = solve(0 == min, Pm_t);
%     Pm_twm(i) = double(Pm_t(wm_t));
% end
% plot(t, Pm_twm)

% w_arr1 = linspace(10,w_s1,100);
% 
% w_arr2 = linspace(0.1,w_s1,100);
% wm_arr1 = linspace(0.1,w_s1,100);
% wm_arr2 = linspace(0.1,w_s1,100);
% for i = 1:length(t)
%     min1 = 0 == 2*log(wm) - 2*log(wm-w_arr1(i)) - w_arr1(i)/(wm-w_arr1(i));
%     %min2 = diff(t_s2(Pm_c,wm,w_arr2(i)), wm);
%     test = vpasolve(min1, wm, w_arr1(i))
%     wm_arr1(i) = vpasolve(min1, wm, w_arr1(i));
%     %wm_arr2(i) = double(solve(0 == min2, wm));
% end
% plot(w_arr1, wm_arr1)%, w_arr2, wm_arr2)
% w_arr2 = linspace(0.01,600,100);




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