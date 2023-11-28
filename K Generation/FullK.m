freq = 20 / 1000; % s to ms
LoadFullDynamics(@LoadRobotKinematics, true, freq);

R = diag([1/(0.5^2) 1/(1^2)]);
Q = diag([1/(0.2)^2, 1/(pi/16)^2, 1/(pi/4)^2, 1/(wm*r/Gw)^2, 1/(pi/32)^2, 1/(pi/4)^2]);

% Initial Figure
p1s = linspace(-3*pi/4, 3*pi/4, 100);
p2s = linspace(-3*pi/4, 3*pi/4, 100);
vts = linspace(-pi, pi, 100);
v1s = linspace(-pi, pi, 100);
v2s = linspace(-pi, pi, 100);
Vws = linspace(-1, 1, 100);
V2s = linspace(-1, 1, 100);
for p1 = p1s
    for p2 = p2s
        for vt = vts
            for v1 = v1s
                for v2 = v2s
                    for Vw = Vws
                        for V2 = V2s
                            A_now = double(A(0,p1,p2,vt,v1,v2,Vw,V2));
                            B_now = double(B(0,p1,p2,vt,v1,v2,Vw,V2));
                            H_now = double((0,p1,p2,vt,v1,v2,Vw,V2));
                        
                            Add Ks
                            [K,~,~] = dlqr(A_now, B_now, Q, R);
                        end
                    end
                end
            end
        end
    end
end

uploadToArduino(K, 0, 0);