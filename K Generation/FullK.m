freq = 20 / 1000; % s to ms
LoadFullDynamics(@LoadRobotKinematics, true, freq);

R = diag([1/(1^2) 1/(1^2)]);
Q = diag([1/(0.2)^2, 1/(pi/16)^2, 1/(pi/4)^2, 1/(5*wm*r/Gw)^2, 1/(pi/32)^2, 1/(pi/4)^2]);

% Initial Figure
n = 3;
p1 = linspace(-pi/2, pi/2, n);
p2 = linspace(-pi/2, pi/2, n);
vt = linspace(-wm*r, wm*r, n);
v1 = linspace(-pi, pi, n);
v2 = linspace(-pi, pi, n);
Vw = linspace(-1, 1, n);
V2 = linspace(-1, 1, n);

KStored = zeros(n, n, n, 2, 6);
HStored = zeros(n, n, n, 6, 6);

counter = 1;

for i = 1:n
    for j = 1:n
        for k = 1:n
            A_now = double(A(0,0,p2(i),0,0,v2(j),Vw(k),0));
            B_now = double(B(0,0,p2(i),0,0,v2(j),Vw(k),0));
            H_now = double(H(0,0,p2(i),0,0,v2(j),Vw(k),0));
        
            [K,~,~] = dlqr(A_now, B_now, Q, R);
            
            KStored(i,j,k,:,:) = K;
            HStored(i,j,k,:,:) = H_now;
    
            counter = counter + 1;
        end
    end
end

uploadToArduino(KStored, HStored);