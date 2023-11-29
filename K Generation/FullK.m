freq = 20 / 1000; % s to ms
LoadFullDynamics(@LoadRobotKinematics, true, freq);

R = diag([1/(1^2) 1/(1^2)]);
Q = diag([1/(0.2)^2, 1/(pi/16)^2, 1/(pi/4)^2, 1/(wm*r/Gw)^2, 1/(pi/32)^2, 1/(pi/4)^2]);

% Initial Figure
n = 5;
p1 = linspace(-3*pi/4, 3*pi/4, 100);
p2 = linspace(-3*pi/4, 3*pi/4, 100);
vt = linspace(-pi, pi, 100);
v1 = linspace(-pi, pi, 100);
v2 = linspace(-pi, pi, 100);
Vw = linspace(-1, 1, 100);
V2 = linspace(-1, 1, 100);

KStored = zeros(n, n, n, n, n, n, 2, 6);
HStored = zeros(n, n, n, n, n, n, 6, 6);

counter = 1;

for i = 1:n
    for j = 1:n
        for k = 1:n
            for l = 1:n
                for n = 1:n
                    for o = 1:n
                        A_now = double(A(0,p1(i),p2(j),vt(k),0,v2(l),Vw(n),V2(o)));
                        B_now = double(B(0,p1(i),p2(j),vt(k),0,v2(l),Vw(n),V2(o)));
                        H_now = double(H(0,p1(i),p2(j),vt(k),0,v2(l),Vw(n),V2(o)))
                    
                        [K,~,~] = dlqr(A_now, B_now, Q, R)

                        % Add K to K array
                        % Add H_now to K array

                        % In arduino:
                        % Map states to index: ex: p1_index, from -pi,
                        % pi to 0, 99
                        % K = KStored[p1_index][p2_index]...
                        % matrix multiplication -> voltages = K * (goal - H * state)'
                        
                        KStored(i,j,k,l,n,o,:,:) = K;
                        HStored(i,j,k,l,n,o,:,:) = H_now;

                        counter = counter + 1
                    end
                end
            end
        end
    end
end

uploadToArduino(KStored, HStored);