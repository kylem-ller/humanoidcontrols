function [A, B, f, H] = LoadFullDynamics(kinematics, includeTransfer, freq)
    [r, L1, Lc1, Lc2, mw, m1, m2, Ic1, Ic2, Gw, G2, wm, Tm] = kinematics();
    g = 9.81; % m/s^2

    % State & Input Variables
    x0 = sym('x',[6 1]);
    u0 = sym('u',[2 1]);
    s0 = [x0; u0];
    pt = x0(1);
    p1 = x0(2);
    p2 = x0(3);
    vt = x0(4);
    v1 = x0(5);
    v2 = x0(6);
    Vw = u0(1);
    V2 = u0(2);
    
    % Equations Of Motion
    sigma1 = -m1*Lc1*cos(p1)+m2*(-L1*cos(p1)-Lc2*cos(p1+p2));
    sigma2 = -m2*Lc2*cos(p1+p2);
    sigma3 = Ic1+Ic2+m1*Lc1^2+m2*(L1^2+Lc2^2+2*L1*Lc2*cos(p2));
    sigma4 = Ic2+m2*Lc2*(Lc2+L1*cos(p2));
    M = [mw+m1+m2 sigma1 sigma2;...
         sigma1 sigma3 sigma4;...
         sigma2 sigma4 Ic2+m2*Lc2^2];
    
    V = [m1*Lc1*v1^2*sin(p1)+m2*(L1*v1^2*sin(p1)+Lc2*(v1+v2)^2*sin(p1+p2));...
         -m2*L1*Lc2*v2*(2*v1+v2)*sin(p2);...
         m2*L1*Lc2*v1^2*sin(v2)];
    
    G = [0;...
         -m1*g*Lc1*sin(p1)+m2*g*(-L1*sin(p1-Lc2*sin(p1+p2)));...
         -m2*g*Lc2*sin(p1+p2)];
    
    T = [Gw*Tm./r*(Vw-(vt./r+v1)./wm*Gw);...
         0;...
         G2*Tm*(V2-v2./wm*G2)];
    
    % Linearized State Space
    % f = M^(-1)*(T-V-G);
    acc = M^(-1)*(T-V-G);
    f = (x0 + freq*[x0(end/2+1:end); acc] + .5*freq^2*[acc; zeros(3,1)]);

    % f = [x0(end/2+1:end); M^(-1)*(T-V-G)];
    % f2 = 0.5*freq^2*jacobian([M^(-1)*(T-V-G); zeros(3,1)], u0);
    A = jacobian(f, x0);
    % A = jacobian([x0(end/2+1:end); f], x0);
    % A = freq * A + eye(6);
    % B = jacobian([.5*freq^2*f; freq*f], u0);
    B = jacobian(f, u0);
    
    % Transfer
    if (includeTransfer)
        L3 = sqrt(L1^2+Lc2^2+2*L1*Lc2*cos(p2));
        dL3 = -(L1*Lc2*v2*sin(p2))/L3;
        p2c = p1 + asin(Lc2*sin(p2)/L3);
        v2c = v1 + Lc2/L3*(v2*cos(p2)-(dL3/L3*sin(p2)))/(L3^2*sqrt(1-((Lc2*sin(p2))/L3)^2));
%         pc = m1*g*Lc1*sin(p1)+m2*g*L3*sin(p2);%(m1*p1+m2*p2c)/(m1+m2);
%         vc = m1*g*Lc1*v1*cos(p1)+m2*g*(L3*v2*cos(p2)+dL3*sin(p2));%(m1*v1+m2*v2c)/(m1+m2);
        pc = (m1*p1+m2*p2c)/(m1+m2);
        vc = (m1*v1+m2*v2c)/(m1+m2);

        f2 = [pt; pc; p2; vt; vc; v2];
        H(s0) = jacobian(f2, x0);
        assign(H);

        A = H * A * H^-1;
        B = H * B;
    else
        H = 0;
    end

    % Discretized State Space
%     A = freq * A + eye(6);
%     B = freq * B;

    A(s0) = A;
    B(s0) = B;
    f(s0) = f;
    f2(s0) = f2;

    assign(A);
    assign(B);
    assign(f);
    assign(f2);
end

function assign(var)
    assignin('base',inputname(1),var);
end