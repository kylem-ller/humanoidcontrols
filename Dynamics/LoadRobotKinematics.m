function [r, L1, Lc1, Lc2, mw, m1, m2, Ic1, Ic2, Gw, G2, wm, Tm] = LoadRobotKinematics()
    r = 40 / 1000; % mm to m
    
    wheelZ = -0.935; % in
    armZ = 6.031; % in
    L1 = (armZ - wheelZ) * 0.0254; % in to m
    Lc1 = (0.524 - wheelZ) * 0.0254; % in to m
    Lc2 = 1.5 * sqrt(0.853^2 + (5.378 - armZ)^2) * 0.0254; % in to m
    
    mw = 0.05; % kg
    m1 = 2.097 * 0.453592; % lb to kg
    m2 = 0.8 * 0.559 * 0.453592; % lb to kg
    
    Ic1 = 0.194 * 0.0002926397; % lb-in^2 to kg-m^2
    Ic2 = 0.663 * 0.0002926397; % lb-in^2 to kg-m^2
    
    Gw = 40;
    G2 = 45;
    overvolt = 16 / 12;
    wm = overvolt * 2 * 3.1415 * 6000 / 60; % rad/s
    Tm = overvolt * 0.0025; % Nm

    assign(r);
    assign(L1);
    assign(Lc2);
    assign(Gw);
    assign(G2);
    assign(wm);
end

function assign(var)
    assignin('base',inputname(1),var);
end