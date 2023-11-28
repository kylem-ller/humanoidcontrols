function LoadAcrobatDynamics(kinematics, includeTransfer, freq)
    [A, B, f, H] = LoadFullDynamics(kinematics, includeTransfer, freq);

    x0 = sym('x',[4 1]);
    u0 = sym('u',[1 1]);
    s0 = [x0; u0];

    A = A(0, x0(1), x0(2), 0, x0(3), x0(4), 0, u0(1));
    B = B(0, x0(1), x0(2), 0, x0(3), x0(4), 0, u0(1));
    f = f(0, x0(1), x0(2), 0, x0(3), x0(4), 0, u0(1));

    i = [1 4];
    A(i,:) = [];
    A(:,i) = [];
    B(i,:) = [];
    B(:,1) = [];
    f(i,:) = [];

    A(s0) = A;
    B(s0) = B;
    f(s0) = f;

    assign(A)
    assign(B)
    assign(f)

    if includeTransfer
        H = H(0, x0(1), x0(2), 0, x0(3), x0(4), 0, u0(1));
        H(i,:) = [];
        H(:,i) = [];
        H(s0) = H;
        assign(H)
    end
end

function assign(var)
    assignin('base',inputname(1),var);
end