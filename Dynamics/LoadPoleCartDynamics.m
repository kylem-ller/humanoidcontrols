function LoadPoleCartDynamics(kinematics, freq)
    [A, B, f, ~] = LoadFullDynamics(kinematics, false, freq);

    x0 = sym('x',[4 1]);
    u0 = sym('u',[1 1]);
    s0 = [x0; u0];

    A = A(x0(1), x0(2), 0, x0(3), x0(4), 0, u0(1), 0);
    B = B(x0(1), x0(2), 0, x0(3), x0(4), 0, u0(1), 0);
    f = f(x0(1), x0(2), 0, x0(3), x0(4), 0, u0(1), 0);

    i = [3 6];
    A(i,:) = [];
    A(:,i) = [];
    B(i,:) = [];
    B(:,2) = [];
    f(i,:) = [];

    A(s0) = A;
    B(s0) = B;
    f(s0) = f;

    assign(A)
    assign(B)
    assign(f)
end

function assign(var)
    assignin('base',inputname(1),var);
end