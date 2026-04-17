function [qSol, info] = computeIK_UR5(ctx, p)
    % ctx = contexto criado no setup
    % p   = [x y z]

    yaw = atan2(p(2), p(1)) - pi/2;

    % p   = [thetaZ thetaY thetaX x y z]

    thetaZ = yaw;
    thetaY = 0;
    thetaX = -1.5708;
    x = p(1);    y = p(2);
    z = p(3);

    Rz = axang2rotm([0 0 1 thetaZ]);
    Ry = axang2rotm([0 1 0 thetaY]);
    Rx = axang2rotm([1 0 0 thetaX]);
    R  = Rz * Ry * Rx;

    T = eye(4);
    T(1:3,1:3) = R;
    T(1:3,4)   = [x; y; z];

    % Seed: juntas atuais do robô
    try
        q0 = getJointConfiguration(ctx.ur);
    catch
        q0 = homeConfiguration(ctx.ur5);
    end
    
    q0 = wrapToPi(q0);
    [qSol, info] = ctx.ik(ctx.eeName, T, ctx.weights, q0);
    qSol = wrapToPi(qSol);
    bestSol = qSol;
    % bestCost = norm(qSol(2) - q0(2));
    bestCost = norm(qSol - q0);
    
    for k = 1:20
        qTest = ctx.ik(ctx.eeName, T, ctx.weights, q0 + 0.2*randn(size(q0)));
        qTest = wrapToPi(qTest);
        % cost = norm(qTest(2) - q0(2));
        cost = norm(qTest - q0);
        
        if cost < bestCost
            bestSol = qTest;
            bestCost = cost;
        end
    end

    qSol = bestSol;

    disp("Melhor solucao encontrada")
    disp(qSol);
end