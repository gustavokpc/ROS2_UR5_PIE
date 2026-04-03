function pose_robot = coordConvert(pose_cage)
    % coordConvert
    % Converte pose da cage no formato:
    %   [rot_x rot_y rot_z rot_w X Y Z]
    % para pose do robo no formato:
    %   [tz ty tx x y z]
    %
    % Entrada:
    %   pose_cage -> vetor 1x7 = [qx qy qz qw x y z]
    %
    % Saida:
    %   pose_robot -> vetor 1x6 = [tz ty tx x y z]

    % =========================
    % Matrizes de calibracao
    % =========================
    X_gc = [ 0.7329    0.6785    0.0499    0.0007;
            -0.0454    0.1220   -0.9915   -0.0300;
            -0.6788    0.7244    0.1202   -0.0309;
             0         0         0         1.0000];

    X_bc = [-0.0252    0.9996    0.0134    0.5261;
            -0.9997   -0.0252   -0.0001   -0.2224;
             0.0003   -0.0134    0.9999    0.3846;
             0         0         0         1.0000];

    % valida entrada
    if ~isnumeric(pose_cage) || numel(pose_cage) ~= 7
        error('A entrada deve ser um vetor numerico 1x7 no formato [rot_x rot_y rot_z rot_w X Y Z].');
    end

    pose_cage = reshape(pose_cage, 1, 7);

    % 1) pose da cage -> matriz homogênea
    Tcg = cameraPose2T(pose_cage);

    % 2) base_cage -> base_robo
    %    Tbg = X_bc * Tcg * inv(X_gc)
    Tbg = X_bc * Tcg / X_gc;

    % 3) matriz homogênea -> [tz ty tx x y z]
    pose_robot = T2robotPose(Tbg);
end

function T = cameraPose2T(pose)
    % pose = [qx qy qz qw x y z]

    qx = pose(1);
    qy = pose(2);
    qz = pose(3);
    qw = pose(4);
    x  = pose(5);
    y  = pose(6);
    z  = pose(7);

    q = [qw qx qy qz];
    q = q / norm(q);

    qw = q(1);
    qx = q(2);
    qy = q(3);
    qz = q(4);

    R = [1 - 2*(qy^2 + qz^2),   2*(qx*qy - qz*qw),   2*(qx*qz + qy*qw);
         2*(qx*qy + qz*qw),     1 - 2*(qx^2 + qz^2), 2*(qy*qz - qx*qw);
         2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),   1 - 2*(qx^2 + qy^2)];

    T = [R [x; y; z];
         0 0 0 1];
end

function pose = T2robotPose(T)
    % Converte matriz homogênea para:
    % [tz ty tx x y z]
    % assumindo R = Rz(tz) * Ry(ty) * Rx(tx)

    R = T(1:3,1:3);
    x = T(1,4);
    y = T(2,4);
    z = T(3,4);

    if abs(R(3,1)) < 1 - 1e-10
        ty = -asin(R(3,1));
        tz = atan2(R(2,1), R(1,1));
        tx = atan2(R(3,2), R(3,3));
    else
        ty = -asin(max(min(R(3,1),1),-1));

        if R(3,1) <= -1 + 1e-10
            ty = pi/2;
            tz = atan2(-R(1,2), R(2,2));
            tx = 0;
        else
            ty = -pi/2;
            tz = atan2(-R(1,2), R(2,2));
            tx = 0;
        end
    end

    pose = [tz ty tx x y z];
end