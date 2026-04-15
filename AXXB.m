clear; clc;

X_gc_encontrado = [ 0.7329    0.6785    0.0499    0.0007
                   -0.0454    0.1220   -0.9915   -0.0300
                   -0.6788    0.7244    0.1202   -0.0309
                    0         0         0         1.0000];

X_bc_encontrado = [-0.0252    0.9996    0.0134    0.5261
                   -0.9997   -0.0252   -0.0001   -0.2224
                    0.0003   -0.0134    0.9999    0.3846
                    0         0         0         1.0000];


% [tz ty tx x y z]
robot_poses = [
  -0.000047 -0.000036 -1.570796 -0.000000 0.401500 1.001100;
  0.390418 0.145707 -1.465647 -0.848300 0.563000 0.343200;
  -0.575100 -0.060274 -2.151467 0.639300 0.914800 0.096900;
  -1.586586 0.815573 -1.580463 0.402800 0.539800 0.573100;
  0.943815 0.632761 -2.262261 -0.388100 0.806400 0.280400;
  1.828574 -0.425923 -1.560268 -0.566200 0.112200 0.864400;
  2.976858 -0.343972 -1.562936 -0.392500 -0.084300 0.834200;
  1.041092 -0.091403 -1.486165 -0.759100 -0.225900 0.579400;
  -2.277017 0.213360 -1.989544 0.002800 -0.948400 0.050800;
  -2.045034 -0.254267 -1.825729 0.475700 -0.873400 0.200000;
  -2.520400 -0.666875 -0.863983 0.295500 -0.518900 0.909000;
  -1.267814 0.242504 -2.064393 0.797300 -0.440100 0.254800;
  -1.399817 -0.401406 -1.709991 0.820400 -0.077400 0.655000;
  0.088449 1.034095 -1.286044 0.533600 0.564500 0.677700;
  2.603074 0.867138 -1.491742 -0.836700 0.130200 0.376600;
  -1.563078 0.023653 -1.316831 0.183100 -0.096700 0.561200;
  -0.021176 0.101102 -1.566060 -0.030700 0.187000 0.401900;
  2.061384 0.011970 -1.583646 -0.345100 -0.071800 0.923300;
  1.669629 0.033195 -1.169373 -0.326200 0.734500 0.572600;
  -2.494136 0.261889 -1.751178 -0.308500 -0.798500 0.389300;
  -2.494224 0.261926 -1.751182 -0.308600 -0.798500 0.389200;
];

% [rot_x rot_y rot_z rot_w X Y Z]
cage_poses = [
  -0.014356 0.009874 -0.675950 -0.736741 -0.300124 -0.046563 1.376329;
  0.033341 -0.077896 -0.792379 -0.604115 -0.468406 -0.866127 0.694618;
  -0.284885 -0.112362 0.447365 0.840286 -0.834712 0.605645 0.482723;
  -0.029039 0.367660 -0.080774 0.925991 -0.455924 0.337634 0.939923;
  -0.357216 -0.242713 0.875520 0.216683 -0.757395 -0.395607 0.655409;
  0.843495 -0.500867 -0.154796 0.116988 -0.043052 -0.571901 1.228962;
  -0.114814 -0.107803 -0.786635 0.596994 0.112784 -0.418301 1.202191;
  0.080435 0.015568 0.946022 0.313575 0.316144 -0.776694 0.938303;
  0.182166 -0.247255 0.329393 -0.892850 1.009802 -0.059244 0.438643;
  -0.172141 -0.049714 -0.306593 0.934824 0.927724 0.413889 0.584172;
  0.128238 -0.373514 -0.374811 0.838784 0.553439 0.221220 1.256583;
  0.281011 -0.080383 -0.151890 -0.944193 0.532067 0.745044 0.648670;
  -0.044693 0.170001 -0.016997 -0.984283 0.129790 0.724605 1.097245;
  0.349809 -0.409190 -0.560560 -0.629261 -0.449088 0.522060 1.038112;
  0.381545 0.254977 -0.791262 0.404123 -0.107719 -0.872432 0.727831;
  0.114428 0.046599 -0.049080 0.991124 0.166028 0.097729 0.928920;
  -0.023742 0.039467 0.643632 0.763948 -0.079617 -0.069209 0.777717;
  0.027386 0.056118 -0.972444 0.224619 0.124117 -0.359568 1.295950;
  -0.035752 0.205519 0.977859 0.016578 -0.671395 -0.317701 0.927175;
  0.856601 0.450938 -0.153235 -0.198518 0.847658 -0.374718 0.765335;
  -0.013654 -0.201072 0.505480 -0.838972 0.849599 -0.373839 0.768409;
];

%% Converte poses para matrizes homogêneas
N = size(robot_poses,1);

Tbg = zeros(4,4,N);   % base_robo -> gripper
Tcg = zeros(4,4,N);   % base_cage -> gripper

for i = 1:N
    Tbg(:,:,i) = robotPose2T(robot_poses(i,:));
    Tcg(:,:,i) = cameraPose2T(cage_poses(i,:));
end

%% ==========================================================
% Resolve diretamente:
% Tbg(:,:,i) * X_gc = X_bc * Tcg(:,:,i)
%
% Parametros:
% p = [rx_gc ry_gc rz_gc tx_gc ty_gc tz_gc rx_bc ry_bc rz_bc tx_bc ty_bc tz_bc]
%
% rotacao parametrizada por vetor de Rodrigues
%% ==========================================================
p0 = 0.1 * randn(12,1);

options = optimset('Display','iter', ...
                   'MaxFunEvals', 200000, ...
                   'MaxIter', 200000, ...
                   'TolX', 1e-10, ...
                   'TolFun', 1e-10);

p = fminsearch(@(pp) calibrationCost(pp, Tbg, Tcg), p0, options);

J_final = calibrationCost(p, Tbg, Tcg);
disp(J_final);

J0 = calibrationCost(p0, Tbg, Tcg);
disp(J0);

X_gc = vec2T(p(1:6));
X_bc = vec2T(p(7:12));

[pos_errors, rot_errors, score] = computeErrors(Tbg, Tcg, X_gc, X_bc);

fprintf('\n=== ERROS ANTES DE REMOVER OUTLIERS ===\n');
for i = 1:N
    fprintf('i=%02d | pos=%.6f m | rot=%.3f deg | score=%.3f\n', ...
        i, pos_errors(i), rot_errors(i), score(i));
end

fprintf('\nErro medio posicao: %.6f m\n', mean(pos_errors));
fprintf('Erro maximo posicao: %.6f m\n', max(pos_errors));
fprintf('Erro medio rotacao: %.6f graus\n', mean(rot_errors));
fprintf('Erro maximo rotacao: %.6f graus\n', max(rot_errors));

% ========= DETECCAO DE INLIERS =========
idx_keep = findInliersMAD(score, 3.5);
idx_out  = setdiff(1:N, idx_keep);

fprintf('\nInliers mantidos:\n');
disp(idx_keep);

fprintf('Outliers removidos:\n');
disp(idx_out);

% ========= RECALIBRACAO SEM OUTLIERS =========
Tbg_in = Tbg(:,:,idx_keep);
Tcg_in = Tcg(:,:,idx_keep);

p_refined = fminsearch(@(pp) calibrationCost(pp, Tbg_in, Tcg_in), p, options);

X_gc_refined = vec2T(p_refined(1:6));
X_bc_refined = vec2T(p_refined(7:12));

[pos_errors2, rot_errors2, score2] = computeErrors(Tbg_in, Tcg_in, X_gc_refined, X_bc_refined);

fprintf('\n=== ERROS APOS REMOVER OUTLIERS ===\n');
fprintf('Erro medio posicao: %.6f m\n', mean(pos_errors2));
fprintf('Erro maximo posicao: %.6f m\n', max(pos_errors2));
fprintf('Erro medio rotacao: %.6f graus\n', mean(rot_errors2));
fprintf('Erro maximo rotacao: %.6f graus\n', max(rot_errors2));

disp('X_gc refinado:');
disp(X_gc_refined);

disp('X_bc refinado:');
disp(X_bc_refined);

%% Validacao
pos_errors = zeros(N,1);
rot_errors = zeros(N,1);

for i = 1:N
    % Tbg * X_gc = X_bc * Tcg
    % => Tbg_est = X_bc * Tcg / X_gc
    T_est  = X_bc * Tcg(:,:,i) / X_gc;
    T_real = Tbg(:,:,i);

    dp = T_real(1:3,4) - T_est(1:3,4);
    pos_errors(i) = norm(dp);

    R_err = T_real(1:3,1:3) * T_est(1:3,1:3)';
    ang = acos(max(min((trace(R_err)-1)/2,1),-1));
    rot_errors(i) = rad2deg(ang);
end

fprintf('\nErro medio posicao: %.6f m\n', mean(pos_errors));
fprintf('Erro maximo posicao: %.6f m\n', max(pos_errors));
fprintf('Erro medio rotacao: %.6f graus\n', mean(rot_errors));
fprintf('Erro maximo rotacao: %.6f graus\n', max(rot_errors));

%% =========================
% FUNCOES AUXILIARES
%% =========================

function J = calibrationCost(p, Tbg, Tcg)
    X_gc = vec2T(p(1:6));
    X_bc = vec2T(p(7:12));

    N = size(Tbg,3);
    J = 0;

    for i = 1:N
        E = Tbg(:,:,i) * X_gc - X_bc * Tcg(:,:,i);

        % peso um pouco maior na translacao, mas sem exagerar
        Er = E(1:3,1:3);
        Et = E(1:3,4);

        J = J + norm(Er,'fro')^2 + 10*norm(Et)^2;
    end
end

function T = vec2T(v)
    r = v(1:3);
    t = v(4:6);

    R = rodriguesToRotm(r);

    T = eye(4);
    T(1:3,1:3) = R;
    T(1:3,4) = t;
end

function R = rodriguesToRotm(r)
    theta = norm(r);

    if theta < 1e-12
        R = eye(3);
        return;
    end

    k = r / theta;
    K = skew3(k);

    R = eye(3) + sin(theta)*K + (1-cos(theta))*(K*K);
end

function S = skew3(v)
    S = [   0   -v(3)  v(2);
          v(3)   0    -v(1);
         -v(2) v(1)    0  ];
end

function T = robotPose2T(pose)
    % pose = [tz ty tx x y z]

    tz = pose(1);
    ty = pose(2);
    tx = pose(3);
    x  = pose(4);
    y  = pose(5);
    z  = pose(6);

    Rz = [cos(tz) -sin(tz) 0;
          sin(tz)  cos(tz) 0;
             0        0    1];

    Ry = [ cos(ty) 0 sin(ty);
              0    1   0;
          -sin(ty) 0 cos(ty)];

    Rx = [1 0 0;
          0 cos(tx) -sin(tx);
          0 sin(tx)  cos(tx)];

    R = Rz * Ry * Rx;

    T = [R [x; y; z];
         0 0 0 1];
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

    R = [1-2*(qy^2+qz^2),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw);
         2*(qx*qy+qz*qw), 1-2*(qx^2+qz^2),   2*(qy*qz-qx*qw);
         2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx^2+qy^2)];

    T = [R [x; y; z];
         0 0 0 1];
end







% ------------------------------------------------------------------------------------------

function pose_robot = cageToRobotPose(pose_cage, X_bc, X_gc)
    % pose_cage = [qx qy qz qw x y z]
    % pose_robot = [tz ty tx x y z]

    % 1) pose da cage -> matriz homogênea
    Tcg = cameraPose2T(pose_cage);

    % 2) converte do frame base_cage para base_robo
    %    Tbg = X_bc * Tcg * inv(X_gc)
    Tbg = X_bc * Tcg / X_gc;

    % 3) matriz homogênea -> formato do robo [tz ty tx x y z]
    pose_robot = T2robotPose(Tbg);
end

function pose = T2robotPose(T)
    % Converte matriz homogênea para:
    % pose = [tz ty tx x y z]
    % assumindo R = Rz(tz) * Ry(ty) * Rx(tx)

    R = T(1:3,1:3);
    x = T(1,4);
    y = T(2,4);
    z = T(3,4);

    % Extracao ZYX
    % R(3,1) = -sin(ty)
    if abs(R(3,1)) < 1 - 1e-10
        ty = -asin(R(3,1));
        tz = atan2(R(2,1), R(1,1));
        tx = atan2(R(3,2), R(3,3));
    else
        % caso singular (gimbal lock)
        ty = -asin(max(min(R(3,1),1),-1));

        if R(3,1) <= -1 + 1e-10
            % ty = +pi/2
            ty = pi/2;
            tz = atan2(-R(1,2), R(2,2));
            tx = 0;
        else
            % ty = -pi/2
            ty = -pi/2;
            tz = atan2(-R(1,2), R(2,2));
            tx = 0;
        end
    end

    pose = [tz ty tx x y z];
end

%% ============================================
% EXEMPLO DE USO
% pose_cage = [rot_x rot_y rot_z rot_w X Y Z]
%% ============================================

N = size(cage_poses,1);

for i = 1:N
    pose = cage_poses(i,:);

    pose_robot = cageToRobotPose(pose, X_bc, X_gc);

    fprintf('Pose %d convertida:\n', i);
    disp(pose_robot);
end


function [pos_errors, rot_errors, score] = computeErrors(Tbg, Tcg, X_gc, X_bc)
    N = size(Tbg,3);
    pos_errors = zeros(N,1);
    rot_errors = zeros(N,1);

    for i = 1:N
        T_est  = X_bc * Tcg(:,:,i) / X_gc;
        T_real = Tbg(:,:,i);

        dp = T_real(1:3,4) - T_est(1:3,4);
        pos_errors(i) = norm(dp);

        R_err = T_real(1:3,1:3) * T_est(1:3,1:3)';
        ang = acos(max(min((trace(R_err)-1)/2,1),-1));
        rot_errors(i) = rad2deg(ang);
    end

    % score combinado: 1 cm ~ 1 grau
    score = pos_errors/0.01 + rot_errors/1.0;
end

function idx_keep = findInliersMAD(score, thr)
    med_s = median(score);
    mad_s = median(abs(score - med_s));

    if mad_s < 1e-12
        idx_keep = 1:length(score);
        return;
    end

    z_rob = 0.6745 * (score - med_s) / mad_s;
    idx_keep = find(abs(z_rob) <= thr);
end