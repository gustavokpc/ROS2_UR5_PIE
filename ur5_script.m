clear;
ur5 = loadrobot('universalUR5');

username = 'student';
password = 'student';

ROS2Folder = '/opt/ros/humble';
ROS2Workspace = '/opt/ros/humble';

ROS2DeviceAddress = '147.250.35.73';
robotAddress = '147.250.35.40'; 

device = ros2device(ROS2DeviceAddress,username,password);
device.ROS2Folder = ROS2Folder;
device.ROS2Workspace = ROS2Workspace;

%% --- TCP ---
tcp = rigidBody('tcp');
jnt = rigidBodyJoint('tcp_fixed','fixed');

offset = trvec2tform([0 0 0.21]); % ajuste real
setFixedTransform(jnt, offset);

tcp.Joint = jnt;
addBody(ur5, tcp, 'tool0');

%% --- ROS2 node do robô ---
ur = urROS2Node('RigidBodyTree',ur5);

%% --- Criar contexto para a função IK ---
ctx.ur = ur;
ctx.ur5 = ur5;
ctx.eeName = "tcp";
ctx.weights = [1 1 1 1 1 1];
ctx.ik = inverseKinematics("RigidBodyTree", ur5);

jointHome = [0   -1.5708   0   -1.5709    0   0]; %sendJointConfiguration(ctx.ur,jointHome,'EndTime',5);

%% --- Ler pose atual ---
p_cart = getCartesianPose(ur); % [thetaZ thetaY thetaX x y z]

disp("Pose cartesiana atual:");
disp(p_cart);

% %% --- Definir ponto alvo ---
% p_target = [-0.7687    0.05    0.75]; % [x y z]

%% --- Calcular IK usando sua função ---
[qSol, info] = computeIK_UR5(ctx, p_cart);

disp("Solução IK (rad):");
disp(qSol);

%% --- Enviar para o robô ---
% sendJointConfiguration(ur, qSol, 'EndTime',5);