% export ROS_IP=147.250.35.73
% export ROS_MASTER_URI=http://147.250.35.39

folder = fullfile(getenv("HOME"), "pie_ws", "src");
ros2genmsg(folder);
savepath;
rehash toolboxcache;

% --- ENV ---
setenv('ROS_MASTER_URI','http://147.250.35.39')
setenv('ROS_IP','147.250.35.73');
setenv('ROS_DOMAIN_ID', '0');
setenv('ROS_LOCALHOST_ONLY','0');

ctx = setupUR5();

% --- ROS2 node ---
rosNode = ros2node("/matlab_ur5_node");

% --- Cliente para IO (Tool Output) ---
ioClient = ros2svcclient(rosNode, ...
    "/io_and_status_controller/set_io", ...
    "ur_msgs/SetIO");


waitForServer(ioClient);

% --- Estado compartilhado ---
shared.lastP = [];
shared.lastUpdate = -inf;   % último instante que aceitou um ponto
shared.throttle = 2.0;      % segundos
shared.locked = false;      % evita mandar outro comando enquanto executa

tic; % relógio pra throttle

% Subscriber: só armazena (com throttle de 3s)

% SUB CAGE
sub = ros2subscriber(rosNode, "/erreur_bras", "geometry_msgs/Transform", ...
    @(msg) storePointThrottled(msg));

% %SUB CAMERA
% sub = ros2subscriber(rosNode, "/object_positions", "geometry_msgs/Point", ...
%     @(msg) storePointThrottled(msg));

pub = ros2publisher(rosNode, ...
    "/robot_pose", ...
    "geometry_msgs/Transform");

jointHome = [0   -1.5708   0   -1.5709    0   0]; %sendJointConfiguration(ctx.ur,jointHome,'EndTime',5);

disp("Subscriber ready. Aceitando ponto novo a cada 3s.");
disp("Aperte Enter para enviar o robô ao último ponto aceito.");

figure;
set(gcf, 'KeyPressFcn', @(src,event) keyCallback(event, ctx));

while true

    % --- publish sempre ---
    publishPose(ctx, pub);

    pause(0.05); % ~20 Hz

end

% ---------------- CALLBACK SUBSCRIBER----------------
function storePointThrottled(msg)
    shared = evalin("base","shared");
    t = toc;

    if (t - shared.lastUpdate) < shared.throttle
        return; % ignora até completar 3s
    end

    x = msg.translation.x;
    y = msg.translation.y;
    z = msg.translation.z;
    
    % x = msg.x;
    % y = msg.y;
    % z = msg.z;

    shared.lastP = [x y z];
    shared.lastUpdate = t;

    assignin("base","shared",shared);

    disp("Ponto aceito (throttle 3s):");
    disp(shared.lastP);
end

% % ---------------- "CALLBACK" PUBLISHER----------------
function publishPose(ctx, pub)

    msg = ros2message(pub);

    p = getCartesianPose(ctx.ur);

    if numel(p) < 6
        warning("Pose inválida (menos de 6 elementos)");
        return;
    end

    % posição arredondada
    msg.translation.x = round(p(4), 4);
    msg.translation.y = round(p(5), 4);
    msg.translation.z = round(p(6), 4);

    quat = eul2quat([p(1) p(2) p(3)]);
    msg.rotation.w = round(quat(1), 4);
    msg.rotation.x = round(quat(2), 4);
    msg.rotation.y = round(quat(3), 4);
    msg.rotation.z = round(quat(4), 4);

    send(pub, msg);
end

function keyCallback(event, ctx)

    if strcmp(event.Key, 'return')

        disp("ENTER pressionado!");

        shared = evalin("base","shared");

        if isempty(shared.lastP)
            disp("Ainda não tenho ponto.");
            return;
        end

        if shared.locked
            disp("Robô ocupado.");
            return;
        end

        shared.locked = true;
        assignin("base","shared",shared);

        p = shared.lastP;
        disp("Enviando robô para:");
        disp(p);

        [qSol, info] = computeIK_UR5(ctx, p);

        if isfield(info,"ExitFlag") && info.ExitFlag > 0
            sendJointConfigurationAndWait(ctx.ur, qSol, 'EndTime',10);
        else
            disp("IK falhou.");
        end

        shared = evalin("base","shared");
        shared.locked = false;
        assignin("base","shared",shared);
    end

end