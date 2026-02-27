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

ctx = setupUR5();

% --- ROS2 node ---
rosNode = ros2node("/matlab_ur5_node");

% --- Cliente para IO (Tool Output) ---
ioClient = ros2svcclient(rosNode, ...
    "/io_and_status_controller/set_io", ...
    "ur_msgs/SetIO");

% --- Estado compartilhado ---
shared.lastP = [];
shared.lastUpdate = -inf;   % último instante que aceitou um ponto
shared.throttle = 3.0;      % segundos
shared.locked = false;      % evita mandar outro comando enquanto executa

tic; % relógio pra throttle

% Subscriber: só armazena (com throttle de 3s)
sub = ros2subscriber(rosNode, "/transformation", "geometry_msgs/Transform", ...
    @(msg) storePointThrottled(msg));

disp("Subscriber ready. Aceitando ponto novo a cada 3s.");
disp("Aperte Enter para enviar o robô ao último ponto aceito.");

% --- LOOP: manda só quando apertar Enter ---
while true
    input("Enter = enviar para o último ponto | Ctrl+C = sair ", "s");

    shared = evalin("base","shared");

    if isempty(shared.lastP)
        disp("Ainda não tenho nenhum ponto (ou ainda não passou 3s).");
        continue;
    end

    if shared.locked
        disp("Robô ainda executando o último comando.");
        continue;
    end

    shared.locked = true;
    assignin("base","shared",shared);

    p = shared.lastP;
    disp("Enviando robô para:");
    disp(p);

    [qSol, info] = computeIK_UR5(ctx, p);
    if isfield(info,"ExitFlag") && info.ExitFlag <= 0
        disp("IK falhou. Não vou mandar comando.");
    else

        % DESCOMENTE para mover de verdade:
        sendJointConfigurationAndWait(ctx.ur, qSol, 'EndTime', 7);
        pause(3); % teste
        setToolOutput(ioClient, 1.0);
        pause(2);
        setToolOutput(ioClient, 0.0);
    end

    shared = evalin("base","shared");
    shared.locked = false;
    assignin("base","shared",shared);
end

% ---------------- CALLBACK ----------------
function storePointThrottled(msg)
    shared = evalin("base","shared");
    t = toc;

    if (t - shared.lastUpdate) < shared.throttle
        return; % ignora até completar 3s
    end

    x = msg.translation.x;
    y = msg.translation.y;
    z = msg.translation.z;

    shared.lastP = [x y z];
    shared.lastUpdate = t;

    assignin("base","shared",shared);

    disp("Ponto aceito (throttle 3s):");
    disp(shared.lastP);
end