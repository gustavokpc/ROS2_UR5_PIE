ros2genmsg("/opt/ros/humble/share")

% 1) Conecta no ROS2
node = ros2node("/matlab_node");

% 2) Cria client do serviço SetIO
cli = ros2svcclient(node, "/io_and_status_controller/set_io", "ur_msgs/SetIO");

% 3) Monta request
req = ros2message(cli);
req.fun   = int8(1);   % 1 = set digital output
req.pin   = int8(0);   % pino 0 (ajuste: 0 ou 1 para tool digital out)
req.state = 0.0;       % 1.0 liga, 0.0 desliga

% 4) Chama o serviço
resp = call(cli, req, "Timeout", 2);
disp(resp);