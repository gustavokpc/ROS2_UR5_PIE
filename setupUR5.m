function ctx = setupUR5()
    % ctx = contexto do robô (setup pesado)

    ctx.ur5 = loadrobot('universalUR5');

    username = 'student';
    password = 'student';

    ROS2Folder = '/opt/ros/humble';
    ROS2Workspace = '/opt/ros/humble';

    ROS2DeviceAddress = '147.250.35.73';

    device = ros2device(ROS2DeviceAddress, username, password);
    device.ROS2Folder = ROS2Folder;
    device.ROS2Workspace = ROS2Workspace;

    tcp = rigidBody('tcp');
    jnt = rigidBodyJoint('tcp_fixed','fixed');
    
    % offset tool0 -> ponta da garra
    offset = trvec2tform([0 0 0.21]);  % <-- troque pelo valor real
    setFixedTransform(jnt, offset);
    
    tcp.Joint = jnt;
    addBody(ctx.ur5, tcp, 'tool0');

    ctx.ur = urROS2Node('RigidBodyTree', ctx.ur5);

    % IK criado UMA vez
    ctx.ik = inverseKinematics("RigidBodyTree", ctx.ur5);

    % Parâmetros fixos
    ctx.eeName = "tcp";
    % ctx.weights = [1 1 1 1 1 1];

    ctx.weights = [1 1 1 0.2 0.2 0.2];  % TESTE PARA SUAVIZAR MOVIMENTO
end