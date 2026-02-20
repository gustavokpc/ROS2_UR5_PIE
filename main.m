% export ROS_IP=147.250.35.73
% export ROS_MASTER_URI=http://147.250.35.39

setenv('ROS_MASTER_URI','http://147.250.35.39')
setenv('ROS_IP','147.250.35.73');

setenv('ROS_DOMAIN_ID', '0');

ctx = setupUR5();

%% --- Créer node MATLAB ROS2 pour subscriber
rosNode = ros2node("/matlab_ur5_node");

%% --- S'abonner au topic /object_pose
sub = ros2subscriber(rosNode, "/transformation", "geometry_msgs/Transform", @(msg) moveUR5Callback(msg, ctx));

disp("Iniciando setup...");

disp("Subscriber ready. Waiting for object positions...");

%% ------------------- CALLBACK -------------------
function moveUR5Callback(msg, ctx)
    x = msg.translation.x;
    y = msg.translation.y;
    z = msg.translation.z;

    disp("Nouvelle position cible reçue :");
    disp([x y z]);
    
    p = [x, y, z];  % Create position vector from received message

    [qSol, info] = computeIK_UR5(ctx, p);

    %sendJointConfigurationAndWait(ctx.ur,qSol,'EndTime',7);
    
    pause(3)

end
