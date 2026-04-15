% --- ENV (igual ao outro script) ---
setenv('ROS_MASTER_URI','http://147.250.35.39')
setenv('ROS_IP','147.250.35.73');
setenv('ROS_DOMAIN_ID', '0');
setenv('ROS_LOCALHOST_ONLY','0');

ctx = setupUR5();

% --- ROS2 node ---
rosNode = ros2node("/pose_publisher");

% --- Publisher (STAMPED) ---
pub = ros2publisher(rosNode, ...
    "/robot_pose", ...
    "geometry_msgs/TransformStamped");

disp("Publishing pose (TransformStamped)...");

while true

    publishPoseStamped(ctx, pub, rosNode);

    pause(0.5); % ~2 Hz

end