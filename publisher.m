% --- ROS ENVIRONMENT SETUP (same as other script) ---
setenv('ROS_MASTER_URI','http://147.250.35.39')
setenv('ROS_IP','147.250.35.73');
setenv('ROS_DOMAIN_ID', '0');
setenv('ROS_LOCALHOST_ONLY','0');

% Initialize robot context (heavy setup done once)
ctx = setupUR5();

% --- ROS2 NODE ---
% Create a ROS2 node for publishing robot pose
rosNode = ros2node("/pose_publisher");

% --- PUBLISHER (TransformStamped) ---
% Topic: /robot_pose
% Message type: geometry_msgs/TransformStamped
pub = ros2publisher(rosNode, ...
    "/robot_pose", ...
    "geometry_msgs/TransformStamped");

disp("Publishing robot pose (TransformStamped)...");

% --- MAIN LOOP ---
% Continuously publish robot pose
while true

    % Publish current robot pose
    publishPoseStamped(ctx, pub, rosNode);
    
    % Loop rate ~2 Hz
    pause(0.5);

end