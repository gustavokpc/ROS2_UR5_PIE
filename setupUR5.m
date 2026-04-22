% setupUR5 initializes the UR5 robot model and ROS2 connection.
% This function performs all heavy setup operations (robot model loading,
% ROS2 device connection, kinematics configuration) once and stores them
% in a context structure (ctx). This avoids reinitializing everything
% inside control loops, improving performance when sending commands
% to move the robot.
%
% Output:
%   ctx - Structure containing robot model, ROS2 node, IK solver,
%         and configuration parameters

function ctx = setupUR5()
    % ctx = robot context (heavy setup)

    % Load the UR5 robot model (RigidBodyTree)
    ctx.ur5 = loadrobot('universalUR5');

    % Credentials for ROS2 device connection
    username = 'student';
    password = 'student';

    % ROS2 installation and workspace paths on the remote device
    ROS2Folder = '/opt/ros/humble';
    ROS2Workspace = '/opt/ros/humble';

    % IP address of the ROS2-enabled robot controller
    ROS2DeviceAddress = '147.250.35.73';

    % Create a ROS2 device object and configure its environment
    device = ros2device(ROS2DeviceAddress, username, password);
    device.ROS2Folder = ROS2Folder;
    device.ROS2Workspace = ROS2Workspace;

    % Create a new rigid body representing the tool center point (TCP)
    tcp = rigidBody('tcp');
    jnt = rigidBodyJoint('tcp_fixed','fixed');
    
    % Define the transformation offset from tool0 to the gripper tip
    % (adjust this value to match the real hardware)
    offset = trvec2tform([0 0 0.21]);
    setFixedTransform(jnt, offset);
    
    % Attach the TCP body to the robot model
    tcp.Joint = jnt;
    addBody(ctx.ur5, tcp, 'tool0');

    % Create a ROS2 node interface for controlling the UR5
    ctx.ur = urROS2Node('RigidBodyTree', ctx.ur5);

    % Create the inverse kinematics solver ONCE (for efficiency)
    ctx.ik = inverseKinematics("RigidBodyTree", ctx.ur5);

    % Store the end-effector (TCP) name
    ctx.eeName = "tcp";

    % Define IK weights:
    % Higher values prioritize position/orientation accuracy
    % Lower orientation weights help smooth the motion
    ctx.weights = [1 1 1 0.2 0.2 0.2];  % tuned for smoother movement
end