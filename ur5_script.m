clear;
ur5 = loadrobot('universalUR5');

username = 'student';
password = 'student';
ROS2Folder = '/opt/ros/humble';
ROS2Workspace = '/opt/ros/humble'; % In case of binary installation, ROS 2 Work-space is same as ROS 2 Folder

ROS2DeviceAddress = '147.250.35.73';

robotAddress = '147.250.35.40'; 

device = ros2device(ROS2DeviceAddress,username,password);
device.ROS2Folder = ROS2Folder;device.ROS2Workspace = ROS2Workspace;

tcp = rigidBody('tcp');
jnt = rigidBodyJoint('tcp_fixed','fixed');

% offset tool0 -> ponta da garra
offset = trvec2tform([0 0 0.06]);  % <-- troque pelo valor real
setFixedTransform(jnt, offset);

tcp.Joint = jnt;
addBody(ur5, tcp, 'tool0');

ur = urROS2Node('RigidBodyTree',ur5);

home = [0   0   -1.5708   0    0.2515   1.0011];

jointHome = [0   -1.5708   0   -1.5709    0   0];

desPos = [-0.1160   -0.3798   -1.2395   -0.0388    0.1859    0.9962];

jointDesPos = [0.6318   -1.2926   -0.3766   -1.9710   -0.6836    0.0002];

%sendJointConfiguration(ur,jointHome,'EndTime',5);

%jointAngles = getJointConfiguration(ur,10)

ik = inverseKinematics("RigidBodyTree", ur5);

% --- If you have a connected urROS2Node 'ur', read current Cartesian pose ---
p = getCartesianPose(ur);   % [thetaZ thetaY thetaX x y z]

%p = [0   -1.5708  0 0.2300    0.0552    0.97360];

thetaZ = p(1); thetaY = p(2); thetaX = p(3);
x = p(4);      y = p(5);      z = p(6);

% --- Build rotation matrix using Z-Y-X order (as your doc states) ---
Rz = axang2rotm([0 0 1 thetaZ]);
Ry = axang2rotm([0 1 0 thetaY]);
Rx = axang2rotm([1 0 0 thetaX]);
R  = Rz * Ry * Rx;

% --- Homogeneous transform (base -> tool0/TCP frame origin) ---
T = eye(4);
T(1:3,1:3) = R;
T(1:3,4)   = [x; y; z];

% --- Initial guess: best is current joints from the real robot ---
q0 = getJointConfiguration(ur);   % 1x6 (recommended)

% --- Solve IK ---
weights = [1 1 1 1 1 1];          % [xyz rpy] relative weights

[qSol_tool0, info] = ik("tool0", T, weights, q0);

[qSol_tcp, info] = ik("tcp", T, weights, q0);

qSol_tool0 = wrapToPi(qSol_tool0);

qSol_tcp = wrapToPi(qSol_tcp);

% --- Results ---
disp("qSol (joint angles, rad):");
disp(qSol_tcp);

disp("Cartesian values tool0")
disp(p);

