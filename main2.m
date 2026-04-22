% export ROS_IP=147.250.35.73
% export ROS_MASTER_URI=http://147.250.35.39

% Generate ROS2 message interfaces from workspace
folder = fullfile(getenv("HOME"), "pie_ws", "src");
ros2genmsg(folder);
savepath;
rehash toolboxcache;

% --- ROS ENVIRONMENT ---
setenv('ROS_MASTER_URI','http://147.250.35.39')
setenv('ROS_IP','147.250.35.73');
setenv('ROS_DOMAIN_ID', '0');
setenv('ROS_LOCALHOST_ONLY','0');

% Initialize robot context (robot model, IK, ROS2 interface, etc.)
ctx = setupUR5();

% --- ROS2 NODE ---
rosNode = ros2node("/matlab_ur5_node");

% --- IO SERVICE CLIENT (gripper control) ---
ioClient = ros2svcclient(rosNode, ...
    "/io_and_status_controller/set_io", ...
    "ur_msgs/SetIO");

% Wait until ROS service becomes available
waitForServer(ioClient);

% --- SHARED STATE STRUCTURE ---
% Used to communicate between subscriber callback and main loop
shared.lastP = [];        % last accepted point [x y z]
shared.lastUpdate = -inf; % timestamp of last accepted message
shared.throttle = 2.0;    % minimum time between accepted messages (seconds)
shared.locked = false;    % prevents sending multiple robot commands simultaneously

tic; % timer used for throttling logic

% --- CAGE SUBSCRIBER ---
sub = ros2subscriber(rosNode, "/instruction_cage", ...
    "geometry_msgs/Transform", ...
    @(msg) storePointThrottled(ctx, msg));

% --- CAMERA SUBSCRIBER ---
% sub = ros2subscriber(rosNode, "/object_positions", ...
%     "geometry_msgs/Point", ...
%     @(msg) storePointThrottled(msg));

% Home joint configuration (optional reference position)
jointHome = [0 -1.5708 0 -1.5709 0 0];  % sendJointConfiguration(ctx.ur, jointHome, 'EndTime', 5);

disp("Subscriber ready. Accepting new point every 3 seconds.");
disp("Press Enter to send robot to last accepted point.");

% =========================================================
% MAIN CONTROL LOOP
% =========================================================
while true

    % Wait for user trigger
    input("Enter = send robot to last point | Ctrl+C = exit ", "s");

    % Retrieve shared state from base workspace
    shared = evalin("base","shared");

    % Check if any valid point exists
    if isempty(shared.lastP)
        disp("No valid point received yet (or still in throttle delay).");
        continue;
    end

    % Check if robot is currently executing a command
    if shared.locked
        disp("Robot is still executing previous command.");
        continue;
    end

    % Lock execution to avoid concurrent commands
    shared.locked = true;
    assignin("base","shared",shared);

    % Target point [x y z]
    p_goal = shared.lastP;

    % Compute radial direction in XY plane (from base to point)
    dir = p_goal(1:2) / norm(p_goal(1:2));

    % Extend direction to 3D
    dir3 = [dir 0];

    % Offset point 10 cm backwards from target
    d = 0.10;
    p_intermediate = p_goal - d * dir3;

    % Keep original height
    p_intermediate(3) = p_goal(3);

    disp("Sending robot to intermediate point:");
    disp(p_intermediate);

    % Compute inverse kinematics for intermediate position
    [qSol, info] = computeIK_UR5(ctx, p_intermediate);

    if isfield(info,"ExitFlag") && info.ExitFlag <= 0
        disp("IK failed. Command not sent.");
    else
        % Move to intermediate position first
        sendJointConfigurationAndWait(ctx.ur, qSol, 'EndTime', 10);
        pause(2);

        % Compute IK for final target position
        [qSol, info] = computeIK_UR5(ctx, p_goal);
        sendJointConfigurationAndWait(ctx.ur, qSol, 'EndTime', 5);
        pause(1);

        % Optional gripper control (currently disabled)
        % setToolOutput(ioClient, 1.0); % close gripper
        % sendJointConfiguration(ctx.ur, jointHome, 'EndTime', 5); % go home
        % pause(1);
        % setToolOutput(ioClient, 0.0); % open gripper
    end

    % Unlock robot after execution
    shared = evalin("base","shared");
    shared.locked = false;
    assignin("base","shared",shared);

end

% =========================================================
% SUBSCRIBER CALLBACK FUNCTION
% =========================================================
function storePointThrottled(ctx, msg)

    % Access shared state
    shared = evalin("base","shared");

    % Current time (for throttle control)
    t = toc;

    % Ignore messages if throttle time has not passed
    if (t - shared.lastUpdate) < shared.throttle
        return;
    end

    % =====================================================
    % CAGE INPUT FORMAT
    % =====================================================
    x = msg.translation.x;
    y = msg.translation.y;
    z = msg.translation.z;

    % =====================================================
    % CAMERA INPUT FORMAT
    % =====================================================
    % x = msg.x;
    % y = msg.y;
    % z = msg.z;

    % Store latest valid point
    shared.lastP = [x y z];

    % Update timestamp
    shared.lastUpdate = t;

    % Save back to base workspace
    assignin("base","shared",shared);

    disp("Point accepted (3s throttle):");
    disp(shared.lastP);

    % Optional: preview IK for debugging
    disp("Best solution found:")
    [qSol, info] = computeIK_UR5(ctx, shared.lastP);
    disp(qSol);

end