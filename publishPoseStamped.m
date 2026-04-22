% publishPoseStamped creates and publishes the current robot pose as a ROS2 message.
%
% The function:
% - Retrieves the current Cartesian pose of the robot
% - Fills a ROS2 TransformStamped message
% - Publishes position (x, y, z)
% - Publishes orientation either:
%       (1) as a quaternion (recommended for ROS), or
%       (2) as raw Euler angles (custom/debug usage)
%
% Inputs:
%   ctx     - Robot context (contains UR interface)
%   pub     - ROS2 publisher (TransformStamped)
%   rosNode - ROS2 node (used for timestamp)

function publishPoseStamped(ctx, pub, rosNode)

    % Create a ROS2 message from the publisher type
    msg = ros2message(pub);

    % Get current robot Cartesian pose
    % Expected format: [tz ty tx x y z]
    p = getCartesianPose(ctx.ur);

    % Validate pose size
    if numel(p) < 6
        warning("Invalid pose");
        return;
    end

    % --- HEADER ---
    % Set timestamp using ROS2 node clock
    msg.header.stamp = ros2time(rosNode,"now");

    % --- TRANSLATION ---
    % Position in meters (rounded for cleaner output)
    msg.transform.translation.x = round(p(4), 4);
    msg.transform.translation.y = round(p(5), 4);
    msg.transform.translation.z = round(p(6), 4);
    

    % --- ROTATION OPTIONS ---

    % Option 1 (RECOMMENDED): Convert ZYX Euler angles to quaternion
    % MATLAB uses ZYX order: [yaw pitch roll] = [Z Y X]
    % Uncomment to use quaternion representation
    %
    % quat = eul2quat([p(1) p(2) p(3)]);
    % msg.transform.rotation.w = quat(1);
    % msg.transform.rotation.x = quat(2);
    % msg.transform.rotation.y = quat(3);
    % msg.transform.rotation.z = quat(4);


    % Option 2: Publish raw Euler angles (non-standard for ROS, for debugging)
    % Stored as:
    % x = roll (tx), y = pitch (ty), z = yaw (tz)
    msg.transform.rotation.w = 0.0;
    msg.transform.rotation.x = p(3);
    msg.transform.rotation.y = p(2);
    msg.transform.rotation.z = p(1);

    % --- SEND MESSAGE ---
    % Publish the message to the ROS2 topic
    send(pub, msg);

end