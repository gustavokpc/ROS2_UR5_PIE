function publishPoseStamped(ctx, pub, rosNode)

    msg = ros2message(pub);

    p = getCartesianPose(ctx.ur);

    if numel(p) < 6
        warning("Pose inválida");
        return;
    end

    % --- HEADER ---
    msg.header.stamp = ros2time(rosNode,"now");

    % --- TRANSLATION ---
    msg.transform.translation.x = round(p(4), 4);
    msg.transform.translation.y = round(p(5), 4);
    msg.transform.translation.z = round(p(6), 4);
    
    % disp ([p(1) p(2) p(3));
    % rpy = rotvec2rpy([p(3),p(2),p(1)]);
    % 
    % disp(rpy);

    % --- ROTATION (ZYX -> quaternion) ---
    % quat = eul2quat([rpy(3) rpy(2) rpy(1)]); % ZYX padrão MATLAB
    % 
    % msg.transform.rotation.w = quat(1);
    % msg.transform.rotation.x = quat(2);
    % msg.transform.rotation.y = quat(3);
    % msg.transform.rotation.z = quat(4);
    
    msg.transform.rotation.w = 0.0;
    msg.transform.rotation.x = p(3);
    msg.transform.rotation.y = p(2);
    msg.transform.rotation.z = p(1);

    % --- SEND ---
    send(pub, msg);

end