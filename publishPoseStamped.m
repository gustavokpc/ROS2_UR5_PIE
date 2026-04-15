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

    % --- ROTATION (ZYX -> quaternion) ---
    % quat = eul2quat([p(1) p(2) p(3)]); % ZYX padrão MATLAB

    msg.transform.rotation.w = 0.0;
    msg.transform.rotation.x = p(3);
    msg.transform.rotation.y = p(2);
    msg.transform.rotation.z = p(1);

    % --- SEND ---
    send(pub, msg);

end