function setToolOutput(ioClient, state)
    req = ros2message(ioClient);
    req.fun = int8(1);      % Digital output
    req.pin = int8(16);     % Tool Output 0
    req.state = single(state);   % float32
    call(ioClient, req);
end