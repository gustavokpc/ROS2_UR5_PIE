% setToolOutput controls the digital output of a UR5 robot tool via ROS2.
% It is typically used to open or close a gripper by setting the state
% of a tool digital pin (e.g., Tool Output 0).
%
% Inputs:
%   ioClient - ROS2 service client used to send the command
%   state    - Desired output state (1.0 = ON/close, 0.0 = OFF/open)

function setToolOutput(ioClient, state)
    % Create a ROS2 service request message using the provided client
    req = ros2message(ioClient);
    
    % Specify the function type:
    % 1 corresponds to setting a digital output
    req.fun = int8(1);      
    
    % Select the pin to control:
    % Pin 16 corresponds to Tool Digital Output 0 on the UR5 robot
    req.pin = int8(16);     
    
    % Set the desired state of the output:
    % Typically 1.0 = ON (close gripper), 0.0 = OFF (open gripper)
    % Must be sent as a float32 value
    req.state = single(state);   
    
    % Send the request to the robot via the ROS2 service
    call(ioClient, req);
end