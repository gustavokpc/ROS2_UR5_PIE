% computeIK_UR5 computes the inverse kinematics (IK) solution for the UR5 robot.
% Given a desired Cartesian position [x y z], it builds a target pose with a
% predefined orientation and solves for the joint configuration that reaches it.
%
% The function:
% - Computes a yaw angle so the end-effector faces the target
% - Builds a homogeneous transformation matrix (position + orientation)
% - Uses the IK solver stored in the context (ctx)
% - Starts from the current robot configuration (or home if unavailable)
% - Improves the solution by trying multiple random initial guesses
% - Selects the solution closest to the initial configuration (smooth motion)
%
% Inputs:
%   ctx - Robot context created in setup (contains robot model, IK solver, etc.)
%   p   - Target position [x y z] in Cartesian space
%
% Outputs:
%   qSol - Best joint configuration found (in radians)
%   info - Information returned by the IK solver

function [qSol, info] = computeIK_UR5(ctx, p)

    % Compute yaw angle so the tool faces the target position
    % atan2 gives the angle in the XY plane
    yaw = atan2(p(2), p(1)) - pi/2;

    % Define desired orientation using ZYX Euler angles
    % [thetaZ thetaY thetaX x y z]
    thetaZ = yaw;       % rotation around Z (align with target)
    thetaY = 0;         % no tilt around Y
    thetaX = -1.5708;   % fixed downward orientation (~ -90 degrees)

    % Extract position components
    x = p(1);
    y = p(2);
    z = p(3);

    % Build rotation matrix from axis-angle rotations
    Rz = axang2rotm([0 0 1 thetaZ]); % rotation around Z axis
    Ry = axang2rotm([0 1 0 thetaY]); % rotation around Y axis
    Rx = axang2rotm([1 0 0 thetaX]); % rotation around X axis

    % Combined rotation (ZYX order)
    R  = Rz * Ry * Rx;

    % Build homogeneous transformation matrix
    T = eye(4);
    T(1:3,1:3) = R;          % rotation part
    T(1:3,4)   = [x; y; z];  % translation part

    % Initial guess (seed): current robot joint configuration
    % If unavailable, fall back to home configuration
    try
        q0 = getJointConfiguration(ctx.ur);
    catch
        q0 = homeConfiguration(ctx.ur5);
    end
    
    % Normalize angles to [-pi, pi]
    q0 = wrapToPi(q0);

    % Compute IK solution using the initial guess
    [qSol, info] = ctx.ik(ctx.eeName, T, ctx.weights, q0);

    % Normalize resulting joint angles
    qSol = wrapToPi(qSol);

    % Initialize best solution tracking
    bestSol = qSol;

    % Cost = distance from initial configuration (encourages smooth motion)
    bestCost = norm(qSol - q0);
    
    % Try multiple random initial guesses to avoid local minima
    for k = 1:20
        % Generate a random perturbation around the initial guess
        qTest = ctx.ik(ctx.eeName, T, ctx.weights, q0 + 0.2*randn(size(q0)));
        qTest = wrapToPi(qTest);

        % Compute cost relative to initial configuration
        cost = norm(qTest - q0);
        
        % Keep solution if it is better (closer to q0)
        if cost < bestCost
            bestSol = qTest;
            bestCost = cost;
        end
    end

    % Return the best solution found
    qSol = bestSol;
end