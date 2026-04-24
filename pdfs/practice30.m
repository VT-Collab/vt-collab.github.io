clear
clc

% initial guess
w0 = [1; 1];
% lower bound (weights must be positive for stability)
lb = [0; 0];
% upper bound (weights cannot exceed this amount)
ub = [2000; 2000];
% optimize the function using gradient descent
options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
w = fmincon(@robot_model,w0,[],[],[],[],lb,ub,[],options);

% function that simulates the robot arm with control parameters w
% returns the cumulative error between theta and theta_d
% we previously used this code in Lecture 27 on Robot Control
function loss = robot_model(w)

    % pick your system parameters
    deltaT = 0.01;
    m1 = 1;
    m2 = 1;
    Iz1 = 0.1;
    Iz2 = 0.1;
    L = 1;
    g = 9.81;
    
    % design your control law
    Kp = diag([w(1), w(2)]);
    Kd = 20 * eye(2);
    theta_d = [0; pi/2];
    
    % initial conditions
    theta = [0; 0];
    thetadot = [0; 0];
    thetadotdot = [0; 0];
    
    % main loop
    loss = 0;
    for idx = 1:300
        
        % integrate to update velocity and position
        thetadot = thetadot + deltaT * thetadotdot;
        theta = theta + deltaT * thetadot;
    
        % dynamics
        M = [Iz1 + Iz2 + L^2*m1 + 2*L^2*m2 + 2*L^2*m2*cos(theta(2)),... 
            Iz2 + L^2*m2 + L^2*m2*cos(theta(2));...
            Iz2 + L^2*m2 + L^2*m2*cos(theta(2)), m2*L^2 + Iz2];
        C = [-L^2*m2*thetadot(2)*sin(theta(2)),...
            -L^2*m2*thetadot(1)*sin(theta(2)) - L^2*m2*thetadot(2)*sin(theta(2));
            L^2*m2*thetadot(1)*sin(theta(2)), 0];
        G = [g*m2*(L*cos(theta(1) + theta(2)) + L*cos(theta(1))) + L*g*m1*cos(theta(1));
                L*g*m2*cos(theta(1) + theta(2))];
    
        % use your control law to calculate tau
        tau = Kp*(theta_d - theta) - Kd*thetadot + G;
    
        % update robot acceleration based on controller and dynamics
        thetadotdot = M \ (tau - C*thetadot - G);
    
        % update loss
        loss = loss + deltaT*norm(theta - theta_d);    
        
    end
end