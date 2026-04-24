close all
clear

% create figure
figure
axis([-2.5, 2.5, -2, 2])
grid on
hold on

% pick your system parameters
deltaT = 0.01;
m1 = 1;
m2 = 1;
Iz1 = 0.1;
Iz2 = 0.1;
L = 1;
g = 9.81;
tau = [0; 0];

% design your control law
%%% [your controller gains and desired position here]

% initial conditions
theta = [0; 0];
thetadot = [0; 0];
thetadotdot = [0; 0];

% main loop
for idx = 1:500
    
    % plot the robot
    % 1. get the position of each link
    p0 = [0; 0];
    p1 = L*[cos(theta(1)); sin(theta(1))];
    p2 = p1 + L*[cos(theta(1)+theta(2)); sin(theta(1)+theta(2))];
    P = [p0, p1, p2];
    % 2. draw the robot and save the frame
    cla;
    plot(P(1,:), P(2,:), 'o-', 'color',[1, 0.5, 0],'linewidth',4)
    drawnow
    
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
    %%% tau = [your control law here]
    tau = 0;

    % update robot acceleration based on controller and dynamics
    thetadotdot = M \ (tau - C*thetadot - G);
    
end