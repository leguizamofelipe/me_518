clc
clear all

for k=[1, 50, 100, 150, 200, 250, 300]
    % Armature parameters
    L = 0.0006; % Armature inductance
    R = 1.4; % Armature resistance

    % Constants
    Ka = 12; % Amp gain
    Km = 4.375; % Torque constant
    Kb = 0.00867; % Back emf constant
    Ke = 1; % Encoder transfer function

    % Inertia and damping
    n = 200; % Gear ratio
    J_m = 0.00844;
    J_l = 1*k;
    C_l = 0.5*k;
    C_m = 0.00013;
    J = J_m + J_l/n^2;
    C = C_m + C_l/n^2;
    
    hold on
    sim('final_model.slx');
    xlabel('Time');
    ylabel('Angle (deg)');
    [r, c] = size(ans.theta);
    plot(linspace(0,5, r), ans.theta, 'DisplayName', num2str(k));
    legend()
end

hold off