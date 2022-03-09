%% #1
%Try theta = pi/2, phi = pi/4, compare to hand solution
theta = 90;
phi = 45;

R_ab = rotz(theta) * rotx(phi);

%% #2
alpha = atan(3/4) * 180/pi; % Calculate angles as on 
beta = 90-alpha;

rot_ac = rotx(180) * rotz(-alpha); % ac_R

rot_bc = rotx(-90) * rotz(90 + beta); %bc_R

T_ac = zeros(4);
T_ac(1:3, 1:3) = rot_ac;
T_ac(:, 4) = [-3; 4; 2; 1]; %a_Pcorg
T_ac(4, :) = [0, 0, 0, 1]; % Last row of T

T_bc = zeros(4);
T_bc(1:3, 1:3) = rot_bc;
T_bc(:, 4) = [3; 0; 0; 1]; %b_Pcorg
T_bc(4, :) = [0, 0, 0, 1]; % Last row of T

%% #3

syms theta1
syms theta2
syms theta3
syms L_2

% a alpha d theta (degrees)
dh_table = [0 0 0 theta1; 0 90 0 theta2+90; L_2 0 0 theta3];

% Robot a
for i=1:3
    T_i = find_T_i(dh_table, i, true);
    disp(i);
    disp(T_i);
end

T_total = find_T_total(dh_table, true);

% a alpha d theta (degrees)
syms d1
syms L_1
dh_table = [0 0 d1 0; L_1 0 0 theta2; L_2 0 0 theta3];

% Robot b
for i=1:3
    T_i = find_T_i(dh_table, i, true);
    disp(i);
    disp(T_i);
end

T_total = find_T_total(dh_table, true);
    
