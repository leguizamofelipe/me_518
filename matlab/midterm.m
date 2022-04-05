%% #1
%Try theta = pi/2, phi = pi/4, compare to hand solution
theta = 90;
phi = 45;

R_ab = rotz(theta) * rotx(phi);

%% #2
alpha = atan(3/4) * 180/pi; % Calculate angles as on hand solution
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

rot_ca = rot_ac';
T_ca = zeros(4);
T_ca(1:3, 1:3) = rot_ca;
T_ca(1:3, 4) = -rot_ac' * [-3; 4; 2]; %c_Paorg
T_ca(4, :) = [0, 0, 0, 1]; % Last row of T
T_ca(4,4) = 1;

%% #3

syms theta1
syms theta2
syms theta3
syms L_2

% a alpha d theta (degrees)
dh_table = [0 0 0 theta1; 0 90 0 theta2; L_2 0 0 theta3];

% Robot a
disp('****************************Robot a')

T_01 = find_T_i(dh_table, 1, true);
T_02 = T_01 * find_T_i(dh_table, 2, true);
T_03 = T_01 * find_T_i(dh_table, 2, true) * find_T_i(dh_table, 3, true);

disp(T_01);
disp(T_02);
disp(T_03);

T_total = find_T_total(dh_table, true);

disp(T_total);

% Robot b

% a alpha d theta (degrees)
syms d_1
syms L_1
dh_table = [0 0 d_1 0; L_1 0 0 theta2; L_2 0 0 theta3];

disp('****************************Robot b')
T_01 = find_T_i(dh_table, 1, true);
T_02 = T_01 * find_T_i(dh_table, 2, true);
T_03 = T_01 * find_T_i(dh_table, 2, true) * find_T_i(dh_table, 3, true);

disp(T_01);
disp(T_02);
disp(T_03);

T_total = find_T_total(dh_table, true);

disp(T_total);

%% #5
% a alpha d theta (degrees)
syms thetadot1
syms thetadot2
syms thetadot3

syms theta1
syms theta2
syms theta3

syms L_1
syms L_2
syms L_3

dh_table = [0 0 0 theta1; L_1 0 0 theta2; L_2 0 0 theta3];

T_01 = find_T_i(dh_table, 1, true);
T_12 = find_T_i(dh_table, 2, true);
T_23 = find_T_i(dh_table, 3, true);

R_01 = T_01(1:3, 1:3);
R_12 = T_12(1:3, 1:3);
R_23 = T_23(1:3, 1:3);

w_11 = [0; 0; thetadot1];
v_11 = [0 0 0].';

w_22 = R_12.' * w_11 + thetadot2 * [0 0 1].';
v_22 = R_12.' * (v_11 + cross(w_11, [L_1 0 0].'));

w_33 = R_23.' * w_22 + thetadot3 * [0 0 1].';
v_33 = R_23.' * (v_22 + cross(w_22, [L_2 0 0].'));

R_34 = eye(3);

w_44 = R_34.' * w_33;
v_44 = R_34.' * (v_33 + cross(w_33, [L_3 0 0].'));

j_vel = jacobian(v_44, [thetadot1; thetadot2; thetadot3]);

% Force propagation
syms Fx
syms Fy
syms Fz

n_44 = [0 0 0].';

f_33 = [Fx; Fy; Fz];
n_33 = R_34 * n_44 + cross([L_3; 0; 0], f_33);

f_22 = R_23*f_33;
n_22 = R_23*n_33 + cross([L_2; 0; 0], f_22);

f_11 = R_12*f_22;
n_11 = R_12*n_22 + cross([L_1; 0; 0], f_11);

tau = [n_11(3) n_22(3) n_33(3)].';

j_force = jacobian(tau, [Fx; Fy; Fz]);
