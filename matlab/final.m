%% Part 1 

% Problem 1
syms d_1
syms d_2
syms d_3

% a alpha d theta (degrees)
dh_table = [0 0 d_1 0; 0 90 d_2 -90; 0 -90 d_3 0];

T_1 = find_T_i(dh_table, 1, false);
T_2 = find_T_i(dh_table, 2, false);
T_3 = find_T_i(dh_table, 3, false);

T_0_1 = T_1;
T_0_2 = T_1 * T_2;
T_0_3 = T_1 * T_2 * T_3;

disp('T_0_1');
disp(T_0_1);
disp('T_0_2');
disp(T_0_2);
disp('T_0_3');
disp(T_0_3);

%Problem 2 by hand

%Problem 3
clear all

syms t_1
syms t_3

syms L_2

syms Ixx1
syms Iyy1
syms Izz1

syms Ixx3
syms Iyy3
syms Izz3

syms d2

% a alpha d theta (degrees)
dh_table = [0 0 0 t_1; 0 90 d2 0; 0 0 L_2 t_3; 0 0 0 0];
syms g
v_dot_0 = [0 0 g].';
syms m1
syms m2
syms m3
masses = {m1, m2, m3};
inertias = {[Ixx1 0 0; 0 Iyy1 0; 0 0 Izz1], zeros(3), [Ixx3 0 0; 0 Iyy3 0; 0 0 Izz3]};

P_c_i = {[0 0 0].', [0 0 0].', [0 0 0].'};

[omegas, omega_dots, v_dots, v_dots_cg, F, N, f, n] = newton_euler(dh_table, v_dot_0, masses, inertias, P_c_i);

tau_1 = n{1}(3);
tau_2 = f{2}(3);
tau_3 = n{3}(3);

disp('tau 1:');
disp(tau_1);

disp('tau 2:');
disp(tau_2);

disp('tau 3:');
disp(tau_3);

%% Part 2

% Part a
clear all

t_0 = 120;
t_dot_0 = 0;
t_dot_f = 0;
t_f = 60;
time_f = 1;

a_0 = t_0;
a_1 = t_dot_0;
a_2 = 3*(t_f-t_0)/(time_f^2)-2*t_dot_0/time_f-1*t_dot_f/time_f;
a_3 = -2*(t_f-t_0)/(time_f^3)+1*(t_dot_f-t_dot_0)/(time_f^2);

t = 0:0.01:1;

[r, c] = size(t);

figure()
subplot(4,1,1)
plot(t, a_0 + a_1*t + a_2*t.^2 + a_3*t.^3) 
ylabel('position (°)')
subplot(4,1,2)
plot(t, a_1 + 2*a_2*t + 3*a_3*t.^2)
ylabel('velocity (°/s)')
subplot(4,1,3)
plot(t, 2*a_2 + 6*a_3*t) 
ylabel('acceleration (°/s^2)')
subplot(4,1,4)
plot(t, 6*a_3*ones(r,c)) 
ylabel('jerk (°/s^3)')
xlabel('Time (s)');
sgtitle('Part a: third order polynomial');

% Part b
clear all

t_0 = 120;
t_f = 60;
t_dot_0 = 0;
t_dot_f = 0;
t_double_dot_0 = 0;
t_double_dot_f = 0;
time_f = 1;

a_0 = t_0;
a_1 = t_dot_0;
a_2 = t_double_dot_0/2;
a_3 = (20*t_f-20*t_0-(8*t_dot_f+12*t_dot_0)*time_f-(3*t_double_dot_0-t_double_dot_f)*time_f^2)/(2*time_f^3);
a_4 = (30*t_0-30*t_f+(14*t_dot_f+16*t_dot_0)*time_f+(3*t_double_dot_0-2*t_double_dot_f)*time_f^2)/(2*time_f^4);
a_5 = (12*t_f-12*t_0-(6*t_dot_f+6*t_dot_0)*time_f-(t_double_dot_0-t_double_dot_f)*time_f^2)/(2*time_f^5);

t = 0:0.01:1;

figure()
subplot(4,1,1)
plot(t, a_0 + a_1*t + a_2*t.^2 + a_3*t.^3 + a_4*t.^4 + a_5*t.^5) 
ylabel('position (°)')
subplot(4,1,2)
plot(t, a_1 + 2*a_2*t + 3*a_3*t.^2 + 4*a_4*t.^3 + 5*a_5*t.^4)
ylabel('velocity (°/s)')
subplot(4,1,3)
plot(t, 2*a_2 + 6*a_3*t + 12*a_4*t.^2 + 20*a_5*t.^3) 
ylabel('acceleration (°/s^2)')
subplot(4,1,4)
plot(t, 6*a_3 + 24*a_4*t + 60*a_5*t.^2) 
ylabel('jerk (°/s^3)')
xlabel('Time (s)');
sgtitle('Part b: fifth order polynomial');

% Part c
clear all

a_10 = 60;
a_11 = 0;
syms a_12
syms a_13

syms a_20
syms a_21
syms a_22
syms a_23

vars = [a_12, a_13, a_20, a_21, a_22, a_23];

eqns = [a_10+a_11+a_12+a_13==120, ...
        a_20+a_21+a_22+a_23==120, ...
        a_21+4*a_22+12*a_23==0,   ...
        2*a_12+6*a_13==2*a_22+6*a_23, ...
        a_20+2*a_21+4*a_22+8*a_23==30, ...
        a_11+2*a_12+3*a_13==a_21+2*a_22+3*a_23];

sols = solve(eqns, vars);

a_12 = double(sols.a_12);
a_13 = double(sols.a_13);

a_20 = double(sols.a_20);
a_21 = double(sols.a_21);
a_22 = double(sols.a_22);
a_23 = double(sols.a_23);

t1 = 0:0.01:1;
t2 = 1:0.01:2;
[r, c] = size(t2);

figure()
subplot(4,1,1)
hold on
plot(t1, a_10 + a_11*t1 + a_12*t1.^2 + a_13*t1.^3) 
plot(t2, a_20 + a_21*t2 + a_22*t2.^2 + a_23*t2.^3) 
ylabel('position (°)')
subplot(4,1,2)
hold on
plot(t1, a_11 + 2*a_12*t1 + 3*a_13*t1.^2)
plot(t2, a_21 + 2*a_22*t2 + 3*a_23*t2.^2)
ylabel('velocity (°/s)')
subplot(4,1,3)
hold on
plot(t1, 2*a_12 + 6*a_13*t1) 
plot(t2, 2*a_22 + 6*a_23*t2) 
ylabel('acceleration (°/s^2)')
subplot(4,1,4)
hold on
plot(t1, 6*a_13*ones(r,c)) 
plot(t2, 6*a_23*ones(r,c)) 
ylabel('jerk (°/s^3)')
xlabel('Time (s)');
sgtitle('Part c: two third order polynomials');
