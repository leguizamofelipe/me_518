function [omega_dot_list] = find_omega_dots(dh_table)
    omega_list = find_omegas(dh_table);
    [i_max, ~] = size(dh_table);
    omega_dot_list = cell(1, i_max);
    first_loop = 1;
    for i=0:i_max-1
        if first_loop == 1
           omega_dot_i = [0 0 0].'; % This assumes that the universal frame has no rotation
           omega_i = [0 0 0].';
           first_loop = 0;
        else
           omega_dot_i = omega_dot_list{i}; 
           omega_i = omega_list{i};

        end
        
        theta_dot_i_plus_1 = sym(strcat('t_dot_', num2str(i+1)));
        theta_double_dot_i_plus_1 = sym(strcat('t_double_dot', num2str(i+1)));
        %fprintf('Finding omega %d (i=%d):\n', i+1, i) 
        T_i_plus_1 = find_T_i(dh_table, i+1, true);
        R_i_plus_1 = T_i_plus_1(1:3,1:3);
        omega_dot_i_plus_1 =  R_i_plus_1.' * omega_dot_i + cross(R_i_plus_1.' * omega_i, [0 0 theta_dot_i_plus_1].') + [0 0 theta_double_dot_i_plus_1].'; %Changed here
        omega_dot_list{i+1} = omega_dot_i_plus_1;
        %disp(omega_dot_i_plus_1)
    end