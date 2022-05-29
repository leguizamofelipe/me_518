function [omega_list] = find_omegas(dh_table)
    [i_max, ~] = size(dh_table);
    omega_list = cell(1, i_max);
    first_loop = 1;
    for i=0:i_max-1
        if first_loop == 1
           omega_i = [0 0 0].'; % This assumes that the universal frame has no rotation
           first_loop = 0;
        else
           omega_i = omega_list{i}; 
        end
        
        d_var_name = char(dh_table(i+1, 3));
        if contains(d_var_name, 'd')
            prismatic = true;
        else
            prismatic = false;
        end
        
        if prismatic==false
            theta_dot_i_plus_1 = sym(strcat('t_dot_', num2str(i+1)));
        else
            theta_dot_i_plus_1 = 0;
        end
                
        %fprintf('Finding omega %d (i=%d):\n', i+1, i) 
        T_i_plus_1 = find_T_i(dh_table, i+1, true);
        R_i_plus_1 = T_i_plus_1(1:3,1:3);
        omega_i_plus_1 =  R_i_plus_1.' * omega_i + [0 0 theta_dot_i_plus_1].';
        omega_list{i+1} = omega_i_plus_1;
        %disp(omega_i_plus_1)
    end
end