function [v_dot_list] = find_v_dots(dh_table, v_dot_0)
    omega_list = find_omegas(dh_table);
    omega_dot_list = find_omega_dots(dh_table);
    [i_max, ~] = size(dh_table);
    v_dot_list = cell(1, i_max);
    first_loop = 1;
    for i=0:i_max-1
        if first_loop == 1
           omega_i = [0 0 0].'; % This assumes that the universal frame has no rotation
           omega_i_dot = [0 0 0].';
           v_dot_i = v_dot_0;
           first_loop = 0;
        else
           omega_i = omega_list{i}; % This assumes that the universal frame has no rotation
           omega_i_dot = omega_dot_list{i};
           v_dot_i = v_dot_list{i}; 
        end
        
        d_dot_i_plus_1 = sym(strcat('d_dot_', num2str(i+1)));
        d_double_dot_i_plus_1 = sym(strcat('d_double_dot', num2str(i+1)));
        
        T_i_plus_1 = find_T_i(dh_table, i+1, true);
        R_i_plus_1 = T_i_plus_1(1:3,1:3);
        P_i = T_i_plus_1(1:3, 4);
        
        if dh_table(i+1, 3) == 0
            prismatic = false;
        else
            prismatic = true;
        end
        
        if prismatic==false
            v_dot_i_plus_1 = R_i_plus_1.' * (cross(omega_i_dot, P_i) + cross(omega_i, cross(omega_i, P_i)) + v_dot_i);
        else
            v_dot_i_plus_1 = R_i_plus_1.' * (cross(omega_i_dot, P_i) + cross(omega_i, cross(omega_i, P_i)) + v_dot_i) + cross(2 * omega_list{i+1}, [0 0 d_dot_i_plus_1].') + [0 0 d_double_dot_i_plus_1].';
        end
        
        v_dot_list{i+1} = v_dot_i_plus_1;
        %disp(omega_i_plus_1)
    end
end