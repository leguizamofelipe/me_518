function [v_dot_cg_list] = find_v_dots_cg(dh_table, v_dot_0, P_c_i)
    omega_list = find_omegas(dh_table);
    omega_dot_list = find_omega_dots(dh_table);
    v_dot_list = find_v_dots(dh_table, v_dot_0);
    [i_max, ~] = size(dh_table);
    v_dot_cg_list = cell(1, i_max);
    for i=0:i_max-2        
        % Assume mass is concentrated with no inertia
        P_c_i_plus_1 = P_c_i{i+1};
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        v_dot_i_plus_1_cg = cross(omega_dot_list{i+1}, P_c_i_plus_1) + cross(omega_list{i+1}, cross(omega_list{i+1}, P_c_i_plus_1)) + v_dot_list{i+1};
        v_dot_cg_list{i+1} = v_dot_i_plus_1_cg;
        %disp(omega_i_plus_1)
    end
end