function [omegas, omega_dots, v_dots, v_dots_cg, F, N, f, n] =  newton_euler(dh_table, v_dot_0, masses, inertias, P_c_i)
    omegas = find_omegas(dh_table);
    omega_dots = find_omega_dots(dh_table);
    v_dots = find_v_dots(dh_table, v_dot_0);
    v_dots_cg = find_v_dots_cg(dh_table, v_dot_0, P_c_i);
    [i_max, ~] = size(dh_table);
    F = cell(1, i_max);
    N = cell(1, i_max);
    for i=1:i_max-1
        F{i} = masses{i} * v_dots_cg{i};
        N{i} = inertias{i} * omega_dots{i} + cross(omegas{i}, inertias{i} * omegas{i});
    end
    
    % Inwards calculation of forces and torques
    f = cell(1, i_max);
    n = cell(1, i_max);
    
    for k=1:i_max
        f{k} = zeros(3, 1);
        n{k} = zeros(3, 1);
    end

    for i=i_max-1:-1:1
        T_i = find_T_i(dh_table, i+1, true);
        R_i = T_i(1:3, 1:3);
        f{i} = R_i * f{i+1} + F{i};
        n{i} = N{i} + R_i * n{i+1} + cross(P_c_i{i}, F{i}) + cross(T_i(1:3, 4), R_i * f{i+1});
    end
        end