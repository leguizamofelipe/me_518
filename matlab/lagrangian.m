function [lagrangian, k, u] = lagrangian(dh_table)
    %omega_dots, v_dots, v_dots_cg, F, N, f, n, masses)
    [i_max, ~] = size(dh_table);
    k = cell(1, i_max-1);
    v = cell(0, i_max-1);
    u = cell(0, i_max-1);
    lagrangian = 0;
    for i=1:i_max-1
        T_i_i_minus_1 = find_T_i(dh_table, i-1, true);
        R_i_i_minus_1 = T_i_i_minus_1(1:3, 1:3);
        P_i_i_minus_1 = T_i_i_minus_1(1:3, 4);

        T_0_i = find_T_total(dh_table, i, true);
        R_0_i = T_0_i(1:3, 1:3);
        P_0_i = T_0_i(1:3, 4);

        v{i} = R_i_i_minus_1 * (v{i-1} + cross(omegas{i}, P_i_i_minus_1));
        % For HW 3 assume 0vCi = 0vi
        v_0i = R_0_i * v{i};
        
        k{i} = 0.5 * masses{i} * R_0_i * (v_0i).' * v_0i + 0.5 * omegas{i} * inertias{i} * omegas{i};
        u{i} = -masses{i} * g.' * P_0_i;
        lagrangian = lagrangian + k{i} - u{i};
    end
    
end
