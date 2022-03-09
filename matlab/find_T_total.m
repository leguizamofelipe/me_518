function result = find_T_total(dh_table, symbolic)
    [r, c] = size(dh_table);
    result = eye(c);
    for i = 1:r
        result = result * find_T_i(dh_table, i, symbolic);
    end
end