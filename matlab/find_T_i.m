function T_i = find_T_i(dh_table, i, symbolic)
    a_i_minus_1     = dh_table(i, 1);
    alpha_i_minus_1 = dh_table(i, 2);
    d_i             = dh_table(i, 3);
    theta_i         = dh_table(i, 4);
    
    ci = cosd(theta_i);
    si = sind(theta_i);
        
    if symbolic==true
        c = sym(strcat('c', num2str(i)));
        s = sym(strcat('s', num2str(i)));

        syms ci
        syms si
        
        ci = subs(c, c);
        si = subs(s, s);
    end

    T_i =   [       
            [        ci                 , -si                               ,          0            , a_i_minus_1                  ]
            [si * cosd(alpha_i_minus_1) ,  ci * cosd(alpha_i_minus_1)       , -sind(alpha_i_minus_1), -sind(alpha_i_minus_1) * d_i ]
            [si * sind(alpha_i_minus_1) ,  ci * sind(alpha_i_minus_1)       ,  cosd(alpha_i_minus_1),  cosd(alpha_i_minus_1) * d_i ]
            [             0             ,                    0              ,          0            , 1                            ]   
    ];
end
