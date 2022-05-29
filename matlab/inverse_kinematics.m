function [sols] = inverse_kinematics(T_0_h, l_1, l_2, l_3)
    T_3_H = [1 0 0 l_3; 0 1 0 0; 0 0 1 0; 0 0 0 1];
    
    T_0_3 = T_0_h * T_3_H^-1;
    
    sphi = T_0_3(2, 1);
    cphi = T_0_3(1, 1);
    
    phi = atan2(sphi, cphi);
    
    x = T_0_3(1,4);
    y = T_0_3(2,4);    
    
    c2 = (x^2 + y^2 -l_1^2-l_2^2)/(2*l_1*l_2);
    first_loop = true;
    for i=1:2
        if first_loop == true
            s2 = (1-c2^2)^0.5;
        else
            s2 = -(1-c2^2)^0.5;
        end

        t2 = atan2(s2, c2);

        k1 = l_1 + l_2*c2;

        k2 = l_2 * s2;

        t1 = atan2(y,x) - atan2(k2, k1);

        t3 = phi - t1 - t2;
        
        first_loop = false;

        sols{i} = [t1, t2, t3];
    end

end