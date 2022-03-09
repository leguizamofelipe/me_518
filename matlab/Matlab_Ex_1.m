% Link lengths given in problem statement
L_1 = 4;
L_2 = 3;
L_3 = 2;

% Joint angles
theta = [0, 0, 0];

%% PART A - Define DH parameters in a 4x4 matrix
% a alpha d theta (degrees)
dh_table = [    0   0 0 theta(1); 
                L_1 0 0 theta(2); 
                L_2 0 0 theta(3);
                ];
            
%% PART B - Find i-1/i_T, where i is the argument to the function
T_01 = find_T_i(dh_table, 1, true); %0/1
T_12 = find_T_i(dh_table, 2, true); %1/2
T_23 = find_T_i(dh_table, 3, true); %2/3

disp('Part b: T_01, T_12, T_23 symbolic')
disp(T_01);
disp(T_12);
disp(T_23);

% 3/H T by inspection
T_3H = [1 0 0 L_3
        0 1 0 0
        0 0 1 0
        0 0 0 1];
    
disp('Part b: T_3H by inspection')
disp(T_3H)

%% PART C - Forward pose kinematics solution for T_03, T_0H  

% List of cases
thetas = [[0,0,0]; [10,20,30]; [90,90,90]];

T_03 = find_T_total(dh_table, true);
T_0H = T_03 * T_3H;
disp('Part c: T_03, T_0H symbolic')
disp(T_03)
disp(T_0H)

for k=1:size(thetas,1)
    theta = thetas(k, :);
    
    fprintf('********thetas = [%1.1f,%1.1f,%1.1f]**********\n', theta(1), theta(2), theta(3));
    
    dh_table = [    0   0 0 theta(1); 
                    L_1 0 0 theta(2); 
                    L_2 0 0 theta(3);
                    ];

    T_03 = find_T_total(dh_table, false);
    T_0H = T_03 * T_3H;
    disp('Part c: T_03, T_0H numeric')
    disp(T_03)
    disp(T_0H)

    % Convert angles to radians
    theta = pi/180 * theta;

    % Use toolbox to check answers
    % 0 is revolute joint, 1 is prismatic
    % THETA D A ALPHA SIGMA OFFSET
    Link1 = Link([theta(1), 0, 0, 0], 'modified');
    Link2 = Link([theta(2), 0, L_1, 0], 'modified');
    Link3 = Link([theta(3), 0, L_2, 0], 'modified');
    Link4 = Link([1, 0, L_3, 0], 'modified');

    robot = SerialLink([Link1 Link2 Link3, Link4], 'name', 'Matlab1 Robot');
    robot1 = SerialLink([Link1 Link2 Link3], 'name', 'Matlab1 Robot');
    
    disp('Part c: T_03, T_0H calculated with toolbox')
    T_03 = robot1.fkine([theta(1) theta(2) theta(3)]);
    T_0H = robot.fkine([theta(1) theta(2) theta(3), 0]);
    disp(T_03);
    disp(' ')
    disp(T_0H);
end

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

function result = find_T_total(dh_table, symbolic)
    [r, c] = size(dh_table);
    result = eye(c);
    for i = 1:r
        result = result * find_T_i(dh_table, i, symbolic);
    end
end