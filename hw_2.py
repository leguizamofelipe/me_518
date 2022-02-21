from kinematics_from_dh import *

########################## BEGIN USER DEFINED SECTION ###############
L_1 = 1
L_2 = 2
L_3 = 4
theta_1 = 0
theta_2 = 30
theta_3 = 0

d_2 = 5
########################## END USER DEFINED SECTION ###############

# Angle vals in degrees
p1_0 = np.array([
    #a_i-1 |alpha_i-1|     d     | theta
    [   0   ,   0    ,     0     , theta_1], #1
    [  L_1  ,   90   ,     0     , theta_2], #2
    [  L_2  ,   0    ,     0     , theta_3], #3
])

# Angle vals in degrees
p2_0 = np.array([
    #a_i-1 |alpha_i-1|     d     | theta
    [0     ,     0    ,    L_1    , theta_1], #1
    [0     ,     90   ,    L_2    , theta_2], #2
    [L_3   ,     0    ,     0     , theta_3], #3
])

# Angle vals in degrees
p2_1 = np.array([
    #a_i-1 |alpha_i-1|     d     | theta
    [0     , 0       , L_1 + L_2 , theta_1], #1
    [0     , 90      ,     0     , theta_2], #2
    [L_3   , 0       ,     0     , theta_3], #3
])

# find_T_i(p2_1, 1)
# find_T_i(p2_1, 2)
# find_T_i(p2_1, 3)


# T_p2_0 = find_T_total(p2_0)

a = find_T_i(p1_0, 1)
b = find_T_i(p1_0, 2)

T_p2_2 = np.matmul(a, b)
print('\n')
print(T_p2_2)

# T_p2_1 = find_T_total(p1_0, print_intermediate=True)
