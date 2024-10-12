import numpy as np
import sympy as sp
from math import pi
import roboticstoolbox as rtb
from spatialmath import SE3
# from HW3_utils import FKHW3 
# # Define the tool transformation matrix with the correct Y translation
# tool_transformation = (
#     SE3.Tx(-0.39243-0.082) @  # Translation along X-axis
#     SE3.Tz(0.109) @     # Translation along Z-axis
#     SE3.Ty(-0.093) @    # Translation along Y-axis (negative)
#     SE3.Ry(-pi/2)      # Rotation around Y-axis by 
# )
# # Define the robot using the specified MDH parameters and tool transformation
# robot = rtb.DHRobot(
#     [
#         rtb.RevoluteMDH(d=0.0892, offset=pi),
#         rtb.RevoluteMDH(alpha=pi/2),
#         rtb.RevoluteMDH(a=-0.425),
#     ],
#     tool=tool_transformation,
#     name="RRR_Robot"
# )

# #=============================================<คำตอบข้อ 1>======================================================#
# #code here
# def endEffectorJacobianHW3(q:list[float],ref: str = "0")->list[float]:
#     if ref == "0":
#         J_e = robot.jacob0(q)
#     elif ref == "e":
#         J_e = robot.jacobe(q)
    
#     print('J_e is \n', J_e)
#     return J_e

# q = [0.0,0.0,0.0]
# endEffectorJacobianHW3(q)
# # print('tool_tf is \n',tool_transformation)

############################################################
    # #Prepare Variable
    # R, P, R_e, p_e = HW3_utils.FKHW3(q) 
    # # 1). See Overall of Forward Kinematic Matrix
    #     # Joint positions: p_i for each joint i
    # p_0 = np.array([0, 0, 0])   # Origin (base frame)
    # p_1 = P[:, 0]               # Position of the first joint
    # p_2 = P[:, 1]               # Position of the second joint
    # p_3 = P[:, 2]               # Position of the third joint
    
    # # End-effector position
    # p_e = P[:, 3]
    
    # # Rotation axes (z_i for each joint frame)
    # z_0 = np.array([0, 0, 1])   # z-axis of the base frame (always [0,0,1] for the base)
    # z_1 = R[:, 2, 0]            # z-axis of the first joint frame
    # z_2 = R[:, 2, 1]            # z-axis of the second joint frame
    
    # # Jacobian linear velocity part J_v
    # J_v1 = np.cross(z_0, p_e - p_0)  # First joint contribution
    # J_v2 = np.cross(z_1, p_e - p_1)  # Second joint contribution
    # J_v3 = np.cross(z_2, p_e - p_2)  # Third joint contribution
    
    # # Jacobian angular velocity part J_w
    # J_w1 = z_0  # First joint angular contribution
    # J_w2 = z_1  # Second joint angular contribution
    # J_w3 = z_2  # Third joint angular contribution
    
    # # Combine linear and angular parts into full Jacobian
    # J_v = np.column_stack((J_v1, J_v2, J_v3))
    # J_w = np.column_stack((J_w1, J_w2, J_w3))
    
    # # Full Jacobian: stack J_v (top 3 rows) and J_w (bottom 3 rows)
    # J = np.vstack((J_v, J_w))
    # # print('\n',P)
    # # print("R : \n",R)

    # return J

########################################################   

