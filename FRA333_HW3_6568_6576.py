# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1.ชุตินันท์_6568
2.สิปปนนท์_6576
'''

import numpy as np
import sympy as sp
from math import pi
import roboticstoolbox as rtb
from spatialmath import SE3
from HW3_utils import FKHW3 

d_1 = 0.0892
a_2 = -0.425
a_3 = -0.39243
d_4 = 0.109
d_5 = 0.093
d_6 = 0.082
c_minus_90 = 0
s_minus_90 = -1

# Find End effector and Create rbot

End_P = np.array([a_3 + (-d_6), -d_5 , d_4])
End_R = np.array([[c_minus_90,      0,  s_minus_90],
                  [0,               1,           0],
                  [-(s_minus_90),   0,  c_minus_90]])
End = np.eye(4)  
End[0:3, 3] = End_P
End[0:3, 0:3] = End_R
End_EF = SE3(End)
# print('End_EF is \n', End_EF)

rbot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(a= 0, d= d_1, offset=pi),
        rtb.RevoluteMDH(a= 0, alpha= pi/2),
        rtb.RevoluteMDH(a= a_2)
    ],
    tool = End_EF,
    name = "RRR_Rbot"
)

#===========================================<คำตอบข้อ 1>====================================================#
def endEffectorJacobianHW3(q:list[float])->list[float]:
    R, P, R_e, p_e = FKHW3(q)
    J_e = np.zeros((6, len(q)))

    for i in range(len(q)):

        Pn = p_e - P[:,i]
        Z_i = R[:,2,i] 

        J_e[:3,i] = (np.cross(Z_i, Pn)) # Linear Velo
        J_e[3:, i] = Z_i                # Angular Velo

    print("This RRR robot Jacobian(manual calculation) is: \n",J_e)
    return J_e


#===========================================<คำตอบข้อ 2>====================================================#
def checkSingularityHW3(q:list[float])->bool:
    epsilon = 0.001
    threshold = 0.01

    # Calculate the Jacobian matrix using the previous function
    J_e = endEffectorJacobianHW3(q)

    # Extract the 3x3 linear part of the Jacobian
    J_linear_manual = J_e[:3, :]

    # Compute the determinant of the linear Jacobian
    det_J_linear_manual = np.linalg.det(J_linear_manual)

    # Check if the determinant is close to zero (singular configuration)
    if abs(epsilon - abs(det_J_linear_manual)) < threshold:
        print("Flag = 1: Singularity detected.")
        return True  # Singular
    else:
        print("Flag = 0: Not in singularity.")
        return False  # Non-singular
    
#===========================================<คำตอบข้อ 3>====================================================#
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    J_e = endEffectorJacobianHW3(q)
    J_e_T = J_e.transpose()

    tau = J_e_T @ w

    print('tau is \n', tau)
    return tau


# q = [0.0,0.0,0.0]            
q = [0.0,-pi/2,-0.2]          #<--- create singularity
w = [10, 0, 0, 0, 0, 0]          #<---- Inset Value Here !
endEffectorJacobianHW3(q)
checkSingularityHW3(q)
computeEffortHW3(q,w)