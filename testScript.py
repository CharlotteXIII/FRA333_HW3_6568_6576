# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ex: ธนวัฒน์_6461)
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
# FK = rbot.fkine(q)
# Check rbot
# print(rbot)

#=============================================<ตรวจคำตอบข้อ 1>======================================================#
def endEffectorJacobianHW3(q:list[float])->list[float]:
    print('################# ข้อ 1 #################')

    J_e = rbot.jacob0(q)
    print("This RRR robot Jacobian is: \n",J_e) 

    print('\n')
    return J_e

#==============================================================================================================#


#=============================================<ตรวจคำตอบข้อ 2>======================================================#
def checkSingularityHW3(q:list[float])->bool:
    print('################# ข้อ 2 #################')

    epsilon = 0.001
    J_e = rbot.jacob0(q)

    # Use Only Linear Velocity
    J_e_linear = J_e[:3, :]

    # Find Det of Linear Velocity
    det_J_linear = np.linalg.det(J_e_linear)

    # absolute
    L = abs(det_J_linear)

    # Check Singularity or not
    if abs(L) < epsilon:
        print(" Flag = 1 is singularity.")
        print('\n')
        return 1
    else:
        print(" Flag = 0 is not singularity.")
        print('\n')
        return 0
#==============================================================================================================#


#=============================================<ตรวจคำตอบข้อ 3>======================================================#
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    print('################# ข้อ 3 #################')
    J_e = rbot.jacob0(q)
    # tau = rbot.pay(w, q, J_e)
    tau = np.dot(J_e.T, w)
    print('tau is \n', tau)
    print('\n')
    return tau
    
#==============================================================================================================#

q = [0.0,0.0,0.0]             #<---- Inset Value Here !
w = [10, 0, 0, 0, 0, 0]       #<---- Inset Value Here !
endEffectorJacobianHW3(q)
checkSingularityHW3(q)
computeEffortHW3(q,w)




