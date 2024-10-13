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

# ประกาศตัวแปร
d_1 = 0.0892
a_2 = -0.425
a_3 = -0.39243
d_4 = 0.109
d_5 = 0.093
d_6 = 0.082
c_minus_90 = 0
s_minus_90 = -1

# หา End effector และสร้าง rbot
# คำนวณตำแหน่งและการหมุนของ end-effector
End_P = np.array([a_3 + (-d_6), -d_5 , d_4]) # ตำแหน่งของ end-effector
End_R = np.array([[c_minus_90,      0,  s_minus_90], # เมทริกซ์การหมุน
                  [0,               1,           0],
                  [-(s_minus_90),   0,  c_minus_90]])
End = np.eye(4)  # สร้างเมทริกซ์ขนาด 4x4
End[0:3, 3] = End_P # กำหนดส่วนของตำแหน่งในเมทริกซ์
End[0:3, 0:3] = End_R # กำหนดส่วนของการหมุนในเมทริกซ์
End_EF = SE3(End) # กำหนด end effector ในรูปแบบ SE3 (การแปลงHomogeneous)
# print('End_EF is \n', End_EF)

# กำหนดโมเดลหุ่นยนต์โดยใช้ DH Parameter
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
# ฟังก์ชันสำหรับคำนวณเมทริกซ์ Jacobian ของ end-effector
def endEffectorJacobianHW3(q:list[float])->list[float]:
    print('################# คำตอบข้อ 1 #################')


    # คำนวณ forward kinematics จากฟังก์ชัน FKHW3
    R, P, R_e, p_e = FKHW3(q)
    # สร้างเมทริกซ์ Jacobian ขนาด 6xn
    J_e = np.zeros((6, len(q)))

    #หา jacobian ในแต่ละ joints
    for Joint in range(len(q)):

        Pose = p_e - P[:,Joint] 
        Z_i = R[:,2,Joint] 

        J_e[:3,Joint] = (np.cross(Z_i, Pose)) # Linear Velocity
        J_e[3:, Joint] = Z_i                # Angular Velocity

    # แสดงผลเมทริกซ์ Jacobian ที่คำนวณได้
    print("This RRR robot Jacobian(manual calculation) is: \n", J_e)
    print('\n')
    return J_e


#===========================================<คำตอบข้อ 2>====================================================#
# ฟังก์ชันสำหรับตรวจสอบว่าหุ่นยนต์อยู่ในสภาวะ Singular หรือไม่
def checkSingularityHW3(q:list[float])->bool:

    # ประกาศตัวแปร
    epsilon = 0.001 
    difference = 0.01

    # คำนวณ Jacobian แบบ manual
    # J_e = endEffectorJacobianHW3(q)

    # แยกส่วนเมทริกซ์ 3x3 ของความเร็วเชิงเส้นใน Jacobian
    J_linear_manual = J_e[:3, :]

    # คำนวณ determinant ของเมทริกซ์ Jacobian เชิงเส้น
    det_J_linear_manual = np.linalg.det(J_linear_manual)

    # ตรวจสอบว่า determinant ใกล้ศูนย์หรือไม่ (เพื่อบอกถึงสภาวะ Singular)
    if abs(epsilon - abs(det_J_linear_manual)) < difference:
        print('################# คำตอบข้อ 2 #################')
        print("Flag = 1: Singularity detected.")
        print('\n')
        return True  # Singular
    else:
        print('################# คำตอบข้อ 2 #################')
        print("Flag = 0: Not in singularity.")
        print('\n')
        return False  # Non-singular
    
#===========================================<คำตอบข้อ 3>====================================================#
# ฟังก์ชันสำหรับคำนวณแรงบิด (Torque) โดยใช้ Jacobian แบบ manual
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    print('################# คำตอบข้อ 3 #################')

    # คำนวณเมทริกซ์ Jacobian โดยใช้แบบ manual
    # J_e = endEffectorJacobianHW3(q)
    # คำนวณTranspose ของ เมทริกซ์ Jacobian
    J_e_T = J_e.transpose()
    # คำนวณแรงบิด
    tau = J_e_T @ w

    print('tau is \n', tau)
    print('\n')
    return tau


q = [0.0,0.0,0.0]                #<--- INSERT Q HERE !
# q = [0.0,-pi/2,-0.2]           # ค่าที่ทำให้เกิด Singularity
w = [10, 0, 0, 0, 0, 0]          #<---- Inset w Here !
# endEffectorJacobianHW3(q)
# checkSingularityHW3(q)
# computeEffortHW3(q,w)

# คำนวนหา Jacobian แค่ครั้งเดียวประกาศให้ใช้ตัวแปรได้ทุกฟังก์ชัน เพื่อลดการเกิด duplicate function calls จาก J_e = endEffectorJacobianHW3(q) ใน code
J_e = endEffectorJacobianHW3(q)
checkSingularityHW3(q)
computeEffortHW3(q, w)