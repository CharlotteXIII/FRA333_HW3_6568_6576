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

# กำหนดตัวแปรที่จะใช้งาน
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
# FK = rbot.fkine(q)
# Check rbot
# print(rbot)

#=============================================<ตรวจคำตอบข้อ 1>======================================================#
def TestJacobianHW3(q:list[float])->list[float]:
    print('################# ตรวจคำตอบข้อ 1 #################')

    J_e = rbot.jacob0(q) #คำนวณหา jacobian matrix โดยใช้ jacob0
    print("This RRR robot Jacobian is: \n",J_e) 

    print('\n')
    return J_e

#==============================================================================================================#


#=============================================<ตรวจคำตอบข้อ 2>======================================================#
def TestSingularityHW3(q:list[float])->bool:

    #กำหนดตัวแปร
    epsilon = 0.001
    J_e = rbot.jacob0(q)  

    # ใช้แค่ค่าของ Linear Velocity จึงทำการนำส่วนเชิงเส้นของ Jacobian matrix ออกมา โดยใช้ [:3, :]
    J_e_linear = J_e[:3, :]

    # หาค่า det ของ J_e_linear โดยใช้ numpy
    det_J_linear = np.linalg.det(J_e_linear)

    # สร้างตัวแปร Lเพื่อเก็บค่า absolute ของ det_J_linear
    L = abs(det_J_linear)

    # ตรวจสอบว่าหุ่นเข้าใกล้ค่า Singularity ไหม ผ่านการเปรียบเทียบกับ epsilon
    if abs(L) < epsilon:
        print('################# ตรวจคำตอบข้อ 2 #################')
        print(" Flag = 1 is singularity.")
        print('\n')
        return 1
    else:
        print('################# ตรวจคำตอบข้อ 2 #################')
        print(" Flag = 0 is not singularity.")
        print('\n')
        return 0
#==============================================================================================================#


#=============================================<ตรวจคำตอบข้อ 3>======================================================#
def TestcomputeEffortHW3(q:list[float], w:list[float])->list[float]:
    print('################# ตรวจคำตอบข้อ 3 #################')
    
    #กำหนดตัวแปร
    J_e = rbot.jacob0(q)

    # คำนวณแรงบิด
    #ใช้ numpy ในการหาค่า tau โดยนำ J_e.T dot w
    # tau = rbot.pay(w, q, J_e)
    tau = np.dot(J_e.T, w)
    print('tau is \n', tau)
    print('\n')
    return tau
    
#==============================================================================================================#
q = [0.0,0.0,0.0]                  #<--- INSERT Q HERE !
# q = [0.0,-pi/2,-0.2]             # ค่าที่ทำให้เกิด Singularity
w = [10, 0, 0, 0, 0, 0]            #<---- Inset w Here !
TestJacobianHW3(q)
TestSingularityHW3(q)
TestcomputeEffortHW3(q,w)




