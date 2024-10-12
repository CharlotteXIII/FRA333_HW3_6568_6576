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

#===========================================<คำตอบข้อ 1>====================================================#
def TestJacobianHW3(q:list[float])->list[float]:
    # Jacobian = np.zeros((6, 6))
    pass
#===========================================<คำตอบข้อ 2>====================================================#
def TestSingularityHW3(q:list[float])->bool:
    pass
    
#===========================================<คำตอบข้อ 3>====================================================#
def TestcomputeEffortHW3(q:list[float], w:list[float])->list[float]:
    pass


# q = [0.0,0.0,0.0] #<--- INSERT Q HERE !
# TestJacobianHW3(q)
# TestSingularityHW3(q)