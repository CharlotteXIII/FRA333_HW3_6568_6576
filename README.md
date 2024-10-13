# FRA333_HW3_6568_6576

ชื่อ_รหัส

1.ชุตินันท์_6568

2.สิปปนนท์_6576

```
#NOTE ระวังกด Run แล้วบัคแสดงออกมาไม่ครบ 6x6 matrix ให้กด Run ใหม่อีกรอบ
```

# การตรวจคำตอบบนไฟล์ testScript.py

## Table of Content

> 0. วิธีการใช้งานโค้ดทั้ง 2 ไฟล์
> 1. แนวคิดการตรวจคำตอบ
> 2. วิธีการตรวจคำตอบ
> 3. ตรวจคำตอบข้อที่ 1
> 4. ตรวจคำตอบข้อที่ 2
> 5. ตรวจคำตอบข้อที่ 3
-----------------------------------------

### 0. วิธีการใช้งาน Code ทั้ง 2 ไฟล์

โค้ดของ FRA333_HW3_6568_6576.py และ testScript.py 

จะมีการกำหนดค่า q และ w ที่อยู่บริเวณด้านท้ายสุดของ Code 

>ใน FRA333_HW3_6568_6576.py โค้ดจะอยู่บริเวณบรรทัดที่ 121 ถึง 123 ผู้ใช้โค้ดสามารถปรับแก้ได้ตามความสนใจ

>ใน testScript.py โค้ดจะอยู่บริเวณบรรทัดที่ 110 ถึง 112 ผู้ใช้โค้ดสามารถปรับแก้ได้ตามความสนใจ
 ```
q = [0.0,0.0,0.0]                #<--- INSERT Q HERE !
# q = [0.0,-pi/2,-0.2]           # ค่าที่ทำให้เกิด Singularity
w = [10, 0, 0, 0, 0, 0]          #<---- Inset w Here !
  ```
จากนั้นกด Run โค้ดได้เลย ผลลัพธ์ที่ได้จะเป็นดังตัวอย่างนี้ 

```
This RRR robot Jacobian(manual calculation) is:    
 [[-1.09000000e-01 -9.30000000e-02 -9.30000000e-02]
 [ 8.99430000e-01  6.64634217e-17  4.04396760e-17] 
 [ 0.00000000e+00 -8.99430000e-01 -4.74430000e-01] 
 [ 0.00000000e+00  1.22464685e-16  1.22464685e-16] 
 [ 0.00000000e+00  1.00000000e+00  1.00000000e+00] 
 [ 1.00000000e+00  6.12323426e-17  6.12323426e-17]]


################# คำตอบข้อ 2 #################     
Flag = 0: Not in singularity.


################# คำตอบข้อ 3 #################     
tau is
 [-1.09 -0.93 -0.93]
```


### 1. แนวคิดการตรวจคำตอบ

ส่วนของการตรวจคำตอบในไฟล์ testScript.py คือการตรวจโดยใช้ roboticstoolbox ในการคำนวนปัญหาของโจทย์ 

และเมื่อคำนวนเสร็จสิ้นให้นำไปเทียบกับ คำตอบที่อยู่ในไฟล์ FRA333_HW3_6568_6576.py 

ตรวจสอบว่าคำตอบใน FRA333_HW3_6568_6576.py ถูกหรือไม่

### 2. วิธีการตรวจคำตอบ



