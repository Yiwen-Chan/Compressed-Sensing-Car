#整合终极版
import time, mpu6050, network, usocket, pid
from machine import I2C, Pin, Timer
from pca9685 import PCA9685
from ssd1306 import SSD1306_I2C    #从ssd1306模块中导入SSD1306_I2C子模块
from speed import SPEED
from qmc import QMC5883L

#压缩感知数组，此数组由matlab生成的观测基A(64行128列)找“1”得到，数组里每12个数一组数字表示某一行第几列是1
B = [20, 34, 36, 55, 56, 60, 68, 73, 75, 78, 81, 87, 2, 7, 37, 38, 41, 44, 50, 60, 77, 98, 100, 111, 19, 28, 54, 57, 58, 67, 90, 101, 107, 110, 119, 125, 21, 32, 41, 48, 62, 75, 82, 92, 97, 100, 118, 121, 2, 5, 15, 23, 26, 33, 36, 46, 59, 66, 90, 127, 25, 37, 42, 44, 61, 66, 96, 97, 105, 107, 120, 124, 30, 38, 71, 76, 82, 85, 87, 108, 115, 119, 121, 127, 1, 21, 31, 33, 69, 78, 86, 105, 109, 113, 119, 128, 1, 11, 14, 15, 27, 30, 34, 35, 54, 79, 100, 107, 1, 7, 22, 31, 32, 36, 47, 53, 63, 70, 108, 112, 3, 10, 27, 33, 44, 63, 64, 71, 81, 101, 116, 117, 19, 42, 45, 49, 53, 58, 59, 61, 69, 106, 117, 123, 17, 34, 39, 46, 53, 62, 68, 70, 76, 77, 97, 119, 2, 15, 18, 29, 34, 42, 63, 71, 98, 105, 118, 121, 4, 20, 27, 66, 67, 68, 69, 80, 86, 94, 115, 118, 24, 27, 41, 43, 56, 72, 73, 82, 93, 97, 112, 122, 1, 18, 51, 64, 66, 76, 79, 82, 87, 99, 114, 123, 10, 16, 23, 45, 57, 64, 78, 100, 112, 120, 125, 127, 4, 14, 24, 31, 60, 61, 73, 88, 94, 101, 126, 127, 29, 67, 79, 81, 83, 89, 93, 95, 118, 123, 127, 128, 17, 23, 35, 40, 47, 66, 71, 88, 106, 111, 116, 121, 3, 15, 22, 60, 62, 69, 72, 74, 95, 104, 120, 121, 6, 36, 37, 47, 48, 56, 61, 64, 80, 95, 103, 119, 7, 13, 18, 23, 27, 46, 61, 76, 83, 85, 92, 104, 1, 16, 26, 50, 61, 70, 74, 75, 115, 116, 122, 7, 12, 16, 17, 33, 69, 71, 91, 93, 98, 107, 126, 24, 28, 35, 39, 59, 65, 81, 82, 86, 105, 108, 128, 11, 21, 52, 54, 59, 77, 79, 80, 101, 114, 120, 122, 5, 11, 13, 31, 39, 49, 50, 63, 80, 87, 107, 120, 9, 28, 37, 40, 55, 63, 84, 89, 91, 94, 100, 113, 114, 116, 13, 14, 18, 21, 24, 43, 47, 81, 91, 96, 115, 117, 4, 19, 26, 35, 52, 56, 63, 79, 91, 92, 96, 116, 6, 9, 25, 30, 39, 52, 60, 66, 85, 110, 112, 117, 12, 13, 30, 36, 40, 56, 65, 68, 74, 99, 101, 113, 3, 4, 9, 16, 20, 24, 31, 53, 62, 83, 99, 111, 5, 17, 19, 30, 43, 54, 60, 64, 86, 118, 124, 128, 5, 11, 20, 32, 48, 51, 58, 65, 70, 89, 104, 112, 4, 46, 57, 69, 77, 84, 87, 88, 89, 98, 118, 122, 15, 43, 44, 52, 70, 73, 83, 85, 86, 106, 113, 125, 9, 22, 38, 56, 59, 67, 70, 88, 91, 102, 124, 125, 23, 38, 39, 41, 42, 51, 54, 72, 74, 90, 92, 93, 1, 4, 8, 20, 29, 38, 40, 45, 97, 102, 110, 123, 8, 11, 23, 49, 64, 83, 84, 85, 94, 96, 102, 121, 26, 30, 32, 37, 49, 51, 52, 67, 73, 78, 111, 126, 13, 29, 42, 44, 48, 59, 62, 76, 84, 109, 112, 126, 6, 12, 29, 35, 43, 50, 53, 72, 78, 90, 104, 128, 3, 10, 47, 65, 68, 92, 98, 102, 110, 114, 115, 126, 128, 21, 34, 44, 45, 50, 79, 85, 88, 90, 99, 103, 6, 7, 14, 20, 37, 49, 57, 62, 82, 113, 116, 124, 3, 29, 46, 48, 49, 54, 75, 91, 103, 104, 108, 125, 2, 6, 9, 11, 12, 28, 45, 55, 76, 81, 92, 119, 122, 16, 21, 22, 26, 28, 40, 42, 46, 51, 94, 106, 110, 6, 8, 10, 12, 18, 19, 22, 33, 39, 73, 89, 111, 19, 25, 27, 50, 51, 77, 84, 88, 95, 108, 113, 5, 7, 8, 32, 47, 72, 78, 95, 99, 102, 106, 115, 8, 26, 45, 53, 86, 93, 98, 100, 101, 103, 109, 124, 2, 14, 17, 52, 58, 72, 75, 89, 99, 102, 107, 109, 10, 24, 65, 87, 90, 94, 95, 97, 109, 111, 117, 125, 2, 10, 55, 58, 74, 80, 83, 108, 110, 120, 124, 126, 3, 12, 25, 28, 34, 41, 80, 96, 104, 106, 109, 127, 5, 14, 16, 40, 41, 55, 67, 84, 103, 105, 114, 117, 8, 9, 36, 43, 48, 57, 71, 74, 77, 105, 123, 15, 17, 25, 31, 32, 38, 55, 65, 96, 103, 122, 123, 13, 18, 22, 25, 33, 35, 57, 58, 68, 75, 93, 114]

##############################################
#  硬件变量初始化
##############################################
key_node = 0    #按键标志位
##############################################
#  软件变量初始化
##############################################
q_buf = 0   #堆叠数据发送标记位
n_buf = 0   #堆叠数据组数计数
turn_buf = 0    #小车转向标记位（1为直走，2为左转，3为右转）
w_buf = 0   #wifi打开标记位
PWM_V = 2000    #pwm系数
##############################################
#  普通外设初始化
##############################################
#按键和LED初始化
LED = Pin(2, Pin.OUT) #构建LED对象,开始熄灭
KEY = Pin(0, Pin.IN, Pin.PULL_UP) #构建KEY对象
#巡线传感器初始化，五路光电
L1 = Pin(18, Pin.IN, Pin.PULL_UP)
L2 = Pin(19, Pin.IN, Pin.PULL_UP)
L3 = Pin(23, Pin.IN, Pin.PULL_UP)
L4 = Pin(5, Pin.IN, Pin.PULL_UP)
L5 = Pin(26, Pin.IN, Pin.PULL_UP)
#编码盘测速模块初始化，2-4路
S1 = Pin(22, Pin.IN, Pin.PULL_UP)
S2 = Pin(32, Pin.IN, Pin.PULL_UP)
S3 = Pin(36, Pin.IN, Pin.PULL_UP)
S4 = Pin(39, Pin.IN, Pin.PULL_UP)
count = 0
count2 = 0
count3 = 0
count4 = 0
##############################################
#  软件函数初始化
##############################################
#定时器参数初始化
set_period = 32    #定时器周期
set_freq = 128     #发送数据周期
#pid参数初始化
set_tar = 1.5      #pid直走默认目标值
set_tar1 = 3       #pid转弯默认目标值
set_kp = 2      #pid比例调节，主要影响上升速度
set_ki = 0         #pid积分调节，主要累计偏差
set_kd = 2         #pid微分调节，主要“阻尼”稳定
speed = [0, 2000, 2500, 3000, 3500, 4095] #默认速度档位控制，6档
#pid对象初始化
pid_motor_1 = pid.IncrementalPID(kp=set_kp, ki=set_ki, kd=set_kd, target=set_tar1)
pid_motor_2 = pid.IncrementalPID(kp=set_kp, ki=set_ki, kd=set_kd, target=set_tar1)
pid_motor_3 = pid.IncrementalPID(kp=set_kp, ki=set_ki, kd=set_kd, target=set_tar1)
pid_motor_4 = pid.IncrementalPID(kp=set_kp, ki=set_ki, kd=set_kd, target=set_tar1)
#数据处理初始化
arr_data = [([0]*16) for i in range(set_freq)]  #初始化叠加数据数组（128行16列）
cs_data = [([0]*16) for i in range(set_freq)]   #初始化叠加数据数组缓存（128行16列）
ccs_data = [([0]*16) for i in range(64)]        #初始化压缩感知压缩后数组（64行16列）
sp = SPEED(set_period, 1)   #初始化测速对象
speed_buf = [0, 0, 0, 0]    #输出速度初始化
#无压缩socket服务器初始化
o = usocket.socket()                #初始化socket服务对象
o_addr = ('192.168.137.1', 8000)     #服务器IP和端口
#压缩socket服务器初始化
s = usocket.socket()                #初始化socket服务对象
s_addr = ('192.168.137.1', 10000)     #服务器IP和端口
##############################################
#  I2C外设初始化
##############################################
#OLED初始化
i2c1 = I2C(sda=Pin(13), scl=Pin(14))         # sda-->13, scl-->14
oled = SSD1306_I2C(128, 64, i2c1, addr=0x3c) #OLED显示屏初始化：128*64分辨率,OLED的I2C地址是0x3c
oled.text("OLED OK", 0,  0)                  #写入第1行内容
oled.show()                                  #OLED执行显示
#电机控制模块初始化I2C，pca9685芯片
i2c2 = I2C(sda=Pin(16, Pin.OUT, Pin.PULL_UP), scl=Pin(17, Pin.OUT, Pin.PULL_UP), freq=40000)
pca = PCA9685(i2c2)                          #初始化PCA对象
pca.reset()                                  #复位pca
pca.freq(freq=1000)                          # set freq = 1000Hz
oled.text("PWM OK",  0, 20)                  #写入第2行内容
oled.show()                                  #OLED执行显示
#陀螺仪初始化
i2c3 = I2C(scl=Pin(15), sda=Pin(27))         #陀螺仪 scl->21   sda->33
accelerometer = mpu6050.accel(i2c3)          #初始化陀螺仪对象
#电子罗盘初始化
qmc = QMC5883L()                             #初始化电子罗盘对象

##############################################
#  服务函数
##############################################

def key(KEY):
    global key_node
    time.sleep_ms(10) #消除抖动
    if KEY.value()==0: #确认按键被按下
        key_node = 1

KEY.irq(key, Pin.IRQ_FALLING) #定义中断，下降沿触发
def fun1(S1):
    global count
    count = count+1
S1.irq(fun1, Pin.IRQ_FALLING) #定义中断，下降沿触发
def fun2(S2):
    global count2
    count2 = count2+1
S2.irq(fun2, Pin.IRQ_FALLING) #定义中断，下降沿触发
def fun3(S3):
    global count3
    count3 = count3+1
S3.irq(fun3, Pin.IRQ_FALLING) #定义中断，下降沿触发
def fun4(S4):
    global count4
    count4 = count4+1
S4.irq(fun4, Pin.IRQ_FALLING) #定义中断，下降沿触发

#小车控制函数
def pidc_forward():     #直走
    pid_motor_1.target(set_tar)
    pid_motor_2.target(set_tar)
    pid_motor_3.target(set_tar)
    pid_motor_4.target(set_tar)
    pca.duty(0, speed_buf[3])
    pca.duty(2, speed_buf[1])
    pca.duty(4, speed_buf[0])
    pca.duty(6, speed_buf[2])

    pca.duty(1, 0)
    pca.duty(3, 0)
    pca.duty(5, 0)
    pca.duty(7, 0)

def pidc_turn_left():        #左转
    pid_motor_1.target(set_tar1)
    pid_motor_2.target(set_tar1)
    pid_motor_3.target(set_tar1)
    pid_motor_4.target(set_tar1)
    pca.duty(0, 0)
    pca.duty(2, 0)
    pca.duty(4, speed_buf[0])
    pca.duty(6, speed_buf[2])

    pca.duty(1, speed_buf[3])
    pca.duty(3, speed_buf[1])
    pca.duty(5, 0)
    pca.duty(7, 0)

def pidc_turn_right():      #右转
    pid_motor_1.target(set_tar1)
    pid_motor_2.target(set_tar1)
    pid_motor_3.target(set_tar1)
    pid_motor_4.target(set_tar1)
    pca.duty(0, speed_buf[3])
    pca.duty(2, speed_buf[1])
    pca.duty(4, 0)
    pca.duty(6, 0)

    pca.duty(1, 0)
    pca.duty(3, 0)
    pca.duty(5, speed_buf[0])
    pca.duty(7, speed_buf[2])

#pid更新函数
def pid_update(speed_in):
    global speed_buf
    speed_buf[0] = int(pid_motor_1.update(speed_in[0])) * PWM_V
    speed_buf[1] = int(pid_motor_2.update(speed_in[1])) * PWM_V
    speed_buf[2] = int(pid_motor_3.update(speed_in[2])) * PWM_V
    speed_buf[3] = int(pid_motor_4.update(speed_in[3])) * PWM_V
    for p in range(4):
        if speed_buf[p] > 4095:
            speed_buf[p] = 4095
        elif speed_buf[p] < 0:
            speed_buf[p] = 0

#定时器函数装载
def timer_fun(tim):                 #定时器服务函数
    #数据获取
    global q_buf                    #发送堆叠数
    global arr_data                 #堆叠的数据数组
    if q_buf == set_freq :          #计数每隔set_freq压缩和发送一次
        q_buf = 0                   #标记为清零
    value_num = accelerometer.get_values()  #获取当前加速度计寄存器数据
    light = [L1.value(), L2.value(), L3.value(), L4.value(), L5.value()]    #获取当前五路红外值
    value_num1 = [value_num.get("GyZ"), value_num.get("AcZ"), value_num.get("AcY"), value_num.get("AcX")]   #获取当前加速度计的值
    speed_now = [sp.speed_update(0, count), sp.speed_update(1, count2), sp.speed_update(2, count3), sp.speed_update(3, count4)] #获取当前速度
    x, y, z, status, temp =qmc.read()   #获取当前电子罗盘寄存器数据
    qmc_now = [x, y]    #获取电子罗盘的值
    #堆叠数据
    for p in range(4):                      #将不同类型的数据整合成一个数组
        arr_data[q_buf][p] = value_num1[p]  #存入加速度计Z轴角速度，Z轴加速度，Y轴加速度，X轴加速度
    for p in range(2):
        arr_data[q_buf][p+4] = qmc_now[p]   #存入电子罗盘X轴磁场强度，Y轴磁场强度
    for p in range(4):
        arr_data[q_buf][p + 6] = speed_buf[p]#存入左前，左后，右前，右后轮子速度
    for p in range(5):
        arr_data[q_buf][p + 10] = light[p]  #存入五路红外值，从左到右
    for p in range(1):
        n_buf += 1
        arr_data[q_buf][p + 15] = n_buf   #存入校验位
    LED.value(0)
    q_buf = q_buf + 1

    #小车运动逻辑控制
    if L1.value()==0 or L2.value()==0:
        if L1.value()==0 or L2.value()==0:
            pidc_turn_left()
            while L4.value()==1 and L5.value()==1:
                if L3.value() == 0 and L1.value() == 1:
                    if L3.value() == 0 and L1.value() == 1:
                        break
            pidc_forward()
            turn_buf = 2

    elif L4.value()==0 or L5.value()==0 :
        if L4.value()==0 or L5.value()==0 :
            pidc_turn_right()
            while L1.value()==1 and L2.value()==1:
                if L3.value() == 0 and L5.value() == 1:
                    if L3.value() == 0 and L5.value() == 1:
                        break
            pidc_forward()
            turn_buf = 3

    else:
        pidc_forward()
    pid_update(speed_now)

#WIFI连接函数
def WIFI_Connect():
    WIFI_LED = Pin(2, Pin.OUT) #初始化WIFI指示灯
    wlan = network.WLAN(network.STA_IF) #STA模式
    wlan.active(True)                   #激活接口
    start_time = time.time()              #记录时间做超时判断
    if not wlan.isconnected():
        print('Connecting to network...')
        wlan.connect('even', '1234567809') #输入WIFI账号密码
        while not wlan.isconnected():  #连接不成功屏幕提示
            #LED闪烁提示
            WIFI_LED.value(1)
            time.sleep_ms(300)
            WIFI_LED.value(0)
            time.sleep_ms(300)
            #超时判断,15秒没连接成功判定为超时
            if time.time()-start_time > 15 :
                print('WIFI Connected Timeout!')
                break
    if wlan.isconnected():       #连接成功屏幕打印网关
        #LED点亮
        LED.value(1)
        #串口打印信息
        print('network information:', wlan.ifconfig())
        #OLED数据显示
        oled.fill(0)   #清屏背景黑色
        oled.text('IP/Subnet/GW:', 0, 0)
        oled.text(wlan.ifconfig()[0], 0, 20)
        oled.text(wlan.ifconfig()[1], 0, 38)
        oled.text(wlan.ifconfig()[2], 0, 56)
        oled.show()
        return True
    else:
        return False

if WIFI_Connect():
    #创建socket连接TCP类似，连接成功后发送“Hello 01Studio！”给服务器。
    o.connect(o_addr)
    s.connect(s_addr)
    LED.value(1)
    w_buf = 1
    #s.send('1')

if w_buf == 1:      #如果连接上WiFi，打开定时器，小车开动
    tim = Timer(0)
    tim.init(period=set_period, mode=Timer.PERIODIC, callback=timer_fun)

while True:
    if q_buf == set_freq:
        print('complete')
        cs_data = arr_data          #把定时器获取的数据存入缓存数组
        o.send(str(cs_data))        #发送原始数据数组
        for n in range(64):         #计算压缩后每行的数值
            for i in range(16):     #计算压缩后同一行的每个列的数值
                for j in range(12):
                    ccs_data[n][i] += cs_data[B[n*12+j]-1][i]  #A数组的行*数据数组的列
        s.send(str(ccs_data))       #发送压缩后的数据数组
        print('send complete')
        ccs_data = [([0]*16) for i in range(64)]    #清空压缩数据数组
        LED.value(1)
