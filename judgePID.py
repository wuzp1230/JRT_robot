#!/usr/bin/env python
# -*- coding: utf-8 -*-
import serial
import time
i=0
sersuc=0
while sersuc==0:
  try:
    port="/dev/ttyUSB"+str(i)
    print "Trying to open port "+str(i)
    sersuc=1
    ser=serial.Serial(port,115200) #Ubuntu
  except:
    print "Could not open port "+str(i)+",trying next port!"
    sersuc=0
    i=i+1
    time.sleep(0.5)
#############################################################
######################    变量声明    ########################
#############################################################
import threading
import numpy as np
from time import clock
#全局变量区
#ser=serial.Serial("/dev/ttyAMA0",115200) #RasPi
MAX=50
a=np.array([[0.5,0.5,0.5],[0.5,0.5,0.5],[0.5,0.5,0.5],[0.5,0.5,0.5]],dtype=np.float64)
#当前加速度，float值
w=np.array([[0.5,0.5,0.5],[0.5,0.5,0.5],[0.5,0.5,0.5],[0.5,0.5,0.5]],dtype=np.float64)
#当前角速度，float值
Angle=np.array([[0.5,0.5,0.5],[0.5,0.5,0.5],[0.5,0.5,0.5],[0.5,0.5,0.5]],dtype=np.float64)
#当前角度，float值
ET=np.array([[0.5,0.5,0.5,0.5],[0.5,0.5,0.5,0.5],[0.5,0.5,0.5,0.5],[0.5,0.5,0.5,0.5]],dtype=np.float64)#当前电机转速，float值
pwm=np.array([[0.5,0.5,0.5,0.5],[0.5,0.5,0.5,0.5],[0.5,0.5,0.5,0.5],[0.5,0.5,0.5,0.5]],dtype=np.float64)#控制量，float值，0-255
v=0#速度，int值

ia=np.array([0.0,0.0,0.0], dtype=np.float64)
iw=np.array([0.0,0.0,0.0], dtype=np.float64) #机身理想状态
iAngle=np.array([0.0,0.0,0.0], dtype=np.float64)
iET=np.array([0.0,0.0,0.0,0.0], dtype=np.float64)
iv=0

ea=np.array([0.0,0.0,0.0], dtype=np.float64)
ew=np.array([0.0,0.0,0.0], dtype=np.float64) #机身误差状态
eAngle=np.array([0.0,0.0,0.0], dtype=np.float64)
eET=np.array([0.0,0.0,0.0,0.0], dtype=np.float64)
ev=0

#供fr等参数使用的速度、时间等参数
spd=0
timex=0
processes=0 #进程计数器，所有进程全部启动之后弹出命令行
FR_BOUND=158 #角度界限最大值
BK_BOUND=-174
herror = 999 #误差历史
#sigmoids
IDEAL_AVAIL = False	#机身理想状态
RUNNING = True		#是否停止
KEY_AVAIL=False         #键盘数据是否可用
SENDING=False           #正在发送

MOTOR_OUT_MAX = 1.0
MOTOR_OUT_MIN = -1.0
ANGLE_CONTROL_OUT_MAX = MOTOR_OUT_MAX*10
ANGLE_CONTROL_OUT_MIN = MOTOR_OUT_MIN*10
g_fSpeedControlOutOld = 0.0
g_fSpeedControlOutNew = 0.0
g_fDirectionControlOutOld = 0.0
g_fDirectionControlOutNew = 0.0
g_nSpeedControlPeriod = 0.0
g_nDirectionControlPeriod = 0.0
SPEED_CONTROL_PERIOD = 1.0
DIRECTION_CONTROL_PERIOD = 1.0
DIRECTION_CONTROL_OUT_MAX = MOTOR_OUT_MAX*10  
DIRECTION_CONTROL_OUT_MIN = MOTOR_OUT_MIN*10
nPeriod = 1000  #timer ARR值
times=0

#主程序入口
def main():
  #start0.Init()
  processes=0
  thread1=start1()
  thread2=NN1()
  thread3=monitK()
  thread1.start()
  thread2.start()
  thread3.start()
  thread1.join()
  thread2.join()
  thread3.join()
################################################################
#####################    神经网络类    #########################
################################################################
#father class is threading.Thread

class NN1(threading.Thread):  #神经网络的线程类
  #Netual Network
  def __init__(self):
    threading.Thread.__init__(self)
  def run(self):
    global processes
    processes=processes+1
    #NNmain()

#########################################################################
#################    monitSerial 监视端口的线程类    ########### OK ###### 
#########################################################################     
class start1(threading.Thread):   
  def __init__(self):
    threading.Thread.__init__(self)
  def run(self):
    global processes #referece global variable
    processes=processes+1
    monitSerial()

def monitSerial():
  print 'monitSerialmain()\n'
  global KEY_AVAIL
  global IDEAL_AVAIL
  global Recv_queue
  while 1:
    #print 'monitSerial loop()\n'
    #if IDEAL_AVAIL==False:# and RUNNING == True:
    #  splitnum() #调试快捷入口
    down_recv=ser.readline()      #读端口
    #print 'readline done()\n'
    if len(down_recv) > 0:
      checksum(down_recv)

def checksum(message):
  """计算校验值"""
  global RECV  #下位机是否接到
  global Recv_queue
  global scale  
  try:
    numbers=message.split(';')#分开数据和校验码
    #print str(numbers[0])
    data=numbers[0]#data为数据主体
    check_code=numbers[1]#check_code为校验码
    check_int=int(check_code)
  except:
    print "Exception!"
  else:
    check=0
    #print(data)
    for letter in data:
      check+=ord(letter)
    #print(check)
    if (check == check_int):#收到消息正确
      splitnum(numbers[0])
    print "Success!"
     
def splitnum(data):
  """分开、存储数据"""
  global IDEAL_AVAIL
  global ANGLE_CONTROL_P  #角度控制比例
  global ANGLE_CONTROL_BIGP #小角度的比例控制
  global ANGLE_CONTROL_D  #角度控制微分，角速度
  global SPEED_CONTROL_P  #速度控制比例
  global DIR_CONTROL_P
  global DIR_CONTROL_D
  global CAR_SPEED_SET #理想速度
  global CAR_DIRECTION_SET
  global ANGLE_CONTROL_OUT_MAX #角度控制最大值
  global ANGLE_CONTROL_OUT_MIN #角度控制最小值
  global g_fDirectionControlOutOld
  global g_fDirectionControlOutNew
  global g_nSpeedControlPeriod
  global g_nDirectionControlPeriod
  global SPEED_CONTROL_PERIOD
  global DIRECTION_CONTROL_PERIOD
  global DIRECTION_CONTROL_OUT_MAX
  global DIRECTION_CONTROL_OUT_MIN

  global a
  global w
  global Angle
  global ET
  global send_data
  global IDEAL_AVAIL
  global times

  #temp_input0=np.hstack((a[0:3,:].reshape(1,9),w[0:3,:].reshape(1,9),Angle[0:3,:].reshape(1,9),ET[0:3,:].reshape(1,12),pwm[1:4,:].reshape(1,12))).ravel()
  #numbers = net2.activate(temp_input0)#模拟从下位机返回的状态
  #激活用来模拟系统的网络：历史机身状态 + pwm历史+pwm实时
  try:
    #print data
    numbers=data.split(',')
    data0=float(numbers[0])#加速度
    data1=float(numbers[1])
    data2=float(numbers[2])
    data3=float(numbers[3])#角速度
    data4=float(numbers[4])
    data5=float(numbers[5])
    data6=float(numbers[6])#角度
    if data6<0:
      data6=-data6-180  #小于0  逐渐接近0
    else:
      data6=-data6+180  #大于0  逐渐接近0
    data6=data6-7.9
    data7=float(numbers[7])
    data8=float(numbers[8])
    data9=float( numbers[9])#电机转速
    data10=float(numbers[10] )
    data11=float(numbers[11] )
    data12=float(numbers[12])
  except:
    print "Exception!"
  else:
    a[3,]=[data0,data1,data2]#加速度
    w[3,]=[data3,data4,data5]#角速度
    Angle[3,]=[data6,data7,data8]#角度
    ET[3,]=[data9,data10,data11,data12]#电机转速
    print "Success!"
  #finally:
      #always_execute_suite
    """if data6<0:
      data6=data6*0.75"""
  ANGLE_CONTROL_P=0.0135
  ANGLE_CONTROL_D=-0.00035

  ANGLE_CONTROL_BIGP=0.0073
  ANGLE_CONTROL_BIGD=-0.0015
  SPEED_CONTROL_P=1
  DIR_CONTROL_P=1.0
  DIR_CONTROL_D=0
  CAR_SPEED_SET=0
  CAR_DIRECTION_SET=0
  """print "\nAcc:"+str(a[3,])#str(data0)+" "+str(data1)+" "+str(data2)
  print "\nAngSpd"+str(w[3,])#str(data3)+" "+str(data4)+" "+str(data5)
  print "\nAngle"+str(Angle[3,])#str(data6)+" "+str(data7)+" "+str(data8)
  print "\nET"+str(ET[3,])+"\n"#str(data9)+" "+str(data10)+" "+str(data11)+" "+str(data12)"""
  """file_log.write("\nAcc:"+str(a[3,])*16)#str(data0)+" "+str(data1)+" "+str(data2)
  file_log.write("\nAngSpd"+str(w[3,]*2000))#str(data3)+" "+str(data4)+" "+str(data5)
  file_log.write("\nAngle"+str(Angle[3,])*180)#str(data6)+" "+str(data7)+" "+str(data8)
  file_log.write("\nET"+str(ET[3,])+"\n")#str(data9)+" "+str(data10)+" "+str(data11)+" "+str(data12)"""
 #/////////////  平衡角度控制  /////////////
  fValue=0.0
  g_fAngleControlOut=0.0

  fP = 0.0
  fDelta = 0.0
  fI = 0.0
  g_fCarSpeed = 0.0
  fValue = 0.0
  speed_bias=0.0

#-------------------------------  角度控制  -----------------------------
  if (Angle[3,0]<=-4.0) or (Angle[3,0]>=4.0): #大角度的时候
    print "BIGANGLE\n"+"Angle="+ str(Angle[3,0]) + " Gryo=" + str(w[3,0]) + " ET" + str(ET[3,0]) +" "+ str(ET[3,1])
    fValue=Angle[3,0]* ANGLE_CONTROL_P + w[3,0] * ANGLE_CONTROL_D
    #2 =20 × 0.07 + 5 × 0.00016
    #平衡角度×比例 +角速度×平衡角度微分
  else: #小角度的时候
    print "SMLANGLE\n"+"Angle="+ str(Angle[3,0]) + " Gryo=" + str(w[3,0]) + " ET" + str(ET[3,0]) +" "+ str(ET[3,1])
    fValue=(Angle[3,0])*abs(Angle[3,0])*ANGLE_CONTROL_BIGP+w[3,0] * ANGLE_CONTROL_BIGD
    #小角度，角度平方×比例控制 + 角速度 × 0.15（微分控制）
  if fValue > ANGLE_CONTROL_OUT_MAX:
    fValue = ANGLE_CONTROL_OUT_MAX
  elif fValue < ANGLE_CONTROL_OUT_MIN:
    fValue = ANGLE_CONTROL_OUT_MIN
  g_fAngleControlOut = 0.0
  g_fAngleControlOut = fValue  #最终平衡角度输出
  print "AngleOut =" + str(g_fAngleControlOut)

#-------------------------------  速度控制  -----------------------------
  g_fSpeedControlOut = 0.0
  """g_fSpeedControlOutNew = 0.0
  g_fCarSpeed = ( ET[3,0]+ET[3,1] )/2 #计算速度，码盘/时间
  fDelta = CAR_SPEED_SET #设定的速度
  fDelta = fDelta - g_fCarSpeed  #实际速度和理想速度的速度差
  fP = fDelta * SPEED_CONTROL_P  #速度的比例
  g_fSpeedControlOutOld = g_fSpeedControlOutNew  #老速度
  g_fSpeedControlOutNew = fP  #新速度，实际等于fP

#-------------------------  速度控制的输出  ---------------------
  fValue = g_fSpeedControlOutNew - g_fSpeedControlOutOld+speed_bias
  g_nSpeedControlPeriod = clock() - g_nSpeedControlPeriod
  #  g_fSpeedControlOut = fValue *(0.5- (g_nSpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD )+ g_fSpeedControlOutOld

  g_nSpeedControlPeriod = clock()"""
  #print "SpeedOut =" + str(g_fSpeedControlOut)
  #最终速度输出值=（新速度值-老速度值）×（0.5-（速度控制区间+1）/速度区间归一化参数）+老速度输出值
#----------------------------  方向控制   ------------------------
  """temp=0.0
  fValue=0.0
  g_fDirectionControlOutOld = g_fDirectionControlOutNew
  temp=Angle[3,2]-CAR_DIRECTION_SET  #继续要转向的角度=当前角度-设定角度
  fValue = temp * DIR_CONTROL_P/180-DIR_CONTROL_D*(w[3,2])
  #新角度=要转向的角度×比例/180 - 微分×要转向的角速度
  if fValue > DIRECTION_CONTROL_OUT_MAX:
    fValue = DIRECTION_CONTROL_OUT_MAX
  if fValue < DIRECTION_CONTROL_OUT_MIN:
    fValue = DIRECTION_CONTROL_OUT_MIN
  g_fDirectionControlOutNew = fValue
  fValue = g_fDirectionControlOutNew - g_fDirectionControlOutOld"""
  g_fDirectionControlOut=0.0
  """g_nSpeedControlPeriod = clock() - g_nSpeedControlPeriod
  g_fDirectionControlOut = fValue * (g_nDirectionControlPeriod + 1) /DIRECTION_CONTROL_PERIOD + g_fDirectionControlOutOld
  g_nSpeedControlPeriod = clock()
  print "DirectionOut =" + str(g_fDirectionControlOut)"""
   #最终角速度输出=（新角度-老角度）×（角度控制时间差+1）/角度控制时间差归一化参数 + 老角速度输出
#------------------------------------------------------------------------------
  fLeft = 0.0
  fRight = 0.0
  fLeft = g_fAngleControlOut - g_fSpeedControlOut - g_fDirectionControlOut
  fRight = g_fAngleControlOut - g_fSpeedControlOut + g_fDirectionControlOut
  print "fLeft is" + str(fLeft) + "fRight is" + str(fRight)
  # 平衡角度所需速度  -  速度  -  方向（角速度）
  if fLeft > MOTOR_OUT_MAX:
    fLeft = MOTOR_OUT_MAX
  elif fLeft < MOTOR_OUT_MIN:
    fLeft = MOTOR_OUT_MIN
  if fRight > MOTOR_OUT_MAX:
    fRight = MOTOR_OUT_MAX
  elif fRight < MOTOR_OUT_MIN:
    fRight = MOTOR_OUT_MIN

#------------------------------------------------------------------------------
  fTail=0.0
  fTail=1.42 * (fLeft+fRight)/2
  fTail=fTail+0.040
  if times%90==20:
    tail1=-50
  elif times%90==23:
    tail1=100
  elif times%90==26:
    tail1=-50
  else:
    tail1=0
  times=times+1
  all_pwm=str(int(255*fLeft))+','+str(int(255*fRight))+','+str(tail1)+','+str(int(255*fTail))
  checkpwm=0
  for letter in all_pwm:
    checkpwm+=ord(letter)
  #print "Checkpwm is" + str(checkpwm)
  send_data=all_pwm+';'+str(checkpwm)
  IDEAL_AVAIL=True
  if IDEAL_AVAIL:
    ser.write(send_data)
    print send_data
    IDEAL_AVAIL=False

  #发送完成

################################################################
#################    monitK监视键盘的类    #####################
################################################################
class monitK(threading.Thread):
  #monit Keyboard
  def __init__(self):
    #self.thread_stop=False;
    threading.Thread.__init__(self)
  def run(self):
    global processes
    processes=processes+1
    #monitKey()  #monitK的函数，检测键盘输入用
    
def monitKey():
  global key_input
  global KEY_AVAIL
  global processes
  global FR_BOUND
  global BK_BOUND
  print "monitKey()_type quit to quit\n"
  while key_input != "quit":
    if processes == 3:
      if Angle[3,0] > FR_BOUND or Angle[3,0] < BK_BOUND:#机身姿态越界
        FALLDOWN=True
        sendtoDown("0,0,0,0")
        while key_input!= "s":
          key_input=raw_input("Fell down,insert \"s\" to begin again>>")
          sendtoDown("0,0,0,0")
        FALLDOWN=False
      if 1:#RUNNING == False:
        key_input=raw_input(">>please input:")
        KEY_AVAIL=True
        judgeK()   #分析输入
        time.sleep(0.1)
        #print ("judgeK Over") 
        #print ("RUNNING:") +str(RUNNING)

def judgeK():
  global key_input
  global KEY_AVAIL  
  print("judgeK!")
  #keyinput()
  #print(key_input)
  keyvalue=key_input.split(',')
  KEY_AVAIL=False
  length=len(keyvalue)
  #print(length)
  control = keyvalue[0]
  cont1=Control() #类声明
  if control == 'fr':
    control1 = keyvalue[1]
    num1 = int(control1)
    control2 = keyvalue[2]
    num2 = int(control2)
    cont1.fr(num1,num2)#spd scope is +-255,time is second,all integer,
    #input should be checked
  elif control == 'bk':
    control1 = keyvalue[1]
    num1 = int(control1)
    control2 = keyvalue[2]
    num2 = int(control2)
    cont1.bk(num1,num2)      
  elif control == 'lt':
    control1 = keyvalue[1]
    num1 = int(control1)
    cont1.lt(num1)
  elif control == 'rt':
    control1 = keyvalue[1]
    num1 = int(control1)
    cont1.rt(num1)
  elif control == 'rd':
    control1 = keyvalue[1]
    num1 = int(control1)
    control2 = keyvalue[2]
    num2 = int(control2)
    cont1.rd(num1,num2)      
  #all below might add '\0'
  elif control == 'stop':
    cont1.stop0()
    #stop all actions and stay balanced
  elif control=='quit':
    return
    #quit and poweroff

  elif control=='studyon': 
    studyOn()
    #study mode on
  elif control=='studyoff':
    studyOff()
    #study mode off
  elif control=='autosave':
    autoSave()
    #auto save matrix
  elif control=='autosaveoff':
    autoSaveoff()
    #auto save samples off
  elif control=='save':
    Save()
    #save matrix manually
  elif control=='load':
    load0()
    #load matrix

  elif control=='saveD':
    saveD()
    #auto save samples
  elif control=='loadD':
    loadD()
    #load samples
  elif control=='offD':
    offD()
    #auto save samples off
  elif control=='setsavetime':
    setST(num1)
    #set matrix auto save time 
  else:
    print ("Input false")
####################################################################
##################    Control控制动作类    ##########################
####################################################################
class Control(object):

  def __init__(self):
    print "Control init!\n"

  def timer(self,time0):
    #start0.main()
    global RUNNING
    print "Control.timer!\n"
    while IDEAL_AVAIL == True:
      RUNNING = True
    
    if int(time0) != 0:
      time.sleep(int(time0))
    RUNNING = False

  def fr(self,spd0,time0):
    global IDEAL_AVAIL
    global RUNNING
    global iAngle
    global iw
    global ia
    global iET
    global timex
    spd = 0.0
    spd = spd0
    timex = time0
    ia=(0.0,0.0,0.0)
    iAngle=(0.0,0.0,Angle[3,3])
    iw=(0.0,0.0,0.0)
    iET=(spd,spd,0.0,0.0)
    RUNNING = True
    print "Control.fr!\n"
    #t1 = threading.Thread(target=self.timer,args=(timex,))#计时器
    #t1 = threading.Thread(target=self.timer,args=(0.1,))
    #t1.start()
    time.sleep(timex)
    #RUNNING=False
    self.stop0()

  def bk(self,spd0,time0):
    global IDEAL_AVAIL
    global RUNNING
    global iAngle
    global iw
    global ia
    global iET
    global timex
    spd = 0.0
    spd = spd0
    timex = time0
    ia=(0.0,0.0,0.0)
    iAngle=(0.0,0.0,Angle[3,3])
    iw=(0.0,0.0,0.0)
    iET=(-spd,-spd,0.0,0.0)
    RUNNING = True
    print "Control.bk!\n"
    #t1 = threading.Thread(target=self.timer,args=(timex,))#计时器
    #t1 = threading.Thread(target=self.timer,args=(0.1,))
    #t1.start()
    time.sleep(timex)
    #RUNNING=False
    self.stop0()

  def stop0(self):
    global IDEAL_AVAIL
    global RUNNING
    global iAngle
    global iw
    global ia
    global iET
    global timex
    spd = 0.0
    spd = spd0
    timex = time0
    ia=(0.0,0.0,0.0)
    iAngle=(0.0,0.0,Angle[3,3])
    iw=(0.0,0.0,0.0)
    iET=(0.0,0.0,0.0,0.0)
    RUNNING = True
    print "Control.stop!\n"
    #t1 = threading.Thread(target=self.timer,args=(timex,))#计时器
    #t1 = threading.Thread(target=self.timer,args=(0.1,))
    #t1.start()
    #RUNNING=False

  def lt(self,Angle0): #Angle you want to turn
    global IDEAL_AVAIL
    global RUNNING
    global iAngle
    global iw
    global iv
    iv = 0.0
    #iAngle[0]=0.0
    #iAngle[1]=0.0    
    AngleT = 0.0
    AngleT=Angle[2] #current AngleZ
    #iAngle[2]=AngleT + Angle0 #Angle division
    iAngle=(0.0,0.0,AngleT + Angle0)
    #iw[0]=0.0
    #iw[1]=0.0
    #iw[2]=0.0
    iw=(0.0,0.0,0.0)
    IDEAL_AVAIL = True
    RUNNING = True
    t1 = threading.Thread(target=self.timer,args=(0.1,))
    t1.start()
    print "Control.lt!\n"
    time.sleep(timex)
    #RUNNING=False

  def rt(self,Angle0): #Angle you want to turn
    global IDEAL_AVAIL
    global RUNNING
    global iAngle
    global iw
    global iv
    iv = 0.0
    #iAngle[0]=0.0
    #iAngle[1]=0.0
    AngleT = 0.0
    AngleT=Angle[2] #current AngleZ
    #iAngle[2]=AngleT - Angle0 #Angle division
    iAngle=(0.0,0.0,AngleT - Angle0)
    #iw[0]=0.0
    #iw[1]=0.0
    #iw[2]=0.0
    iw=(0.0,0.0,0.0)
    IDEAL_AVAIL = True
    RUNNING = True
    t1 = threading.Thread(target=self.timer,args=(0.1,))
    t1.start()
    print "Control.rt!\n"
    time.sleep(timex)
    #RUNNING=False

  def rd(self,spd0,time0):   
    global IDEAL_AVAIL
    global RUNNING
    global iAngle
    global iw
    global iv
    iv = 0.0
    #iAngle[0]=0.0
    #iAngle[1]=0.0
    #iAngle[2]=Angle[2]
    #iw[0]=0.0
    #iw[1]=0.0
    #iw[2] = spd
    iAngle=(0.0,0.0,Angle[2])
    iw=(0.0,0.0,spd0)
    IDEAL_AVAIL = True
    RUNNING = True
    #t1 = threading.Thread(target=self.timer,args=(time0,))
    #t1 = threading.Thread(target=self.timer,args=(time0,))
    #t1.start()
    print "Control.rd!\n"
    time.sleep(timex)
    #RUNNING=False

  def stop0(self):
    global IDEAL_AVAIL
    global RUNNING
    global iAngle
    global iw
    global iv
    iv=0.0
    #iAngle[0]=0.0
    #iAngle[1]=0.0
    #iAngle[2]=0.0
    #iw[0]=0.0
    #iw[1]=0.0
    #iw[2]=0.0
    iAngle=(0.0,0.0,0.0)
    iw=(0.0,0.0,0.0)
    IDEAL_AVAIL = True
    #RUNNING = True
    #t1 = threading.Thread(target=self.timer,args=(0,))
    #t1.start()
    print "Control.stop!\n"
    #RUNNING=False
    
########这些是函数的声明吗？python函数定义了就可以用的，不用特别声明。这里函数体不能为空。
#我先注释掉
#poweroff
  #def studoon():  
#study mode on,STUDY_ON is true
  #def studyoff()：
  
  #def save0()：
  
  #def saveoff():
    
  #def load0():
    
  #def saveD():
    
  #def autoon():
    
  #def autooff():
    
  #def setST():
    
#save_matrix_time is gonna be modified here
"""def refresh(result):   
  global IDEAL_AVAIL
  global a
  global w
  global Angle
  global ET
  global pwm  
  a[0,]=a[1,]
  a[1,]=a[2,]
  a[2,]=a[3,]
  a[3,]=[result[0],result[1],result[2]]#加速度
  w[0,]=w[1,]
  w[1,]=w[2,]
  w[2,]=w[3,]
  w[3,]=[result[3],result[4],result[5]]#角速度
  Angle[0,]=Angle[1,]
  Angle[1,]=Angle[2,]
  Angle[2,]=Angle[3,]
  Angle[3,]=[result[6],result[7],result[8]]#角度

  ET[0,]=ET[1,]
  ET[1,]=ET[2,]
  ET[2,]=ET[3,]
  ET[3,]=[result[9],result[10],result[11],result[12]]#电机转速
  IDEAL_AVAIL = True"""

########################################################
if __name__ == '__main__':
  try:
    main()
  except KeyboardInterrupt as e:
    print("exit!" + e)
