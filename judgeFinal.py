#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""(C) Copyright © 2016 wuzp1230. All rights reserved. ********************
* File Name          : judge.py
* Author             : wuzp1230
* Version            : V1.0
* Date               : 04/06/2016
* Description        : This file provides all the calculation functions,
*                      the heart of the Raspberry Pi and the brain of the car. 
****************************************************************************"""
#############################################################
################    Serial Open    ##########################
#############################################################
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
##################    Linklist    ###########################
#############################################################
    #单链表的实现
#节点类 
class Node(object):
     #初始化节点
    def __init__(self,val,p=0):   
        self.data = val
        self.next = p

class LinkList(object):
  #链表类
    def __init__(self):
        self.head = 0
    #返回指定节点
    def __getitem__(self, key):
        if self.is_empty():
            print "linklist is empty."
            return
        elif key <0  or key > self.getlength():
            print "the given key is error"
            return
        else:
            return self.getitem(key)
   #设置指定节点
    def __setitem__(self, key, value):
        if self.is_empty():
            print "linklist is empty."
            return
        elif key <0  or key > self.getlength():
            print "the given key is error"
            return
        else:
            self.delete(key)
            return self.insert(key)
   #初始化链表
    def initlist(self,data):
        self.head = Node(data[0])
        p = self.head
        for i in data[1:]:
            node = Node(i)
            p.next = node
            p = p.next
   #获得链表长度
    def getlength(self):
        p =  self.head
        length = 0
        while p!=0:
            length+=1
            p = p.next
        return length

  #判断链表是否为空
    def is_empty(self):
        if self.getlength() ==0:
            return True
        else:
            return False
  #清除链表 
    def clear(self):
        self.head = 0

    def append(self,item):
        q = Node(item)
        if self.head ==0:
            self.head = q
        else:
            p = self.head
            while p.next!=0:
                p = p.next
            p.next = q
  #取链表中的内容
    def getitem(self,index):
        if self.is_empty():
            print "Linklist is empty."
            return
        j = 0
        p = self.head
        while p.next!=0 and j <index:
            p = p.next
            j+=1
        if j ==index:
            return p.data
        else:
            print "target is not exist!"
  #插入内容         
    def insert(self,index,item):
        if self.is_empty() or index<0 or index >self.getlength():
            print "Linklist is empty."
            return
        if index ==0:
            q = Node(item,self.head)
            self.head = q
        p = self.head
        post  = self.head
        j = 0
        while p.next!=0 and j<index:
            post = p
            p = p.next
            j+=1
        if index ==j:
            q = Node(item,p)
            post.next = q
            q.next = p
  #删除内容
    def delete(self,index):
        if self.is_empty() or index<0 or index >self.getlength():
            print "Linklist is empty."
            return
        if index ==0:
            q =self.head
            p=q.next
            self.head = p
            return 
        p = self.head
        post  = self.head
        j = 0
        while p.next!=0 and j<index:
            post = p
            p = p.next
            j+=1
        if index ==j:
            post.next = p.next
  #查找内容
    def index(self,value):
        if self.is_empty():
            print "Linklist is empty."
            return
        p = self.head
        i = 0
        while p.next!=0 and not p.data ==value:
            p = p.next
            i+=1
        if p.data == value:
            return i
        else:
            return -1
#链表类的实现
#############################################################
##############   Variables Definition    ####################
#############################################################
import time
import threading
import numpy as np
from pybrain.tools.shortcuts import buildNetwork
from pybrain.datasets import SupervisedDataSet
from pybrain.supervised.trainers import BackpropTrainer
from pybrain.structure.modules import TanhLayer
from pybrain.structure.modules import LinearLayer

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
save_matrix_time=15#second
n=0.01
train_time=1 #训练次数
FR_BOUND=10 #角度界限最大值
BK_BOUND=-26
hAngle = 999 #误差历史
#sigmoids
VAL_AVAIL=False
DOWN_AVAIL=True
FALLDOWN = False
SND_AVAIL=False #whether send queue is avaliable
STUDY_ON=False          #在线学习状态开关
IDEAL_AVAIL = False	#机身理想状态
RUNNING = True		#是否停止
KEY_AVAIL=False         #键盘数据是否可用
SENDING=False           #正在发送
RECV=False              #Recv或者Eror抢夺
NN_READY=True           #神经网络准备好读数据
trained = False         #神经网络训练状态
key_input=' ' #键盘指令
down_recv=' ' #缓冲区数据
send_data=' ' #缓冲区数据
logtime = str(time.strftime('%Y_%m_%d__%H_%M_%S'))
file_log = open(logtime+".txt", 'w+') #打开文件写
file_log.write("\n\nTime:"+logtime+"\n\n")
#print time.strftime('%Y-%m-%d %H:%M:%S')

net1 = buildNetwork(51, 52, 13, hiddenclass=TanhLayer, outclass=LinearLayer)
#net1为要学习的神经网络，输入为3组历史状态+3组控制量，输出为一组预测的机身状态

#二维数组声明用
Recv_queue=LinkList()
Send_queue=LinkList()
#队列常用命令：
#append(String)#向队尾追加一条数据
#getitem(Position)#获取指定位置的数据
#delete(Position)#删除指定位置的数据
#############################################################
################    Main Function    ########################
#############################################################
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
#############################################################
###############    Neural Network    ########################
#############################################################
#father class is threading.Thread

class NN1(threading.Thread):  #神经网络的线程类
  #Netual Network
  def __init__(self):
    threading.Thread.__init__(self)
  def run(self):
    global processes
    processes=processes+1
    NNmain()

def NNmain():
  global IDEAL_AVAIL
  global RUNNING
  global FALLDOWN
  while 1:
    if IDEAL_AVAIL and FALLDOWN==False :#and RUNNING :
      NN_calc()
      
def NN_calc():
  global IDEAL_AVAIL
  global net1 #NN Network
  #global net2 #System
  global a
  global w
  global Angle
  global ET
  global pwm
  global n #n为学习率
  global trained
  global train_time
  global file_log
  global scale
  global hAngle
#  ------    神经网络学习系统姿态过程 ------------------- #
#  ------    Neural Networl Learning Body Status -------- #
  #系统输入   
  sys_input0=np.hstack((a[0:3,:].reshape(1,9),w[0:3,:].reshape(1,9),Angle[0:3,:].reshape(1,9),ET[0:3,:].reshape(1,12),pwm[1:4,:].reshape(1,12))).reshape(1,51)#系统输入,1*51
  #系统输出，实际状态，1*13
  sys_output0=np.hstack((a[3,:],w[3,:],Angle[3,:],ET[3,:])).reshape(1,13)
  dataSet = SupervisedDataSet(51, 13)
  dataSet.addSample((sys_input0),(sys_output0)) #学习实际机身状态
  trainer = BackpropTrainer(net1, dataSet,learningrate=n)
  trained = False
  acceptableError = 0.001
  # train until acceptable error reached
  error = trainer.train()
  train_time=train_time+1
  if error < acceptableError :
    trained = True
#  -----    学习完毕，提取矩阵中的W1 2，B1 2   ---------- # 
#  -----    Learn Complete,extracting paraments   ------- # 
  for mod in net1.modules:
    for conn in net1.connections[mod]:
      if conn.params.size==2652: #2652个,w1，52*51
        W1=conn.params.reshape(52,51) #W1为52*51的矩阵
      if conn.params.size==52: #52个,B1,1*52
        B1=conn.params.reshape(1,52) #B1为1*52的矩阵
      if conn.params.size==676: #676个,W2,13*52
        W2=conn.params.reshape(13,52) #W2为13*52的矩阵
      if conn.params.size==13: #13个,B2,1*13
        B2=conn.params.reshape(1,13) #B2为1*13的矩阵
#  ------    提取完毕，开始计算（13）式   --------------- #
#  ---- Extract complete,calculating baclword function -- #
  dp_du=np.vstack(( np.zeros((47,4)) , np.eye((4)) )) 
  #dp/du,1*51
  temp_input0=np.hstack((a[0:3,:].reshape(1,9),w[0:3,:].reshape(1,9),Angle[0:3,:].reshape(1,9),ET[0:3,:].reshape(1,12),pwm[1:4,:].reshape(1,12))).reshape(1,51)#P,1*51
  W1P_B1=(np.dot(temp_input0,W1.T)+B1)#W1*P+B1,1*52
  temp_sech=( 2/(np.exp(W1P_B1)+np.exp(-W1P_B1)) )**2#sech2(...),1*52
  temp_sech2=temp_sech.sum() #标量,sech一定小于1
  W2_sech=W2*temp_sech2 #13*52
  r_t=np.hstack((ia,iw,iAngle,iET)).ravel() #理想状态  
  y1=np.hstack((a[0:3,:].reshape(1,9),w[0:3,:].reshape(1,9),Angle[0:3,:].reshape(1,9),ET[0:3,:].reshape(1,12),pwm[1:4,:].reshape(1,12))).reshape(51)#P,1*51
  y_=np.hstack(net1.activate(y1)).ravel()#神经网络的y拔
  et_1=r_t-y_
  dJ_du0=np.dot( et_1,W2_sech)
  dJ_du1=n*np.dot( dJ_du0,W1)
  dJ_du=np.dot(dJ_du1,dp_du)  
  #标量*(1*51)*(51*4)=1*4
  pwm[0,]=pwm[1,]
  pwm[1,]=pwm[2,]
  pwm[2,]=pwm[3,]
  pwm[3,]=pwm[3,]+dJ_du
  i=0
  while i<4:
    if pwm[3,i]>1:
      pwm[3,i]=1
    elif pwm[3,i]<-1:
      pwm[3,i]=-1
    i=i+1
  if pwm[3,2]>0.4: #尾部水平速度限制
    pwm[3,2]=0.4
  elif pwm[3,2]<-0.4:
    pwm[3,2]=-0.4
#  ------  计算完毕，开始准备发送数据到下位机  ---------- #
#  ------ Calculate complete,Sending data to Arduino ---- #
  #将4个pwm拼成pwm1,pwm2,pwm3,pwm4
  #然后弄成send_data=pwm1,pwm2,pwm3,pwm4;校验码
  all_pwm=str(int(255*pwm[3,0]))+','+str(int(255*pwm[3,1]))+','+str(int(255*pwm[3,2]))+','+str(int(255*pwm[3,3]))
  checkpwm=0
  for letter in all_pwm:
    checkpwm+=ord(letter)
  send_data=all_pwm+';'+str(checkpwm)
  print "\nError: "+str(error)+" ,Trained "+str(train_time-1)+" times!\n"
  IDEAL_AVAIL=False
  sendtoDown(send_data)

def sendtoDown(data0):  #data0是将要发送的数据;校验码
  global SENDING  #端口独占锁
  global RECV  #下位机是否接到
  global down_recv
  global Send_queue
  global Recv_queue
  Send_queue.append(data0) #添加至队列
  #下面保证同一时刻只有一个send在发送数据
  while Send_queue.getlength() and SENDING==False: #只要队列里面有就一直发
    SENDING=True
    #下面是发送动作
    send_temp = Send_queue.getitem(0)
    #print send_temp
    ser.write(send_temp)
    Send_queue.delete(0)
    SENDING=False
  #发送完成
#  ------   发送完成  ---------- #
#  ------ Sending Over --------- #

#############################################################
##########    Recieving Data by Serial    ###################
#############################################################    
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
    down_recv=ser.readline()      #读端口
    if len(down_recv) > 0:
      Recv_queue.append(down_recv)
      while Recv_queue.getlength():   #读队列
        checksum(Recv_queue.getitem(0))

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
    Recv_queue.delete(0)
     
def splitnum(data):
  """分开、存储数据"""
  global IDEAL_AVAIL
  global hAngle
  #temp_input0=np.hstack((a[0:3,:].reshape(1,9),w[0:3,:].reshape(1,9),Angle[0:3,:].reshape(1,9),ET[0:3,:].reshape(1,12),pwm[1:4,:].reshape(1,12))).ravel()
  #numbers = net2.activate(temp_input0)#模拟从下位机返回的状态
  #激活用来模拟系统的网络：历史机身状态 + pwm历史+pwm实时
  try:
    #print data
    numbers=data.split(',')
    data0=float(numbers[0]/10)#加速度
    data1=float(numbers[1]/10)
    data2=float(numbers[2]/10)
    data3=float(numbers[3]/20)#角速度
    data4=float(numbers[4]/20)
    data5=float(numbers[5]/20)
    data6=float(numbers[6])#角度
    if data6<0:
      data6=-data6-180  #小于0  逐渐接近0
    else:
      data6=-data6+180  #大于0  逐渐接近0
    data6=data6-7.9
    data7=float(numbers[7]/180)
    data8=float(numbers[8]/180)
    data9=float( numbers[9]/330)#电机转速
    data10=float(numbers[10]/330)
    data11=float(numbers[11]/1320)
    data12=float(numbers[12]/1320)
  except:
    print "Exception!"
  else:
    print "Success!"
    data6=data6 / 180  #角度
    a[0,]=a[1,]
    a[1,]=a[2,]
    a[2,]=a[3,]
    a[3,]=[data0,data1,data2]#加速度
    w[0,]=w[1,]
    w[1,]=w[2,]
    w[2,]=w[3,]
    w[3,]=[data3,data4,data5]#角速度
    Angle[0,]=Angle[1,]
    Angle[1,]=Angle[2,]
    Angle[2,]=Angle[3,]
    Angle[3,]=[data6,data7,data8]#角度
    ET[0,]=ET[1,]
    ET[1,]=ET[2,]
    ET[2,]=ET[3,]
    ET[3,]=[data9,data10,data11,data12]#电机转速
    IDEAL_AVAIL = True


################################################################
###################    Keyboard Input    #######################
################################################################
class monitK(threading.Thread):
  #monit Keyboard
  def __init__(self):
    #self.thread_stop=False;
    threading.Thread.__init__(self)
  def run(self):
    global processes
    processes=processes+1
    monitKey()  #monitK的函数，检测键盘输入用
    
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
      if RUNNING == False:
        key_input=raw_input(">>please input:")
        KEY_AVAIL=True
        judgeK()   #分析输入
        time.sleep(0.1)

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
  """elif control=='quit':
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
    #set matrix auto save time """
  else:
    print ("Input false")
####################################################################
##################    Control Functions    #########################
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
    iAngle=(0.0,0.0,Angle[3,2])
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

########################################################
if __name__ == '__main__':
  try:
    main()
  except KeyboardInterrupt as e:
    print("exit!" + e)

#************** (C) COPYRIGHT 2016 ROBOTTEAM *****END OF FILE******************#
