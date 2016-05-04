/***  (C) Copyright © 2016 wuzp1230. All rights reserved. ********************
* File Name          : Robot.ino
* Author             : wuzp1230
* Version            : V1.0
* Date               : 03/26/2016
* Description        : This file provides the functions of Arduino Mega2560,
*                      including driving motor,and moniting status by MPU6050. 
*                      the heart of Arduino and the body & organs of the car. 
******************************************************************************/
#include <Wire.h>
#include <JY901.h>
#define TIMEOUT 100  //时间片长度，单位为ms
#define MAX 100       //字符数组长度
#define LM_LINES 64640     //左轮码盘线数
#define RM_LINES 64640     //右轮码盘线数
#define PM_LINES 614080     //尾部平行码盘线数
#define VM_LINES 775680     //尾部垂直码盘线数
#define PM_BIG_BOUNDARY 2000     //尾部平行码盘界限，L156
#define PM_SMALL_BOUNDARY -2000    
#define VM_BIG_BOUNDARY 200     //尾部平行码盘界限
#define VM_SMALL_BOUNDARY -18000     
//定义获取信息端口
#define RAPi_RX 19    //从树莓派接收的端口，树莓派为Serial1
#define RAPi_TX 18    //发送至树莓派的端口(这两个是反的)
#define MPU_RX  15    //从MPU6050接收的端口，6050为Serial2
#define MPU_TX  14    //发送至MPU6050的端口  TX对TX
//中断端口获取电机信息
//下面是控制电机用

#define TAIL_LEFT 30    //尾部位置左极限
#define TAIL_RIGHT 150  //尾部位置右极限
#define TAIL_UP 30      //尾部位置上极限
#define TAIL_DOWN 150   //尾部位置下极限


bool SEND_AVAIL = false;        //是否允许发送下一组数据
bool RECV_AVAIL = false;        //判断接收字符串是否可用
bool PWM_AVAIL = false;         //判断PWM数据是否可用
bool IS_CHECKED = false;        //数据是否校验成功
char c;                          //用于暂存数据
char ltr[2] = {'%', '\0'};            //用于strcat的数据
char Re_buf[24];                //6050读数据的缓存区
char tempx[50];
float a[3]={0,0,0}, w[3]={0,0,0}, Angle[3]={0,0,0};   //6050读数据
float ha[3]={0,0,0}, hw[3]={0,0,0}, hAngle[3]={0,0,0};  //6050历史最大数据
float ET[4];                    //四个码盘的角速度，Encoder_Theta
float hET[4];    //四个码盘的历史最大角速度，Encoder_Theta
int counter = 0;
int countL = 0, countR = 0, countP = 0, countV = 0;
int is_update=0;
int up=0;
int now;                        //用于测时间
int gap;                         //时间差
int sum = 0;                     //用于校验接收到&要发送的的数据
int i = 0;                    //计数器0
int j = 0;                    //计数器1
int sendTime = 0;              //发送重试次数，范围0-3
int recvTime = 0;              //发送重试次数，范围0-3
int EV0 = 0, EV1 = 0, EV2 = 0, EV3 = 0; //四个码牌的B脚电平，Voltage
int LM_P=0,RM_P=0,PM_P=0,VM_P=0; //控制四个电机的PWM信号，范围为0-±255
long int EP[4] = {0, 0, 0, 0}; //四个码盘的位置，Encoder_Position，0-360
long int EHP0=0, EHP1=0, EHP2=0, EHP3=0;//四个码盘的历史位置
long int ERP0=0, ERP1=0, ERP2=0, ERP3=0;//四个码盘的位置差
String comdata = "";
String data_send="";             //用于发送至树莓派的数据
String data_recv="";             //从树莓派接收到的初始数据
String temp="";       
///////////////////////////////////////////////////////////////////////////////////////
//////////////////////////             Setup and Loop           //////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
void setup() {
  //初始化串口
  Serial.begin(9600);                //0为电脑调试串口
  Serial1.begin(115200);              //1为树莓派，速率115200
  Serial3.begin(9600);                //3为MPU6050

  pinMode(6, OUTPUT);//PWM输出
  pinMode(36, OUTPUT); //四个电机的正反
  pinMode(37, OUTPUT);
  attachInterrupt(0, count0, RISING); // 0号中断2，上升沿中断
  pinMode(46, INPUT);//码盘B端口
  
  pinMode(7, OUTPUT);
  pinMode(40, OUTPUT);
  pinMode(41, OUTPUT);
  attachInterrupt(1, count1, RISING); // 1号中断3
  pinMode(47, INPUT);
  
  pinMode(8, OUTPUT);
  pinMode(42, OUTPUT);
  pinMode(43, OUTPUT);
  attachInterrupt(2, count2, RISING); // 2号中断21
  pinMode(48, INPUT);
  
  pinMode(9, OUTPUT);
  pinMode(44, OUTPUT);
  pinMode(45, OUTPUT);
  attachInterrupt(3, count3, RISING); // 3号中断20
  pinMode(49, INPUT);

}
void loop() {   //任务1，与上位机交互数据
  receiveData(); //执行接收
  getData();
}
///////////////////////////////////////////////////////////////////////////////////////
/////////////////////       Receive Data from Raspberry Pi       ////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
void receiveData()
{
  i = 0;                //计数器重置
  j = 0;                //计数器重置
  sum = 0;              //校验器重置
  //data_recv = "";
  temp = "";
  PWM_AVAIL = false;    //使一次的控制信号只能控制一个时间片
  LM_P = 0;RM_P = 0;PM_P = 0;VM_P = 0;
  //下面开始读
  while ( Serial1.available() > 0)
  {
    data_recv += char(Serial1.read());
    delay(2);
  }
  if(data_recv.length() > 0)
  {
    //Serial.print("1.First received: ");Serial.println(data_recv);
    if ( (data_recv[0] >= '0' && data_recv[0] <= '9') || data_recv[0] == '-' )
    {
      j = 0;
      i = 0;
      while (data_recv[i] != ';' )
      {
        while ( data_recv[i] != ',' && data_recv[i] != ';')
        {
          temp+= data_recv[i];
          sum += (int)data_recv[i];
          i++;
        }//出来时指针指向逗号（下一个数据）或分号
        if(data_recv[i] != ';')
        {sum += (int)data_recv[i];i++;}
        Serial.print("1.i= ");Serial.print(i);
        Serial.print(" Adding sum= ");Serial.println(sum);
        j++;//j是第几组数据
        if (j == 1) {
          LM_P = temp.toInt();
          LM_P=LM_P>255?255:LM_P;
          LM_P=LM_P<-255?-255:LM_P;
          Serial.print("i= ");Serial.print(i);Serial.print("J=1,LM_PWM= ");Serial.println(LM_P);
          //return;
        }//PWM左
        else if (j == 2) {
          RM_P = temp.toInt();
          RM_P=RM_P>255?255:RM_P;
          RM_P=RM_P<-255?-255:RM_P;
          Serial.print("i= ");Serial.print(i);Serial.print("J=2,RM_PWM= ");Serial.println(RM_P);
        }//PWM右
        else if (j == 3) {
          PM_P = temp.toInt();
          PM_P=PM_P>100?100:PM_P;
          PM_P=PM_P<-100?-100:PM_P;
          Serial.print("i= ");Serial.print(i);Serial.print("J=3,PM_PWM= ");Serial.println(PM_P);
        }//PWM水平
        else if (j == 4) {
          VM_P = temp.toInt();
          VM_P=VM_P>255?255:VM_P;
          VM_P=VM_P<-255?-255:VM_P;
          Serial.print("i= ");Serial.print(i);Serial.print("J=4,VM_PWM= ");Serial.println(VM_P);
        }//PWM垂直
        temp = "";
      }//读到sum，出来时指针指向';',tempi=0
      i++; //分号的下一个
      temp = "";
      while (data_recv[i] >='0' && data_recv[i] <='9'  )//读sum
      {
        temp += data_recv[i];
        i++;
      }
      data_recv = "";
      if (sum == temp.toInt())  //校验数据
      {
        setspd(LM_P,RM_P,PM_P,VM_P);
        Serial.println("PWM avaliable!");
        return;
      }
      else
      {
        Serial.println("PWM Error!");
        Serial.println(sum);
        Serial.println(temp.toInt());
        return;
      }
    }
  }
}
///////////////////////////////////////////////////////////////////////////////////////
/////////////////////////             Control Activity            //////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
void setspd(int l1, int r1, int p1, int v1)
{
  analogWrite(6, abs(l1));
  digitalWrite(36, l1>0?1:0);   // sets the LED on
  digitalWrite(37, l1<0?1:0);    // sets the LED off
  analogWrite(7, abs(r1));
  digitalWrite(40, r1>0?1:0);   // sets the LED on
  digitalWrite(41, r1<0?1:0);    // sets the LED off
  if( EP[2]>PM_BIG_BOUNDARY )  //Turn left
  {
    analogWrite(8, abs(p1));
    digitalWrite(42,1);   // sets the LED on
    digitalWrite(43,0);    // sets the LED off
  }
  else if( EP[2]<PM_SMALL_BOUNDARY )
  {
    analogWrite(8, abs(p1));
    digitalWrite(42, 0);    // sets the LED off
    digitalWrite(43, 1);   // sets the LED on
  }
  else
  {
    analogWrite(8, abs(p1));
    digitalWrite(42, p1<0?1:0);    // sets the LED off
    digitalWrite(43, p1>0?1:0);   // sets the LED on
  }
  
  if( EP[3]>VM_BIG_BOUNDARY  )  //防尾部越界
  {
    analogWrite(9, abs(v1));
    digitalWrite(44, 1);   // sets the LED on
    digitalWrite(45, 0);    // sets the LED
  }
  else if( EP[3]<VM_SMALL_BOUNDARY  )
  {
    analogWrite(9, abs(v1));
    digitalWrite(45, 1);   // sets the LED on
    digitalWrite(44, 0);    // sets the LED
  }
  else
  {
    analogWrite(9, abs(v1));
    digitalWrite(45, v1>=0?1:0);   // sets the LED on
    digitalWrite(44, v1<=0?1:0);    // sets the LED
  }
}
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////             Get Data  from MPU6050         ////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
void getData() {   //任务2，从6050和电机获取数据、处理数据
  getMotor(); //OK
  get6050(); //OK
  processData();
  sendData();
  delay(30);
}
void get6050() {   //OK
  //获取6050的参数
  while (Serial3.available()) 
  {
    JY901.CopeSerialData(Serial3.read()); //Call JY901 data cope function
  }
  a[0]=(float)JY901.stcAcc.a[0]/32768*16;  //Oned //5
  a[1]=(float)JY901.stcAcc.a[1]/32768*16;
  a[2]=(float)JY901.stcAcc.a[2]/32768*16;
  w[0]=(float)JY901.stcGyro.w[0]/32768*2000; //10
  w[1]=(float)JY901.stcGyro.w[1]/32768*2000;
  w[2]=(float)JY901.stcGyro.w[2]/32768*2000;
  Angle[0]=(float)JY901.stcAngle.Angle[0]/32768*180; //180
  Angle[1]=(float)JY901.stcAngle.Angle[1]/32768*180;
  Angle[2]=(float)JY901.stcAngle.Angle[2]/32768*180;
  //Serial.print(Angle[0]);Serial.print(Angle[1]);Serial.println(Angle[2]);
}
void getMotor() {   //OK
  //获取电机的位置变化
  now = millis() - now; //精确时间差,
  ERP0 = EP[0] - EHP0;    //电机位置差,线数
  ERP1 = EP[1] - EHP1;
  ERP2 = EP[2] - EHP2;
  ERP3 = EP[3] - EHP3;//角速度=角度/时间，单位为度/s
  if(ERP0!=0&&now!=0)
  {
    ET[0] = 360 * 1000 * ERP0/ 330 / now ;}//左轮码盘线数 //150
  if(ERP1!=0&&now!=0)
  {
    ET[1] = 360 * 1000 * ERP1/ 330 / now ;}//右轮码盘线数 //150
  if(ERP2!=0&&now!=0)
  {
    ET[2] = 360  * 1000 * ERP2/ 1320 / now;}//尾部平行码盘线数 //100
  if(ERP3!=0&&now!=0)
  {
    ET[3] = 360  * 1000 * ERP3/ 1320 / now;}//尾部垂直码盘线数  //450
  now = millis();  //与下一次计时距离一个时间片
  EHP0 = EP[0]; //更新历史数据
  EHP1 = EP[1];
  EHP2 = EP[2];
  EHP3 = EP[3];
}
//处理将要发送的数据，加速度，角速度，角度
///////////////////////////////////////////////////////////////////////////////////////
/////////////////////////             Process Data            //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
void processData(){ 
  data_send = "";
  for (i = 0; i < 3; i++) {
    //char* dtostrf(double _val,signed char _width, unsigned char prec, char* _s)
    dtostrf(a[i],6, 4,tempx);
    //sprintf(tempx, "% f", a[i]);
    data_send += tempx;
    data_send += ',';
  }
  for (i = 0; i < 3; i++) {
    dtostrf(w[i],6, 4,tempx);
    data_send += tempx;
    data_send += ',';
  }
  for (i = 0; i < 3; i++) {
    dtostrf(Angle[i],6, 4,tempx);
    data_send += tempx;
    data_send += ',';
  }
  for (i = 0; i < 3; i++) {
    dtostrf(ET[i],6, 4,tempx);
    data_send += tempx;
    data_send += ',';
  }
      dtostrf(ET[i],6, 4,tempx);
  data_send += tempx;
  //Serial.print("1.Processed Data_send is:");Serial.println(data_send);
}
///////////////////////////////////////////////////////////////////////////////////////
/////////////////////////             Send Data            //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
void sendData() {
  i = 0;
  sum = 0;
  //最终生成的为“数据;校验码”
  data_send+=';';
  Serial.print("2.Data_send is:");Serial.println(data_send);
  while (data_send[i] != ';') { //生成sum
    sum += (int)data_send[i];
    i++;
  }
  //Serial.print("3.sum is:");Serial.println(sum);
  sprintf(tempx, "%d", sum);
  data_send+=tempx;  //追加sum
  Serial1.println(data_send);
  /*Serial1.println(data)从串行端口输出数据，
  跟随一个回车（ASCII 13, 或 ‘r’）和一个换行符（ASCII 10, 或 ‘n’）。
  这个函数所取得的值与Serial1.print()一样。*/
}
///////////////////////////////////////////////////////////////////////////////////////
////////////////////////////             Counter               //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
void count0() { //左轮码盘中断计数  //OK
  EV0 = digitalRead(46); //左轮码盘
  if ( EV0 == 0 )
    EP[0]++;
  else EP[0]--;
}
void count1() { //右轮码盘中断计数
  EV1 = digitalRead(47); //右轮码盘
  if ( EV1 == 1 )
    EP[1]++;
  else EP[1]--;
}
void count2() { //水平码盘中断计数
  EV2 = digitalRead(48); //水平码盘
  if ( EV2 == 1 )
    EP[2]++;
  else EP[2]--;
}
void count3() { //垂直码盘中断计数
  EV3 = digitalRead(49); //垂直码盘
  if ( EV3 == 1 )
    EP[3]++;
  else EP[3]--;
}
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////             Stop            /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
//停机，所有电机回转至180****************************************改进角度
void stop0() {
  while (EP[2] > 10) {
    digitalWrite(42, 0);
    digitalWrite(43, 1);
    analogWrite(8, 50 ); //端口号，PWM速度
    getMotor();
  }
  while (EP[2] < -10) {
    digitalWrite(42, 1);
    digitalWrite(43, 0);
    analogWrite(8, 50 ); //端口号，PWM速度
    getMotor();
  }
  while (EP[2] > 3 && EP[2] <= 10) {
    digitalWrite(42, 0);
    digitalWrite(43, 1);
    analogWrite(8, 10 ); //端口号，PWM速度
    getMotor();
  }
  while (EP[2] >= -10 && EP[2] < 3) {
    digitalWrite(42, 1);
    digitalWrite(43, 0);
    analogWrite(8, 10 ); //端口号，PWM速度
    getMotor();
  }
  while (EP[2] >= -3 && EP[2] < 3) {
    digitalWrite(42, 1);
    digitalWrite(43, 1);
    analogWrite(8, 0 ); //端口号，PWM速度
    getMotor();
  }
  digitalWrite(44, 1);
  digitalWrite(45, 1);
  analogWrite(9, 0 ); //端口号，PWM速度
  getMotor();
}
///////////////////////////////////////////////////////////////////
/*
void control0() //示例控制
{
  while (Serial.available() > 0)
  {
    comdata += char(Serial.read());
    delay(2);
  }
  if (comdata.length() > 0)
  {
    switch(comdata[0])
    {
      case 'w':
        Serial1.print("WWW");
        setspd(255, 255, 0, 0);
        break;
      case 's':
        Serial1.print("SSS");
        setspd(-255, -255, 0, 0);
        break;
      case 'a':
        Serial1.print("AAA");
        setspd(-255, 255, 0, 0);
        break;
      case 'd':
        Serial1.print("DDD");
        setspd(255, -255, 0, 0);
        break;
      case '4':
        Serial1.print("444");
        setspd(0, 0, 50, 0);
        break;
      case '6':
        Serial1.print("666");
        setspd(0, 0, -50, 0);
        break;
      case '8':
        Serial1.print("888");
        setspd(0, 0, 0, 255);
        break;
      case '5':
        Serial1.print("555");
        setspd(0, 0, 0, -255);
        break;
      case '0':
        Serial1.print("000");
        setspd(0, 0, 0, 0);
        break;
      default:
        Serial1.print("comdata[0]");Serial1.println(comdata[0]);
        setspd(0, 0, 0, 0);
        break;
    }
    comdata = "";
    delay(300);
    setspd(0, 0, 0, 0);
    //Serial.println(comdata);
  }
}
void init_0() {
  //自检
  get6050();
  if (a[0] == 0 && a[1] == 0 && a[2] == 0 && w[0] == 0 && w[1] == 0
      && w[2] == 0 && Angle[0] == 0 && Angle[1] == 0 && Angle[2] == 0)
  {
    strcpy(data_send, "MPUER");
  }
  else {
    strcpy(data_send, "MPUOK");
  }
  //读电机
  LM_P = 127, RM_P = 127, PM_P = 127, VM_P = 127;
  control();
  delay(5);
  getMotor();
  if (EP[0] < 180 && EP[1] < 180 && EP[2] < 180 && EP[3] < 180) {
    LM_P = -255, RM_P = -255, PM_P = -255, VM_P = -255;
    control();
    delay(5);
    getMotor();
    if (EP[0] > 180 && EP[1] > 180 && EP[2] > 180 && EP[3] > 180) {
      stop0;
      delay(5);
      getMotor();
      strcpy(temp, ",MOTOK");
      strcat(data_send, temp);
    }
    else {
      stop0();
      strcpy(temp, ",MOTRE");
      strcat(data_send, temp);
    }
  }
  else {
    LM_P = 0, RM_P = 0, PM_P = 0, VM_P = 0;
    control();
    strcpy(temp, ",MOTFE");
    strcat(data_send, temp);
    //发送数据
    sendData();
  }
}
//////////////////////////////////////////////////


/////////////////////////////////////////////////////////////
  else if (sendTime >= 3) {            //发送3次但是对方全部接收错误
    stop0();
    for (i = 0; i <= 5; i++)
    {           //尾部水平来回摆三下
      if (i % 2 == 0) {
        while (EP[2] < 200) {
          digitalWrite(PM_H, 1);
          digitalWrite(PM_L, 0);
          analogWrite(PM_PWM, 255 ); //端口号，PWM速度
          getMotor();
        }
        while (EP[2] > 200 && EP[2] <= 210) {
          digitalWrite(PM_H, 1);
          digitalWrite(PM_L, 0);
          analogWrite(PM_PWM, (209 - EP[2]) ); //端口号，PWM速度
          getMotor();
        }
        delay(5);
      }
      else {
        while (EP[2] > 160) {
          digitalWrite(PM_H, 0);
          digitalWrite(PM_L, 1);
          analogWrite(PM_PWM, 255 ); //端口号，PWM速度
          getMotor();
        }
        while (EP[2] <= 160 && EP[2] > 150) {
          digitalWrite(PM_H, 0);
          digitalWrite(PM_L, 1);
          analogWrite(PM_PWM, (EP[2] - 180) ); //端口号，PWM速度
          getMotor();
        }
        delay(5);
      }
    sendTime = 2;
    }
    stop0();
  }
  */
  /*void control() {
  //控制
  if (LM_P > 0) {
    digitalWrite(LM_H, 1);
    digitalWrite(LM_L, 0);
    analogWrite(LM_PWM, LM_P); //端口号，PWM速度
    Serial.println("1_L");
  }
  else if (LM_P < 0) {
    digitalWrite(LM_H, 0);
    digitalWrite(LM_L, 1);
    analogWrite(LM_PWM, -LM_P); //端口号，PWM速度
    Serial.println("1_R");
  }
  else if (LM_P == 0) {            //紧急停车
    digitalWrite(LM_H, 1);
    digitalWrite(LM_L, 1);
    Serial.println("1_0");
  }
  if (RM_P > 0) {
    digitalWrite(RM_H, 1);
    digitalWrite(RM_L, 0);
    analogWrite(RM_PWM, RM_P); //端口号，
    Serial.println("2_L");
  }
  else if (RM_P < 0) {
    digitalWrite(RM_H, 0);
    digitalWrite(RM_L, 1);
    analogWrite(RM_PWM, -RM_P); //端口号，PWM速度
    Serial.println("2_R");
  }
  else if (RM_P == 0) {
    digitalWrite(RM_H, 1);
    digitalWrite(RM_L, 1);
    Serial.println("2_0");
  }
  if (PM_P > 0 && EP[2] > TAIL_LEFT && EP[2] < TAIL_RIGHT) {
    digitalWrite(PM_H, 1);
    digitalWrite(PM_L, 0);
    analogWrite(PM_PWM, PM_P); //端口号，PWM速度
    Serial.println("3_L");
  }
  else if (PM_P < 0 && EP[2] > TAIL_LEFT && EP[2] < TAIL_RIGHT) {
    digitalWrite(PM_H, 0);
    digitalWrite(PM_L, 1);
    analogWrite(PM_PWM, -PM_P); //端口号，PWM速度
    Serial.println("3_R");
  }
  else if (PM_P == 0) {
    digitalWrite(PM_H, 1);
    digitalWrite(PM_L, 1);
    Serial.println("3_0");
  }
  if (VM_P > 0 && EP[3] > TAIL_UP && EP[3] < TAIL_DOWN) {
    digitalWrite(VM_H, 1);
    digitalWrite(VM_L, 0);
    analogWrite(VM_PWM, VM_P); //端口号，PWM速度
    Serial.println("4_L");
  }
  else if (VM_P < 0 && EP[3] > TAIL_UP && EP[3] < TAIL_DOWN) {
    digitalWrite(VM_H, 0);
    digitalWrite(VM_L, 1);
    analogWrite(VM_PWM, -VM_P); //端口号，PWM速度
    Serial.println("4_R");
  }
  else if (VM_P == 0) {
    digitalWrite(VM_H, 1);
    digitalWrite(VM_L, 1);
    Serial.println("4_0");
  }
}*/

/*****(C) Copyright © 2016 wuzp1230.All rights reserved.*****END OF FILE****************/
