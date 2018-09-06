#include<Servo.h>
#include <Wire.h>
#include "SR04.h"
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID         0xc0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID      0xc2
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD   0x50
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define VL53L0X_REG_SYSRANGE_START                  0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS         0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS             0x14
#define address 0x29
byte gbuf[16];


 /*******************下面定义了所有的接口***************************/
      /********寻迹模块接口**********/
      const int FRONT_SS0=47;
      const int FRONT_SS1=22; 
      const int FRONT_SS2=23; 
      const int FRONT_SS3=24; 
      const int FRONT_SS4=25; 
      const int FRONT_SS5=26;
      const int FRONT_SS6=48;
      const int LEFT_SS1=27; 
      const int LEFT_SS2=28; 
      const int LEFT_SS3=29; 
      const int RIGHT_SS1=30; 
      const int RIGHT_SS2=31; 
      const int RIGHT_SS3=32;
      const int BEHIND_SS1=38; 
      const int BEHIND_SS2=39; 
      const int BEHIND_SS3=40; 
      const int BEHIND_SS4=41; 
      const int BEHIND_SS5=42; 
      /******电机的接口*************/
      const int DIANJI_LEFT_SPEED=4;//左边电机速度串口 暂时用4
      const int DIANJI_LEFT_TOWARDS=33;//左边电机方向串口 暂时用5
      const int DIANJI_RIGHT_SPEED=5;//右边电机速度串口
      const int DIANJI_RIGHT_TOWARDS=34;//右边电机方向串口
      const int DIANJI_CENTER_SPEED=6;//中间电机速度串口
      const int DIANJI_CENTER_TOWARDS=35;//中间电机方向串口
      /*******电磁阀的继电器接口**********/
      const int QIGAN=2;
      /********蜗轮蜗杆电机继电器接口*********/
      const int WOLUN_1=36;
      const int WOLUN_2=37;
      /*****爪子的接口*******/
      const int CLAW=44;
      const int ROTATE=45;
      const int STEER=46;
      /******超声测距的接口********/
      const int BE_CHAOSHEN_ECHO=9; 
      const int BE_CHAOSHEN_TRI=8;
      const int UP_CHAOSHEN_ECHO=11;
      const int UP_CHAOSHEN_TRI=10;
      /*********rfid模块的接口****************/

/***************************************/



/***********下面是激光测距函数**************/
class DisOfVL{
 public:
 DisOfVL(void){
  byte i=0;
 }
int distance() {
  byte val1 = read_byte_data_at(VL53L0X_REG_IDENTIFICATION_REVISION_ID);
  val1 = read_byte_data_at(VL53L0X_REG_IDENTIFICATION_MODEL_ID);
  val1 = read_byte_data_at(VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD);
  val1 = read_byte_data_at(VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD);
  write_byte_data_at(VL53L0X_REG_SYSRANGE_START, 0x01);
  byte val = 0;
  int cnt = 0;
  while (cnt < 100) { // 1 second waiting time max
    delay(10);
    val = read_byte_data_at(VL53L0X_REG_RESULT_RANGE_STATUS);
    if (val & 0x01) break;
    cnt++;
  }
  read_block_data_at(0x14, 12);
  uint16_t acnt = makeuint16(gbuf[7], gbuf[6]);
  uint16_t scnt = makeuint16(gbuf[9], gbuf[8]);
  uint16_t dist = makeuint16(gbuf[11], gbuf[10]);
  byte DeviceRangeStatusInternal = ((gbuf[0] & 0x78) >> 3);
  Serial.print("distance ");       Serial.println(dist);
  return dist;
};
private:
uint16_t bswap(byte b[]) {
  // Big Endian unsigned short to little endian unsigned short
  uint16_t val = ((b[0] << 8) & b[1]);
  return val;
}
uint16_t makeuint16(int lsb, int msb) {
    return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}
void write_byte_data(byte data) {
  Wire.beginTransmission(address);
  Wire.write(data);
  Wire.endTransmission();
}
void write_byte_data_at(byte reg, byte data) {
  // write data word at address and register
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}
void write_word_data_at(byte reg, uint16_t data) {
  // write data word at address and register
  byte b0 = (data &0xFF);
  byte b1 = ((data >> 8) && 0xFF);
    
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(b0);
  Wire.write(b1);
  Wire.endTransmission();
}
byte read_byte_data() {
  Wire.requestFrom(address, 1);
  while (Wire.available() < 1) delay(1);
  byte b = Wire.read();
  return b;
}
byte read_byte_data_at(byte reg) {
  //write_byte_data((byte)0x00);
  write_byte_data(reg);
  Wire.requestFrom(address, 1);
  while (Wire.available() < 1) delay(1);
  byte b = Wire.read();
  return b;
}
uint16_t read_word_data_at(byte reg) {
  write_byte_data(reg);
  Wire.requestFrom(address, 2);
  while (Wire.available() < 2) delay(1);
  gbuf[0] = Wire.read();
  gbuf[1] = Wire.read();
  return bswap(gbuf); 
}
void read_block_data_at(byte reg, int sz) {
  int i = 0;
  write_byte_data(reg);
  Wire.requestFrom(address, sz);
  for (i=0; i<sz; i++) {
    while (Wire.available() < 1) delay(1);
    gbuf[i] = Wire.read();
  }
}
uint16_t VL53L0X_decode_vcsel_period(short vcsel_period_reg) {
  // Converts the encoded VCSEL period register value into the real
  // period in PLL clocks
  uint16_t vcsel_period_pclks = (vcsel_period_reg + 1) << 1;
  return vcsel_period_pclks;
}
};
DisOfVL *Dis;
//下面是读取距离的函数
double distance(){
  Dis=new DisOfVL();
  Serial.print("juli:");
  Serial.print(Dis->distance());
  Serial.println("");
  return Dis->distance();
}    
  /**********************************************/

  
 /**************下面定义了寻迹模块********************/
   int front_ss0;
   int front_ss1;
   int front_ss2;
   int front_ss3;
   int front_ss4;
   int front_ss5;
   int front_ss6;
   int left_ss1;
   int left_ss2;
   int left_ss3;
   int right_ss1;
   int right_ss2;
   int right_ss3;
   int behind_ss1;
   int behind_ss2;
   int behind_ss3;
   int behind_ss4;
   int behind_ss5;
   //把所有寻迹模块读一次
   void read_xunji(){
      front_ss0=digitalRead(FRONT_SS0);
      front_ss1=digitalRead(FRONT_SS1);
      front_ss2=digitalRead(FRONT_SS2);
      front_ss3=digitalRead(FRONT_SS3);
      front_ss4=digitalRead(FRONT_SS4);
      front_ss5=digitalRead(FRONT_SS5);
      front_ss6=digitalRead(FRONT_SS6);
      left_ss1=digitalRead(LEFT_SS1);
      left_ss2=digitalRead(LEFT_SS2);
      left_ss3=digitalRead(LEFT_SS3);
      right_ss1=digitalRead(RIGHT_SS1);
      right_ss2=digitalRead(RIGHT_SS2);
      right_ss3=digitalRead(RIGHT_SS3);
      behind_ss1=digitalRead(BEHIND_SS1);
      behind_ss2=digitalRead(BEHIND_SS2);
      behind_ss3=digitalRead(BEHIND_SS3);
      behind_ss4=digitalRead(BEHIND_SS4);
      behind_ss5=digitalRead(BEHIND_SS5);
    }
 /******************************************/

 /***********下面是爪子的参数与函数**************/
const int CLAW_OPEN=30;
const int CLAW_CLOSE=110;
const int STEER_HO=0;
const int STEER_VE=80;
const int STEER_HO_BE=165;
const int ROTATE_FRONT=20;
const int ROTATE_BEHI=180;
const int INTER_TIME=500;

Servo servo_claw;
Servo servo_rotate;
Servo servo_steer;

void claw_init(){
  servo_claw.attach(44);
  servo_rotate.attach(45);
  servo_steer.attach(46);
  servo_claw.write(CLAW_OPEN);
  servo_rotate.write(ROTATE_FRONT);
  servo_steer.write(STEER_VE);
  delay(1000);
  }
void claw_action1() {
  /*
   * servo_claw: STEER_VE->STEER_HO
   */
  for(int i=STEER_VE;i>STEER_HO;--i){
     servo_steer.write(i);
     delay(20);
    }
  delay(INTER_TIME);
}

void claw_action2() {
  /*
   * CLAW_OPEN->CLAW_CLOSE
   */
   for(int i=CLAW_OPEN;i<CLAW_CLOSE;++i){
      servo_claw.write(i);
      delay(20);
    }
    delay(INTER_TIME);
}

void claw_action3() {
  /*
   * STEER_HO->STEER_VE]\+
   */
   for(int i=STEER_HO;i<STEER_VE;++i){
      servo_steer.write(i);
      delay(20);
    }
    delay(INTER_TIME);
}

void claw_action4() {
  /*
   * ROTATE_FRONT->ROTATE_BEHI
   */
  for(int i=ROTATE_FRONT;i<ROTATE_BEHI;++i){
       servo_rotate.write(i);
       delay(20);
    }
    delay(INTER_TIME);
}

void claw_action5() {
  /*
   * STEER_VE->STEER_HO_BE
   */
   for(int i=STEER_VE;i<STEER_HO_BE;++i){
      servo_steer.write(i);
      delay(20);
    }
    delay(INTER_TIME);
}


void claw_action6() {
  /*
   *CLAW_CLOSE-> CLAW_OPEN
   */
   for(int i=CLAW_CLOSE;i>0;--i){
      servo_claw.write(i);
      delay(20);
    }
    delay(INTER_TIME);
}

void claw_action7() {
  /*
   * STEER_HO_BE->STEER_VE
   */
   for(int i=STEER_HO_BE;i>STEER_VE;--i){
      servo_steer.write(i);
      delay(20);
    }
    delay(INTER_TIME);
}

void claw_action8() {
  /*
   * ROTATE_BEHI->ROTATE_FRONT
   */
   for(int i=ROTATE_BEHI;i>ROTATE_FRONT;--i){
      servo_rotate.write(i);
      delay(20);
    }
    delay(INTER_TIME);
}

//取旗
void claw_quqi(){
  claw_action1();
  claw_action2();
  claw_action3();
}

//放旗
void claw_fangqi(){
  claw_action4();
  claw_action5();
  claw_action6();
  claw_action7();
  claw_action8();  
}

 /******************************************/

 
 /***********下面是电机的参数与函数***********/
const int BLACK=0;
const int WHITE=1;
const int GO_AHEAD_SPEED=20;//直走速度
const int GO_AHEAD_TIME=100;//直走时间
const int TRUN_LEFT_TIME=100;//水平左转时间
const int TURN_RIGHT_TIME=100;//水平右转时间
const int TURN_ROTATE_LEFT_TIME=100;//旋转左转时间
const int TURN_ROTATE_RIGHT_TIME=100;//旋转右转时间
const int LEFT_ROTATE_SPEED=20;//向左旋转速度
const int RIGHT_ROTATE_SPEED=20;//向右旋转速度

//前进
void car_go(){
  digitalWrite(DIANJI_LEFT_TOWARDS,HIGH);
  digitalWrite(DIANJI_RIGHT_TOWARDS,LOW);
  digitalWrite(DIANJI_CENTER_TOWARDS,HIGH);
  analogWrite(DIANJI_LEFT_SPEED,0);
  analogWrite(DIANJI_RIGHT_SPEED,255);
  analogWrite(DIANJI_CENTER_SPEED,255);
}

//后退
void car_back(){
  digitalWrite(DIANJI_LEFT_TOWARDS,LOW);
  digitalWrite(DIANJI_RIGHT_TOWARDS,HIGH);
  digitalWrite(DIANJI_CENTER_TOWARDS,HIGH);
  analogWrite(DIANJI_LEFT_SPEED,255);
  analogWrite(DIANJI_RIGHT_SPEED,0);
  analogWrite(DIANJI_CENTER_SPEED,255);
}

//水平左移
void car_turn_right(){
  int lunzi_1_speed=40;
  int lunzi_2_speed=20;
  int lunzi_3_speed=80;
  digitalWrite(DIANJI_LEFT_TOWARDS,LOW);
  digitalWrite(DIANJI_RIGHT_TOWARDS,LOW);
  digitalWrite(DIANJI_CENTER_TOWARDS,HIGH);
  analogWrite(DIANJI_LEFT_SPEED,255-lunzi_3_speed);
  analogWrite(DIANJI_RIGHT_SPEED,255-lunzi_1_speed);
  analogWrite(DIANJI_CENTER_SPEED,lunzi_2_speed);
}

//水平右移
void car_turn_left(){
  int lunzi_1_speed=40;
  int lunzi_2_speed=20;
  int lunzi_3_speed=100;
  digitalWrite(DIANJI_LEFT_TOWARDS,HIGH);
  digitalWrite(DIANJI_RIGHT_TOWARDS,HIGH);
  digitalWrite(DIANJI_CENTER_TOWARDS,LOW);
  analogWrite(DIANJI_LEFT_SPEED,lunzi_3_speed);
  analogWrite(DIANJI_RIGHT_SPEED,lunzi_1_speed);
  analogWrite(DIANJI_CENTER_SPEED,255-lunzi_2_speed);
}

//旋转左转
void car_rotate_right(){
  digitalWrite(DIANJI_LEFT_TOWARDS,HIGH);//这里电压高低和速度需要根据具体情况再做修改
  digitalWrite(DIANJI_RIGHT_TOWARDS,HIGH);
  digitalWrite(DIANJI_CENTER_TOWARDS,HIGH);
  analogWrite(DIANJI_LEFT_SPEED,LEFT_ROTATE_SPEED);
  analogWrite(DIANJI_RIGHT_SPEED,LEFT_ROTATE_SPEED);
  analogWrite(DIANJI_CENTER_SPEED,LEFT_ROTATE_SPEED);
  //delay(TURN_ROTATE_LEFT_TIME);
}

//旋转右转
void car_rotate_left(){
  digitalWrite(DIANJI_LEFT_TOWARDS,LOW);//这里电压高低和速度需要根据具体情况再做修改
  digitalWrite(DIANJI_RIGHT_TOWARDS,LOW);
  digitalWrite(DIANJI_CENTER_TOWARDS,LOW);
  analogWrite(DIANJI_LEFT_SPEED,255-RIGHT_ROTATE_SPEED);
  analogWrite(DIANJI_RIGHT_SPEED,255-RIGHT_ROTATE_SPEED);
  analogWrite(DIANJI_CENTER_SPEED,255-RIGHT_ROTATE_SPEED);
}

//停下
void car_stop(){
  digitalWrite(DIANJI_LEFT_TOWARDS,HIGH);//这里电压高低和速度需要根据具体情况再做修改
  digitalWrite(DIANJI_RIGHT_TOWARDS,HIGH);
  digitalWrite(DIANJI_CENTER_TOWARDS,HIGH);
  analogWrite(DIANJI_LEFT_SPEED,255);
  analogWrite(DIANJI_RIGHT_SPEED,255);
  analogWrite(DIANJI_CENTER_SPEED,255);
}

//直走寻迹
void go_ahead_xunji(){
      read_xunji();
      if( (front_ss1==BLACK && front_ss0==1)||(front_ss5==BLACK && front_ss6==1) ){
        car_go();
      }else if(front_ss1==BLACK && behind_ss5==BLACK){
        car_turn_left(); //要看安装情况 可能需要修改
      }else if(front_ss5==BLACK && behind_ss1==BLACK){
        car_turn_right();
      }else if(front_ss1==BLACK){
        car_rotate_left();
      }else if(front_ss5==BLACK){
        car_rotate_right();
      }else{
        car_go();
      }
}

//后退寻迹
void go_back_xunji(){
    //后退
      read_xunji();
      if((front_ss1==BLACK && front_ss0==1)||(front_ss5==BLACK && front_ss6==1)){
        car_back();
      }else if(front_ss1==BLACK && behind_ss5==BLACK){
        car_turn_left(); //要看安装情况 可能需要修改
      }else if(front_ss5==BLACK && behind_ss1==BLACK){
        car_turn_right();
      }else if(front_ss1==BLACK){
        car_rotate_left();
      }else if(front_ss5==BLACK){
        car_rotate_right();
      }else{
        car_back();
      }
}


//直走寻迹 遇到岔路口时停下
void go_ahead_xunji_to_cross(){
    //直走
      while(1){
        read_xunji();
        if(left_ss1==1 || right_ss1==1 ){
          car_stop();
          break;
        }else if( (front_ss1==BLACK && front_ss0==1)||(front_ss5==BLACK && front_ss6==1) ){
        car_go();
        }else if(front_ss1==BLACK && behind_ss5==BLACK){
          car_turn_left(); //要看安装情况 可能需要修改
        }else if(front_ss5==BLACK && behind_ss1==BLACK){
          car_turn_right();
        }else if(front_ss1==BLACK){
          car_rotate_left();
        }else if(front_ss5==BLACK){
          car_rotate_right();
        }else{
          car_go();
        }
        delay(50);
      }
}

//后退寻迹 遇到岔路口停下
void go_back_xunji_to_cross(){
    //后退
      while(1){
        read_xunji();
        if(left_ss1==1 || right_ss1==1 ){
          car_stop();
          break;
        }else if( (front_ss1==BLACK && front_ss0==1)||(front_ss5==BLACK && front_ss6==1) ){
          car_back();
        }else if(front_ss1==BLACK && behind_ss5==BLACK){
          car_turn_left(); //要看安装情况 可能需要修改
        }else if(front_ss5==BLACK && behind_ss1==BLACK){
          car_turn_right();
        }else if(front_ss1==BLACK){
          car_rotate_left();
        }else if(front_ss5==BLACK){
          car_rotate_right();
        }else{
          car_back();
        }
        delay(50);
      }
}


int rotate_begin_time=2000;//开始旋转的时间
int rotate_begin_flag=1;//是否完成开始旋转 完成的时候变成0

//左转90度的测试函数 即向左旋转 在转动了90度之后停下
void rotate_left_90(){
  while(1){
    read_xunji();
    if(rotate_begin_flag==1){//开始时 先让它转个2秒
      car_rotate_left();
      delay(rotate_begin_time);
      rotate_begin_flag=0;
    }else{
      if(front_ss1==WHITE && front_ss5==WHITE && front_ss3==BLACK){
        car_stop();
        rotate_begin_flag=1;
        break;
      }else{
        car_rotate_left();
        delay(50);
      }
    }
  }
}

//右转90度的测试函数 即向右旋转 在转动了90度之后停下
void rotate_right_90(){
  while(1){
    read_xunji();
    if(rotate_begin_flag==1){//开始时 先让它转个2秒
      car_rotate_right();
      delay(rotate_begin_time);
      rotate_begin_flag=0;
    }else{
      if(front_ss1==WHITE && front_ss5==WHITE && front_ss3==BLACK){
        car_stop();
        rotate_begin_flag=1;
        break;
      }else{
        car_rotate_right();
        delay(50);
      }
    }
  }
}

//用来调整轮子转动方向的代码
void suitable(){
   digitalWrite(DIANJI_LEFT_TOWARDS,HIGH);//这里电压高低和速度需要根据具体情况再做修改
   digitalWrite(DIANJI_RIGHT_TOWARDS,HIGH);
   digitalWrite(DIANJI_CENTER_TOWARDS,HIGH);
   analogWrite(DIANJI_LEFT_SPEED,100);
   analogWrite(DIANJI_RIGHT_SPEED,100);
   analogWrite(DIANJI_CENTER_SPEED,100);
}

/**************************/

/**********下面是超声测距的参数与函数************/

SR04 be_chaoshen=SR04(BE_CHAOSHEN_ECHO,BE_CHAOSHEN_TRI);
SR04 up_chaoshen=SR04(UP_CHAOSHEN_ECHO,UP_CHAOSHEN_TRI);

//向上测距
long up_getdistance(){
  long a=up_chaoshen.Distance();
  return a;  
}

//向后测距
long be_getdistance(){
  long a=be_chaoshen.Distance();
  return a;
}

/********************************/

/*********下面是蜗轮蜗杆电机的函数和参数************/

void pagan_down(){
  digitalWrite(WOLUN_1,HIGH);
  digitalWrite(WOLUN_2,LOW);
}

void pagan_up(){
  digitalWrite(WOLUN_1,LOW);
  digitalWrite(WOLUN_2,HIGH);
}

void pagan_stop(){
  digitalWrite(WOLUN_1,HIGH);
  digitalWrite(WOLUN_2,HIGH);
}

//测试两个继电器,每3s换一次
void pagan_test1(){
  pagan_up();
  delay(3000);
  pagan_down();  
  delay(3000);
  pagan_stop();
  delay(3000);
}

/********************************************/

/*************下面是气缸的函数***************/

  void shouqi(){
   digitalWrite(QIGAN,HIGH);
  }
  void tuikai(){
   digitalWrite(QIGAN,LOW);
  } 
  void qigan_init(){
    digitalWrite(QIGAN,HIGH);
  }

/****************************************************/

/***************下面是rfid模块的函数和参数定义*****************/

  HardwareSerial *ser;
  uint8_t rfid_encode(uint16_t id, uint8_t *buf, uint8_t *msg, uint8_t msg_len){
    uint8_t *p, *tmp, check=0;
    p = buf;
    *p++ = 0xAA;
    *p++ = 0xBB;
    *p = msg_len + 3;
    if(*p==0xAA){
      *++p = 0;
    }                      
    p++;
    *p++ = 0;
    *p = id>>8;
    check ^= *p;
    if(*p==0xAA){
      *++p = 0;
    }
    p++;
    *p = id & 0x00ff;
    check ^= *p;
    if(*p==0xAA){
      *++p = 0;
    }
    p++;
    for(tmp=msg;tmp-msg<msg_len;tmp++,p++){
      *p = *tmp;
      check ^= *p;
      if(*p==0xAA){
        *p = 0;
      }
    }
    *p++ = check;
    return p-buf;
  }
  int rfid_recv(uint8_t *msg){
    int recv,cnt;
    for(cnt=0;cnt<9;cnt++){
      recv = ser->read();
      if(recv==-1){
        ser->flush();
        return -1;
      }
      if(recv==0xAA&&cnt!=0) ser->read();
    }                                                              
    for(cnt=0;(recv=ser->read())!=-1;cnt++){
      msg[cnt] = recv;
      if(recv==0xAA) ser->read();
    }
    ser->flush();
    return cnt-1;
  }
  // 初始化Rfid模块，设置其工作状态为ISO15693协议
  void rfid_init(){
    int len;
    uint8_t buf[0x50], msg[] = {0x08,0x01,'1'};
    ser->begin(19200);
    len = rfid_encode(0,buf,msg,sizeof(msg));
    ser->write(buf,len);
    delay(200);
    rfid_recv(buf);
  }

  // 构造函数
  void Rfid(HardwareSerial &ser_in){
    ser = &ser_in;
    rfid_init();
  }
  
  // 读取电子标签的标签号，若标签不存在或失败返回-1，成功返回0，读到的9个字节放在data指针处
  int readID(uint8_t *data){
    int len;
    uint8_t msg[0x20] = {0x01,0x10}, buf[0x30];
    len = rfid_encode(0,buf,msg,2);
    ser->write(buf,len);                                                                   
    delay(35);
    len = rfid_recv(data);
    if(len!=9){
      return -1;
    }else{
      return 0;
    }
  }
  int writedata(uint8_t *data){
    int len;
    uint8_t msg[0x20] = {0x01,0x10}, buf[0x30];
    len = rfid_encode(0,buf,msg,2);
    ser->write(buf,len);
    delay(350);
    len = rfid_recv(msg+2);
    if(len!=9){
      return -1;
    }else{    
      msg[0] = 0x06; msg[1] = 0x10; msg[11] = 0; msg[12] = data[0];msg[13] = data[1];msg[14] = data[2];msg[15] = data[3];     ////////////////////不懂！！！！！！！！！！！！！！！！！
      len = rfid_encode(0,buf,msg,16);
      ser->write(buf,len);
      delay(450);
    }  
    return 0;
  }
  // 读取电子标签的前4个字节，若标签不存在或失败返回-1，成功返回0，读到的4个字节放在data指针处
  int read4bytes(uint8_t *data){
    int len;
    uint8_t msg[0x20] = {0x01,0x10}, buf[0x30];
    /////////////
    len = rfid_encode(0,buf,msg,2);
    ser->write(buf,len);
    delay(350);
    len = rfid_recv(msg+2);
    if(len!=9){
      return -1;
    }else{    
      msg[0] = 0x05; msg[1] = 0x10; msg[11] = 0; msg[12] = 1;    
      len = rfid_encode(0,buf,msg,13);
      ser->write(buf,len);
      delay(45);
      len = rfid_recv(msg);
      if(len!=4){
        return -1;
      }else{
        memcpy(data,msg,4);
        return 0;
      }
    }
  }
int ReadData(){
  Rfid(Serial1);                                                       
  uint8_t data[4];
  if(read4bytes(data)==-1){
    return 0;
  }else{
    if(data[0] == 0)//地上的标签
    {
      return (int)data[1];
    }
    else//物块上的标签
    {
      int label;
      label = (int)data[0];
      return label;
    }
 }
}


/*********************************************/

/***********场地初始化函数和定义***********/

int flag_detect[8];//是否已经检测的旗子 未检测的标记为0，已经检测的
//不是要拿的旗子标记为1 是要拿到低杆的旗子标记为2 是要拿到高杆的旗子
//标记为3
int flag_had_put[4]; //平台上是否放上了旗子 索引0是高台 0表示没放 1表示放了
int robo_claw_state; //机器爪的状态 0表示没有拿着旗子 1-8表示拿到的是哪一面旗子
int towards;//机器人当前的朝向 1:朝上 2：朝下 3：朝左 4：朝右
int point_now;//机器人当前所在位置
/***********************************/

//开局初始化函数
void zhengti_init(){
  for(int i=0;i<8;++i){
    flag_detect[i]=0;
  }  
  for(int i=0;i<4;++i){
    flag_had_put[i]=0;
  }
  point_now=1;
}

/***************下面时整体逻辑***************/
/*
 *因为场地是一个非常简单的图，所以根本没有必要做的太复杂，真正重要的路口是9，如果在1-7路口处，只需要直走到9，如果在8-11，也 
 *只要在这之间直走，重要的是走到一个地方后调整方向，机器人还需要知道自己走到了哪个地方
 */
int lujing[3];
//得到从一个点到另一个点的路径,a是初始位置，b是位置
void get_lujing(int a,int b){ 
  for(int i=0;i<3;++i){
    lujing[i]=0;
  }
  if(a<=7 && b<=7){ //当起点、终点都在取旗路口时，说明在第一个岔路口没有取到旗，现在要去第二个岔路口试探取旗
    lujing[0]=a;
    lujing[1]=b;
  }else if(a>=8 && b<=7){ //当起点在放旗路口，终点在取旗路口时，说明刚刚放旗旗完成，现在要去取旗
    lujing[0]=a;
    lujing[1]=9;
    lujing[2]=b;
  }else if(a<=7 && b>=8){ //当起点在取旗路口，终点在放旗路口时，说明刚刚取旗完成，现在要去放旗
    lujing[0]=a; 
    lujing[1]=9;
    lujing[2]=b;
  }
}

//根据路口数向后直走
void  go_back_by_count(int count){
  int first_come=1; //用来标记是否是第一次到达岔路口
  int side_come=0;  //用来标记是左边到了路口还是右边到了路口 左边用1表示 右边用2表示
  while(1){
    if(count==0){ //count等于0时说明已经到了终点，停下
      car_stop();
      break;
    }else{ //count不等于0时，说明还没有到终点，向后寻迹直走
      if( front_ss1==BLACK && front_ss0==1 && first_come==1){  //说明刚刚到达岔路口 且是左边
        count--;
        first_come=0;
        side_come=1;
      }else if(front_ss5==BLACK && front_ss6==1 && first_come==1){  //说明刚刚到达岔路口 且是右边
        count--;
        first_come=0;
        side_come=2;
      }else{ //向后寻迹直走
        if(front_ss0==0 && side_come==1){ //刚刚从岔路口出来 而且之前是左边有黑线
          first_come=1;
          side_come=0;
        }else if(front_ss6==0 && side_come==2){ //刚刚从岔路口出来 而且之前是右边有黑线
          first_come=1;
          side_come=0;
        }
        read_xunji();
        go_back_xunji();
        delay(100);
      }
    }
  }
}


//根据路口数向前直走
void  go_ahead_by_count(int count){
  int first_come=1; //用来标记是否是第一次到达岔路口
  int side_come=0;  //用来标记是左边到了路口还是右边到了路口 左边用1表示 右边用2表示
  while(1){
    if(count==0){ //count等于0时说明已经到了终点，停下
      car_stop();
    }else{ //count不等于0时，说明还没有到终点，向后寻迹直走
      if( front_ss1==BLACK && front_ss0==1 && first_come==1){  //说明刚刚到达岔路口 且是左边
        count--;
        first_come=0;
        side_come=1;
      }else if(front_ss5==BLACK && front_ss6==1 && first_come==1){  //说明刚刚到达岔路口 且是右边
        count--;
        first_come=0;
        side_come=2;
      }else{ //向后寻迹直走
        if(front_ss0==0 && side_come==1){ //刚刚从岔路口出来 而且之前是左边有黑线
          first_come=1;
          side_come=0;
        }else if(front_ss6==0 && side_come==2){ //刚刚从岔路口出来 而且之前是右边有黑线
          first_come=1;
          side_come=0;
        }
        read_xunji();
        go_ahead_xunji();
        delay(100);
      }
    }
  }
}

//根据得到的路径，完成行走，
void go_lujing(){
  if(lujing[2]==0){ //说明起点终点都在前面的口 
    if(lujing[0]<lujing[1]){ //说明要向后直走 其实这是一种不会发生的情况
      go_back_by_count(lujing[1]-lujing[0]);
      rotate_left_90();
      point_now=lujing[1];
    }else if(lujing[0]==lujing[1]){ //这种情况只有在最开始的时候才会出现 即在1号点的位置处 
      
    }
    else{ //要向前直走
      go_ahead_by_count(lujing[0]-lujing[1]);
      rotate_left_90();
      point_now=lujing[1];
    }
  }else if(lujing[0]>=8 && lujing[2]<=7 ){ //需要先向前直走到9号点 再向前走到终点 但是放旗的代码会自动回到9号点，所以按理说lujing[0]就是9
    go_ahead_by_count(8-lujing[2]);
    rotate_left_90();
    point_now=lujing[2];
  }else if(lujing[0]<=7 && lujing[2]>=8){ //先到9号点 再到终点
       go_back_by_count(8-point_now); //到达9号点
       point_now=9;
       if(lujing[2]==8){
          rotate_right_90();
          go_back_by_count(1);
          point_now=8; 
       }else if(lujing[2]==9){ //已经到了9号点 不需要继续走了
          
       }else if(lujing[2]==10){
           rotate_left_90();
           go_back_by_count(2);
           rotate_right_90();
           point_now=10;
       }else if(lujing[2]==11){
           rotate_left_90();
           go_back_by_count(2);
           rotate_right_90();
           point_now=11;
       }
  }
}
/******************************************/


/**********测试代码***********/

/*
 * 向低柱子接近并放旗测试代码，假设到了低柱子的岔路口且车的后面对着柱子（抱杆的那一面）,要通过寻迹和测距去放
 * 旗，最后后退回岔路口,并转到正确方向
 */
int go_and_put_flag_on_low_pillar_signal=0;
void go_and_put_flag_on_low_pillar(){
  while(1){
    if(go_and_put_flag_on_low_pillar_signal==0){// 第一阶段：靠近柱子后停止
      long dis=0;//距离
      for(int i=0;i<5;++i){//测5次距离后取平均值
        dis+=be_getdistance();
        delay(10);
      } 
      dis/=5;
      if(dis>=5 && dis<=8){//如果距离在这个范围内就停下来
      car_stop();
      go_and_put_flag_on_low_pillar_signal=1;
      }else{ //不在这个放范围内就(向后)直走寻迹
        read_xunji(); //更新寻迹模块
        go_back_xunji();
        delay(50); //行进50ms
      }
    }else if(go_and_put_flag_on_low_pillar_signal==1){// 第二阶段：放旗
       claw_fangqi();
       robo_claw_state=0; //重新标记机器爪状态
       go_and_put_flag_on_low_pillar_signal=2;
    }else if(go_and_put_flag_on_low_pillar_signal==2){//第三阶段：回到岔路口（前进）
        go_ahead_xunji_to_cross();
        go_and_put_flag_on_low_pillar_signal=3;
    }else if(go_and_put_flag_on_low_pillar_signal==3){//第四阶段：根据所处位置转动到正确方向
        if(point_now==8){ //对于低柱，如果是在9点那么不需要转动 如果是在8点 需要先右转 向前走到9点 再左转
          rotate_right_90();
          go_ahead_by_count(1);
          rotate_left_90();
          point_now=9; //现在到了9号点
        }
      break;
    }
  }
  go_and_put_flag_on_low_pillar_signal=0;
}
//向高柱子接近测试代码，假设到了柱子前的岔路口，现在要通过测距和寻迹来接近柱子
int go_to_high_pillar_signal=0;
void go_to_high_pillar(){
   long dis=0;//距离
   while(1){
      for(int i=0;i<5;++i){//测5次距离后取平均值
        dis+=be_getdistance();
        delay(10);
      } 
      dis/=5;
      if(dis>=5 && dis<=8){//如果距离在这个范围内就停下来
        car_stop();
        break;
      }else{ //不在这个放范围内就(向后)直走寻迹
        read_xunji(); //更新寻迹模块
        go_back_xunji();
        delay(50); //行进50ms
      }
   }
}
//爬杆的测试代码，假设已经到达了柱子下，开始抱杆爬杆,在爬一会儿后停下，再向下爬一段，停下，再退回（向前）到岔路口,然后再通过
int pagan_signal=1;
void pagan(){
  while(1){
    if(pagan_signal==1){//第一阶段 把气缸推开
      tuikai();
      delay(400);
      pagan_signal=2;
    }else if(pagan_signal==2){//第二阶段 向上爬杆，并不断测距，当接近顶端时停下
      long dis=0; //距离
      for(int i=0;i<5;++i){//测5次距离后取平均值
        dis+=up_getdistance();
        delay(10);
      } 
      dis/=5;
      if(dis>=22 && dis<=24){ //如果测得距离在这个范围内，就停下保住杆
        pagan_stop();
        pagan_signal=3;
        delay(100);
      }else{
        pagan_up();
        delay(200); //向上爬行200ms
      }
    }else if(pagan_signal==3){ //放旗
      claw_fangqi();
      robo_claw_state=0; //放完旗后 重新标记机器爪状态
      pagan_signal=4;
      delay(500);
    }else if(pagan_signal==4){ //向下爬杆,并不断检测与上部柱子的距离，当达到一定程度后就停下，收起气缸
      long dis=0; //距离
      for(int i=0;i<5;++i){//测5次距离后取平均值
        dis+=up_getdistance();
        delay(10);
      } 
      dis/=5;
      if(dis>=105 && dis<=115){ 
        pagan_stop();
        delay(200);
        shouqi();
        delay(200);
        pagan_signal=5;
      }else{
        pagan_down();
      }
    }else if(pagan_signal==5){
       go_ahead_xunji_to_cross();
       pagan_signal=6;
    }else if(pagan_signal==6){
      if(point_now==10){
         rotate_left_90();
         go_ahead_by_count(1);
         rotate_right_90();
      }else if(point_now==11){
         rotate_left_90();
         go_ahead_by_count(2);
         rotate_right_90();
      }
      point_now=9;
      break;
    }
  }
   pagan_signal=1;
}
//从岔路口开始去取旗，并在取旗后退回岔路口，转向
void go_take_flag(){
  int dis; //距离
  while(1){ //首先向正方体柱子靠近
    dis=distance();
    if(dis<=140 && dis >=100){ //达到一定的距离时 根据rfid来判断要不要取旗取旗
      car_stop(); 
      int rfid_number;
      for(int i=0;i<7;++i){
        rfid_number=ReadData();
        if(rfid_number>=5){
          break;
        }
        delay(20);
      }
      if(rfid_number>=5){ //暂时假设我们在蓝方场地 需要取理实交融四面旗
        claw_quqi();
        robo_claw_state=rfid_number; //标记此时机器人手中的旗
      }
      if(rfid_number==5 || rfid_number==6){ //标记该处旗子已读 且为哪种类型
        flag_detect[point_now-1]=3;
      }else if(rfid_number==7 || rfid_number==8){
        flag_detect[point_now-1]=2;
      }else{
        flag_detect[point_now-1]=1;  
      }
      break;
    }else{
      read_xunji();
      go_ahead_xunji();
      delay(50);
    }
  }
    go_back_xunji_to_cross(); //返回岔路口
    rotate_right_90(); //调整方向
}
/***********************/

void setup() {
  /*
  * 在setup阶段完成所有的初始化，包括爪子部分初始化、启动部分初始化
  * 场地初始化
  */
   Wire.begin();  
   Serial.begin(9600);
   pinMode(FRONT_SS1,INPUT);
   pinMode(FRONT_SS2,INPUT);
   pinMode(FRONT_SS3,INPUT);
   pinMode(FRONT_SS4,INPUT);
   pinMode(FRONT_SS5,INPUT);
   pinMode(LEFT_SS1,INPUT);
   pinMode(LEFT_SS2,INPUT);
   pinMode(LEFT_SS3,INPUT);
   pinMode(RIGHT_SS1,INPUT);
   pinMode(RIGHT_SS2,INPUT);
   pinMode(RIGHT_SS3,INPUT);
   pinMode(DIANJI_LEFT_SPEED,OUTPUT);
   pinMode(DIANJI_LEFT_TOWARDS,OUTPUT);
   pinMode(DIANJI_RIGHT_SPEED,OUTPUT);
   pinMode(DIANJI_RIGHT_TOWARDS,OUTPUT);
   pinMode(DIANJI_CENTER_SPEED,OUTPUT);
   pinMode(DIANJI_CENTER_TOWARDS,OUTPUT);
   pinMode(QIGAN,OUTPUT);
   pinMode(WOLUN_1,OUTPUT);
   pinMode(WOLUN_2,OUTPUT);
   pinMode(CLAW,OUTPUT);
   pinMode(ROTATE,OUTPUT);
   pinMode(STEER,OUTPUT);
   
   /*
    * 场地初始化 
    */
    zhengti_init();
    
    /*
     * 爪子部分初始化函数 claw_init() 
    */
    claw_init();
   
    /*
     * 气缸初始化函数 
     */
     delay(1000);
}

/*
int flag_detect[8];//是否已经检测的旗子 未检测的标记为0，已经检测的
//不是要拿的旗子标记为1 是要拿到低杆的旗子标记为2 是要拿到高杆的旗子
//标记为3
int flag_had_put[4]; //平台上是否放上了旗子 索引0是高台 0表示没放 1表示放了
int robo_claw_state; //机器爪的状态 0表示没有拿着旗子 1-8表示拿到的是哪一面旗子
int towards;//机器人当前的朝向 1:朝上 2：朝下 3：朝左 4：朝右
int point_now;//机器人当前所在位置
*/

 
int zhengti_signal=0;
void loop() {
  Serial.println(ReadData());
  delay(500);
  /*************下面是整体的逻辑***********/
  
  if(zhengti_signal==0){  //阶段一 先走到第一个岔路口
    go_ahead_xunji_to_cross();
    zhengti_signal=1;
  }else if(zhengti_signal==1){ //阶段二 不断去试探 直到柱子上放满了旗子
    while(1){
      if(flag_had_put[0]==1 && flag_had_put[1]==1 && flag_had_put[2]==1 && flag_had_put[3]==1){ //完成了所有的任务
        zhengti_signal=2;
        break;
      }else{ //这个状态总是从手里没旗开始的
        int point_dest; //目的地
        for(int i=0;i<8;++i){ //去寻找最小的没有被检测的点
          if(flag_detect[i]==0){
              point_dest=i+1;
              break;
          }
        }
        get_lujing(point_now,point_dest); //获得路径
        go_lujing(); //到达目的地
        go_take_flag(); //取旗
        if(robo_claw_state!=0){ //不等于0 说明拿到了旗子 那么要去放旗子啦！
          if(robo_claw_state==5){ //拿到了 “理”
              get_lujing(point_now,11); //获得路径
              go_lujing(); //到达目的地
              go_to_high_pillar();
              pagan();
          }else if(robo_claw_state==6){ //拿到了 “实”
              get_lujing(point_now,10); //获得路径
              go_lujing(); //到达目的地
              go_to_high_pillar();
              pagan();
          }else if(robo_claw_state==7){ //拿到了 “交”
              get_lujing(point_now,9); //获得路径
              go_lujing(); //到达目的地
              go_and_put_flag_on_low_pillar();
          }else if(robo_claw_state==8){ //拿到了 “融”
              get_lujing(point_now,8); //获得路径
              go_lujing(); //到达目的地
              go_and_put_flag_on_low_pillar();
          }
        }
      }
    }
  }
  /***********************************/
}
