#include <PID_v1.h>
#include <FutabaSBUS.h>
#include<Servo.h>
FutabaSBUS sbus;
Servo yaw;
Servo roll;
int yawval, rollval;
int rcsig[25];
const int laser = 26;
const int video = 28;
const int ejector = 30;
bool rev1, rev2, rev3, rev4;//����
//-----------------
int mafor, mago, maturn;
int mbfor, mbgo, mbturn;
int mcfor, mcgo, mcturn;
int mdfor, mdgo, mdturn;
int speed_max = 110;
int speed_min = -110;
//-----------------4�������ת,ƽ��,ǰ��ֵ
const int L = 1015, R = 1035;
double M1PWMOUT, M2PWMOUT, M3PWMOUT, M4PWMOUT;//M1���pwm��M2���pwm���Դ�����
double ref1, ref2, ref3, ref4;//�ĸ�����Ĳο�ת��
double in1, in2, in3, in4, M1S, M2S, M3S, M4S;//in1-4������ĸ�����M1-4S��ת����ת�٣�û�е�λ�����ֵ
double Kp = 5, Ki = 10, Kd = 0; //PIDϵ��
unsigned long t;
PID M1PID(&M1S, &M1PWMOUT, &ref1, Kp, Ki, Kd, DIRECT); //����PID��
PID M2PID(&M2S, &M2PWMOUT, &ref2, Kp, Ki, Kd, DIRECT);
PID M3PID(&M3S, &M3PWMOUT, &ref3, Kp, Ki, Kd, DIRECT);
PID M4PID(&M4S, &M4PWMOUT, &ref4, Kp, Ki, Kd, DIRECT);
//-----------------------
struct motor
{
  int in1;
  int in2;
  int pwm;
  int A_out;
  int B_out;
};
motor M1, M2, M3, M4;
void dataReceived(ChannelData channels) {
  // do something with the data
  for (int i = 1; i <= 16; i++)
  {
    rcsig[i] = channels.data[i - 1];
  }
  rcsig[17] = channels.channels.channel17;
  rcsig[18] = channels.channels.channel18;
}

void Speed()
{
  GetM1Speed();
  GetM2Speed();
  GetM3Speed();
  GetM4Speed();
}
//--------�����ٶ�
void moving()//�ƶ�
{
  if (M1PWMOUT > 0)
  {
    analogWrite(M1.pwm, M1PWMOUT);
    digitalWrite(M1.in1, 1);
    digitalWrite(M1.in2, 0);
  }
  else
  {
    analogWrite(M1.pwm, abs(M1PWMOUT));
    digitalWrite(M1.in1, 0);
    digitalWrite(M1.in2, 1);
  }
  if (M2PWMOUT > 0)
  {
    analogWrite(M2.pwm, M2PWMOUT);
    digitalWrite(M2.in1, 1);
    digitalWrite(M2.in2, 0);
  }
  else
  {
    analogWrite(M2.pwm, abs(M2PWMOUT));
    digitalWrite(M2.in1, 0);
    digitalWrite(M2.in2, 1);
  }
  if (M3PWMOUT > 0)
  {
    analogWrite(M3.pwm, M3PWMOUT);
    digitalWrite(M3.in1, 1);
    digitalWrite(M3.in2, 0);
  }
  else
  {
    analogWrite(M3.pwm, abs(M3PWMOUT));
    digitalWrite(M3.in1, 0);
    digitalWrite(M3.in2, 1);
  }
  if (M4PWMOUT > 0)
  {
    analogWrite(M4.pwm, M4PWMOUT);
    digitalWrite(M4.in1, 1);
    digitalWrite(M4.in2, 0);
  }
  else
  {
    analogWrite(M4.pwm, abs(M4PWMOUT));
    digitalWrite(M4.in1, 0);
    digitalWrite(M4.in2, 1);
  }
  //Serial.println("moved");
}
void fail_safe()
{
  stop_all();
}
//-------------------------�ƶ�
void stop_all()//ֹͣ
{
  digitalWrite(M1.in1, 0);
  digitalWrite(M2.in1, 0);
  digitalWrite(M3.in1, 0);
  digitalWrite(M4.in1, 0);
  digitalWrite(M1.in2, 0);
  digitalWrite(M2.in2, 0);
  digitalWrite(M3.in2, 0);
  digitalWrite(M4.in2, 0);
  M1PWMOUT = 0;
  M2PWMOUT = 0;
  M3PWMOUT = 0;
  M4PWMOUT = 0;
  ref1 = 0;
  ref2 = 0;
  ref3 = 0;
  ref4 = 0;
}
/**********ˢת��*****************/
void GetM1Speed()//ˢת��
{
  if (rev1)
    M1S = in1;//M1S���in1
  else
    M1S = -in1;
  in1 = 0;//����ٶȽ�������㣬��¼��һ��Ĵ�������
}
void GetM2Speed()
{
  if (rev2)
    M2S = in2;
  else
    M2S = -in2;
  in2 = 0;//����ٶȽ�������㣬��¼��һ��Ĵ�������
}
void GetM3Speed()
{
  if (rev3)
    M3S = in3;
  else
    M3S = -in3;
  in3 = 0;//����ٶȽ�������㣬��¼��һ��Ĵ�������
}
void GetM4Speed()
{
  if (rev4)
    M4S = in4;
  else
    M4S = -in4;
  in4 = 0;//����ٶȽ�������㣬��¼��һ��Ĵ�������
}
/***********************************/
void setspeed()//PID���㣬�ó����PWM
{
  M1PID.Compute();
  M2PID.Compute();
  M3PID.Compute();
  M4PID.Compute();
}
void buzzer(int T)
{
  digitalWrite(35, 1);
  delay(T);
  digitalWrite(35, 0);
}
void setup()
{
  M1.A_out = 18;
  M2.A_out = 19;
  M3.A_out = 20;
  M4.A_out = 21;
  M1.B_out = 39;
  M2.B_out = 36;
  M3.B_out = 34;
  M4.B_out = 32;
  M1.in1 = 40;
  M1.in2 = 42;
  M2.in1 = 44;
  M2.in2 = 46;
  M3.in1 = 48;
  M3.in2 = 50;
  M4.in1 = 51;
  M4.in2 = 52;
  M1.pwm = 2;
  M2.pwm = 3;
  M3.pwm = 4;
  M4.pwm = 5;
  pinMode(M1.pwm, OUTPUT);
  pinMode(M2.pwm, OUTPUT);
  pinMode(M3.pwm, OUTPUT);
  pinMode(M4.pwm, OUTPUT);
  pinMode(M1.in1, OUTPUT);
  pinMode(M1.in2, OUTPUT);
  pinMode(M2.in1, OUTPUT);
  pinMode(M2.in2, OUTPUT);
  pinMode(M3.in1, OUTPUT);
  pinMode(M3.in2, OUTPUT);
  pinMode(M4.in1, OUTPUT);
  pinMode(M4.in2, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(laser, OUTPUT);
  pinMode(video, OUTPUT);
  pinMode(M1.B_out, INPUT);
  pinMode(M2.B_out, INPUT);
  pinMode(M3.B_out, INPUT);
  pinMode(M4.B_out, INPUT);
  pinMode(M1.A_out, INPUT);
  pinMode(M2.A_out, INPUT);
  pinMode(M3.A_out, INPUT);
  pinMode(M4.A_out, INPUT);
  pinMode(35, OUTPUT);
  /************/
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(M1.A_out), macount, FALLING);//�����źű����Ǳ仯�ģ��������½��Կɣ��ⲿ�ж϶�ת�٣�
  attachInterrupt(digitalPinToInterrupt(M2.A_out), mbcount, FALLING);//�����źű����Ǳ仯�ģ��������½��Կ�
  attachInterrupt(digitalPinToInterrupt(M3.A_out), mccount, FALLING);//�����źű����Ǳ仯�ģ��������½��Կ�
  attachInterrupt(digitalPinToInterrupt(M4.A_out), mdcount, FALLING);//�����źű����Ǳ仯�ģ��������½��Կ�
  M1PID.SetMode(AUTOMATIC);//����PIDΪ�Զ�ģʽ
  M1PID.SetSampleTime(30);//����PID����Ƶ��Ϊ100ms
  M2PID.SetMode(AUTOMATIC);//����PIDΪ�Զ�ģʽ
  M2PID.SetSampleTime(30);//����PID����Ƶ��Ϊ100ms
  M3PID.SetMode(AUTOMATIC);//����PIDΪ�Զ�ģʽ
  M3PID.SetSampleTime(30);//����PID����Ƶ��Ϊ100ms
  M4PID.SetMode(AUTOMATIC);//����PIDΪ�Զ�ģʽ
  M4PID.SetSampleTime(30);//����PID����Ƶ��Ϊ100ms
  M1PID.SetOutputLimits(-255, 255);//PID���ֵ����Ϊ-255~255
  M2PID.SetOutputLimits(-255, 255);
  M3PID.SetOutputLimits(-255, 255);
  M4PID.SetOutputLimits(-255, 255);
  yaw.attach(22);
  roll.attach(24);
  buzzer(100);
  delay(100);
  buzzer(100);
  delay(100);
  buzzer(200);
  sbus.begin(Serial2);
  sbus.attachDataReceived(dataReceived);
  t = millis();
}
void loop()
{
  sbus.receive();
  if (rcsig[1] == 0)
    fail_safe();
  else
  {
    if (millis() > t)
    {
      Speed();
      t = millis() + 30;
    }
    yawval = map(rcsig[3], 340, 1708, 0, 180);
    rollval = map(rcsig[6], 340, 1708, 0, 180);
    yaw.write(yawval);
    roll.write(rollval);
    if (rcsig[5] > 1024)//���Ʒ���
      digitalWrite(ejector, HIGH);//����
    else
      digitalWrite(ejector, LOW);//������
    if (rcsig[8] > 1024)
      digitalWrite(laser, HIGH);
    else
      digitalWrite(laser, LOW);
    if (rcsig[9] > 1024)
      digitalWrite(video, HIGH);
    else
      digitalWrite(video, LOW);
    if (!((rcsig[2] > L) && (rcsig[2] < R)))
    {
      mafor = map(rcsig[2], 340, 1708, speed_min, speed_max);
      mbfor = map(rcsig[2], 340, 1708, speed_min, speed_max);
      mcfor = map(rcsig[2], 340, 1708, speed_min, speed_max);
      mdfor = map(rcsig[2], 340, 1708, speed_min, speed_max);
    }
    else
    {
      mafor = 0;
      mbfor = 0;
      mcfor = 0;
      mdfor = 0;
    }
    if (!((rcsig[1] > L) && (rcsig[1] < R)))
    {
      mago = map(rcsig[1], 340, 1708, speed_min, speed_max);
      mbgo = map(rcsig[1], 340, 1708, speed_max, speed_min);
      mcgo = map(rcsig[1], 340, 1708, speed_min, speed_max);
      mdgo = map(rcsig[1], 340, 1708, speed_max, speed_min);
    }
    else
    {
      mago = 0;
      mbgo = 0;
      mcgo = 0;
      mdgo = 0;
    }
    if (!((rcsig[4] > L) && (rcsig[4] < R)))
    {
      maturn = map(rcsig[4], 340, 1708, speed_min, speed_max);
      mbturn = map(rcsig[4], 340, 1708, speed_min, speed_max);
      mcturn = map(rcsig[4], 340, 1708, speed_max, speed_min);
      mdturn = map(rcsig[4], 340, 1708, speed_max, speed_min);
    }
    else
    {
      maturn = 0;
      mbturn = 0;
      mcturn = 0;
      mdturn = 0;
    }
    ref1 = max(min(mafor + mago + maturn, speed_max), speed_min);
    ref2 = max(min(mbfor + mbgo + mbturn, speed_max), speed_min);
    ref3 = max(min(mcfor + mcgo + mcturn, speed_max), speed_min);
    ref4 = max(min(mdfor + mdgo + mdturn, speed_max), speed_min);
    if ((rcsig[1] > L) && (rcsig[1] < R) && (rcsig[2] > L) && (rcsig[2] < R) && (rcsig[4] > L) && (rcsig[4] < R))
      stop_all();
    setspeed();
    moving();
  }

}
void macount()//ת�ټ�һ
{
  rev1 = digitalRead(M1.B_out);
  in1++;
}
void mbcount()
{
  rev2 = digitalRead(M2.B_out);
  in2++;
}
void mccount()
{
  rev3 = !digitalRead(M3.B_out);
  in3++;
}
void mdcount()
{
  rev4 = !digitalRead(M4.B_out);
  in4++;
}
