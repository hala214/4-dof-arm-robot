#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm;

#define SERVO1_MIN 72
#define SERVO1_MAX 520

#define SERVO2_MIN 200
#define SERVO2_MAX 510

#define SERVO3_MIN 480
#define SERVO3_MAX 80

#define SERVO4_MIN 95
#define SERVO4_MAX 500

int angleToPulse1(int a){return map(a,0,180,SERVO1_MIN,SERVO1_MAX);}
int angleToPulse2(int a){return map(a,0,180,SERVO2_MIN,SERVO2_MAX);}
int angleToPulse3(int a){return map(a,0,180,SERVO3_MIN,SERVO3_MAX);}
int angleToPulse4(int a){return map(a,0,180,SERVO4_MIN,SERVO4_MAX);}

void setup(){
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50);
}

void loop(){
  if(Serial.available()){
    String data=Serial.readStringUntil('\n');
    data.trim();

    int i1=data.indexOf(',');
    int i2=data.indexOf(',',i1+1);
    int i3=data.indexOf(',',i2+1);

    if(i1>0 && i2>0 && i3>0){
      float q1=data.substring(0,i1).toFloat();
      float q2=data.substring(i1+1,i2).toFloat();
      float q3=data.substring(i2+1,i3).toFloat();
      float q4=data.substring(i3+1).toFloat();

      pwm.setPWM(0,0,angleToPulse1(q1));
      pwm.setPWM(1,0,angleToPulse2(q2+20));
      pwm.setPWM(2,0,angleToPulse3(q3+180));
      pwm.setPWM(5,0,angleToPulse4(q4+55));
      pwm.setPWM(4,0,290);
    }
  }
}
