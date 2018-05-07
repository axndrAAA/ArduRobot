#include <Arduino.h>
#include <Wire.h>
#include"botMovingManegment.h"
#include"HMC5883L_Simple.h"

BotMovingManegement::BotMovingManegement(){
    pinMode(dir2,OUTPUT);
    pinMode(dir3,OUTPUT);
    pinMode(pwm2,OUTPUT);
    pinMode(pwm3,OUTPUT);
    
}

BotMovingManegement::BotMovingManegement(float (*get_cur_ang)(void)):BotMovingManegement(){
    get_ang_func = get_cur_ang;
}


void BotMovingManegement::turnLeft(){
    analogWrite (pwm2, LOW);
    analogWrite (pwm3, LOW);        
    delay(5);
    digitalWrite(dir2,LOW);
    digitalWrite(dir3,HIGH);
    analogWrite (pwm2, V);
    analogWrite (pwm3, V); 
    Serial.println("left");    
}


void BotMovingManegement::turnRight(){
    analogWrite (pwm3, LOW);
    delay(5);
    digitalWrite(dir2,HIGH);
    digitalWrite(dir3,LOW); 
    analogWrite (pwm2, V);
    analogWrite (pwm3, V);
    Serial.println("right");
}



void BotMovingManegement::goForward(){
        analogWrite (pwm2, LOW);
        analogWrite (pwm3, LOW);       
        delay(5);        
        digitalWrite(dir2,HIGH);
        digitalWrite(dir3,HIGH);
        analogWrite (pwm2, V);
        analogWrite (pwm3, V);
        
}


void BotMovingManegement::stop(){
        analogWrite (pwm2, LOW);
        analogWrite (pwm3, LOW);
}


void BotMovingManegement::goBackward(){
        analogWrite (pwm2, LOW);
        analogWrite (pwm3, LOW);
        delay(5);
        digitalWrite(dir2,LOW);
        digitalWrite(dir3,LOW);
        analogWrite (pwm2, V);
        analogWrite (pwm3, V); 
}

void BotMovingManegement::setV(int _v){
    V = _v;
}

float BotMovingManegement::getHeadingHQ(){
    float res = (*get_ang_func)();
    return res;
}


void BotMovingManegement::turnAngle(int new_angle){
    float last_heading = getHeadingHQ();
  int delta = new_angle - last_heading;
  if(abs(delta)<EPS){
    delta = 0;
    return;
  }
  float pid_val = Kp*delta;

    do{
        delay(500);      
        last_heading = getHeadingHQ();
      
        delta = new_angle - last_heading;
         Serial.print("delta: \t");
         Serial.println(delta);
        if(abs(delta)<EPS){
            delta = 0;
            break;
        }
        pid_val = Kp*abs(delta);
        Serial.print("PID: \t");
        Serial.println(pid_val);
        map(pid_val,0,MAX_PID_VAL,0,MAX_PWM_VAL);
        Serial.print("PWM: \t");
        Serial.println(pid_val);
        setV(pid_val);


        if(delta < 0){
          //влево    
          if(-delta >= 180){
            //все таки вправо
              pid_val = Kp*abs(delta-360);
              // Serial.print("PID: \t");
              // Serial.println(pid_val);
              map(pid_val,0,MAX_PID_VAL,0,MAX_PWM_VAL);
              // Serial.print("PWM: \t");
              // Serial.println(pid_val);
              setV(pid_val);
            turnRight();
            continue;
          }
          turnLeft();
        }else {
            //вправо
          if(delta >= 180){
            //все таки влево
            pid_val = Kp*abs(delta-360);
            // Serial.print("PID: \t");
            // Serial.println(pid_val);
            map(pid_val,0,MAX_PID_VAL,0,MAX_PWM_VAL);
            // Serial.print("PWM: \t");
            // Serial.println(pid_val);
            setV(pid_val);
            turnLeft();
            continue;
          }
          turnRight();
        }              

        }while(abs(delta) > EPS);
        stop();
}
