#include"botMovingManegment.h"
#include"Arduino.h"
BotMovingManegement::BotMovingManegement(){
    pinMode(dir2,OUTPUT);
    pinMode(dir3,OUTPUT);
    pinMode(pwm2,OUTPUT);
    pinMode(pwm3,OUTPUT);
}

void BotMovingManegement::turnLeft(){
    analogWrite (pwm2, LOW);
    analogWrite (pwm3, LOW);        
    delay(5);
    digitalWrite(dir2,LOW);
    digitalWrite(dir3,HIGH);
    analogWrite (pwm2, V);
    analogWrite (pwm3, V); 
}


void BotMovingManegement::turnLeftRight(){
    analogWrite (pwm3, LOW);
    delay(5);
    digitalWrite(dir2,HIGH);
    digitalWrite(dir3,LOW); 
    analogWrite (pwm2, V);
    analogWrite (pwm3, V);
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
