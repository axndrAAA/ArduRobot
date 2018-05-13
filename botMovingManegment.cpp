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
    //Serial.println("left");    
}


void BotMovingManegement::turnRight(){
    analogWrite (pwm3, LOW);
    delay(5);
    digitalWrite(dir2,HIGH);
    digitalWrite(dir3,LOW); 
    analogWrite (pwm2, V);
    analogWrite (pwm3, V);
    //Serial.println("right");
}



void BotMovingManegement::goForward(){
    analogWrite (pwm2, LOW);
    analogWrite (pwm3, LOW);
    delay(5);
    digitalWrite(dir2,LOW);
    digitalWrite(dir3,LOW);
    analogWrite (pwm2, V);
    analogWrite (pwm3, V);
    //Serial.println("forward");        
        
}


void BotMovingManegement::stop(){
    analogWrite (pwm2, LOW);
    analogWrite (pwm3, LOW);
    //Serial.println("stop");
}


void BotMovingManegement::goBackward(){
        analogWrite (pwm2, LOW);
        analogWrite (pwm3, LOW);       
        delay(5);        
        digitalWrite(dir2,HIGH);
        digitalWrite(dir3,HIGH);
        analogWrite (pwm2, V);
        analogWrite (pwm3, V);
    //Serial.println("back");
         
}

void BotMovingManegement::setV(int _v){
    if(_v < MIN_V_PWM){
        V = MIN_V_PWM;        
        return;
    }
    if(_v > MAX_PWM_VAL){
        V = MAX_PWM_VAL;
        return;
    }
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
        //delay(500);      
        last_heading = getHeadingHQ();
      
        delta = new_angle - last_heading;
         Serial.print("delta: \t");
         Serial.println(delta);
        if(abs(delta)<ANG_EPS){
            delta = 0;
            break;
        }
        pid_val = Kp*abs(delta);
        Serial.print("PID: \t");
        Serial.println(pid_val);
        map(pid_val,0,MAX_ANG_PID_VAL,0,MAX_PWM_VAL);
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
              map(pid_val,0,MAX_ANG_PID_VAL,0,MAX_PWM_VAL);
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
            map(pid_val,0,MAX_ANG_PID_VAL,0,MAX_PWM_VAL);
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

void BotMovingManegement::getMessage(String &command){
    command = "";
    if(Mode == 0){
        command +="b1/";
    }else{
        command +="b2/";
    }

    command += "0/";

    int azimut = getHeadingHQ();
    command += String(azimut) + "/e";
}

void BotMovingManegement::getCoordFromComma(const String &command,int *coords){
    //command mode2 = b2/XXX/YYY/XXXt/YYYt/e
    int last_indx = 3;
    int counter = 0;
    String piece;
    
    //начинаем с 2, т.к. нужны только координаты
      for (uint8_t i = last_indx; i < command.length()-1; i++) {
        // Loop through each character and check if it's a comma
        if (command.substring(i, i+1) == "/") {
        	// Grab the piece from the last index up to the current position and store it
        	piece = command.substring(last_indx, i);
            //преобразовываем в int  и добавляем в выходной массив 
            coords[counter] = piece.toInt();
        	// Update the last position and add 1, so it starts from the next character
        	last_indx = i + 1;
        	// Increase the position in the array that we store into
        	counter++;
        }
      }  
}

float BotMovingManegement::getVectAngle(float Xr,float Yr){    
    //нормализаця
    //модуль
    float r_mod = sqrt(pow(Xr,2)+pow(Yr,2));
    //нормализаця
    Yr = (Yr)/r_mod;
    Xr = (Xr)/r_mod;

    //тангенс в радианах
    float point_azim = atan2(Yr,Xr);
      // Correct for when signs are reversed.
    if (point_azim < 0)
        point_azim += 2 * M_PI;

    // Check for wrap due to addition of declination.
    if (point_azim > 2 * M_PI)
        point_azim -= 2 * M_PI;

    //перевод в градусы
    point_azim = point_azim*180.0/M_PI;
    return point_azim;    
}


void BotMovingManegement::executeModeCommand(const String &command){
    //command mode1 = b1/112/e
    String key = command.substring(0,2);
    //Serial.println(key);
    if(key.equals("b1")){
        Mode = 0;
        //Serial.println("Mode 0");        
        mode1Execute(command);
    }else if(key.equals("b2")){
        Mode = 1;
        //Serial.println("Mode 1");        
        mode2Execute(command);      
    }    

}

void BotMovingManegement::mode1Execute(const String &command){
    //command = b1/112/e
    stop();
    String ch1 = command.substring(3,4);
    String ch2 = command.substring(4,5);
    String ch3 = command.substring(5,6);

    int v = ch3.toInt();
    V = map(v,0,9,0,MAX_PWM_VAL);
    //сначала поворачиваем
    if(!ch2.equals("2")){
        if(ch2.equals("3")){
            //поворот вправо 
            turnRight();            
          }
         if(ch2.equals("1")){
            //поворот влево 
            turnLeft();        
          }
    }else{
        //теперь движение назад/вперед
        if(ch1.equals("1")){
              //назад
              goBackward();
              return;              
          }
        if(ch1.equals("3")){
              //вперед 
              goForward();
              return;
        }
        //если ниодно из условий не выполнено - остановка
        stop();
    }
}

void BotMovingManegement::mode2Execute(const String &command){
    //command mode2 = b2/X/Y/Xt/Yt/e
    //где
    //X,Y - текущие координаты бота (положительные числа [0,999])
    //Xt,Yt - координаты точки назначения бота(положительные числа [0,999])

    int coords[4];
    //X[0] = X
    //X[1] = Y
    //X[2] = Xt
    //X[3] = Yt

    //парсинг строки на координаты
    getCoordFromComma(command,coords);

    //координаты радиус-вектора точки назначения
    float Xr = coords[2] - coords[0];
    float Yr = coords[3] - coords[1];

    //получение азимута точки
    float point_azim = getVectAngle(Xr,Yr);

    //debug
    Serial.print("ang: ");    
    Serial.println(point_azim);    

    //разворот на точку
    //debug
    turnAngle(point_azim);

    //вычисление расстояния до точки назначения
    float r_mod = sqrt(pow(Xr,2)+pow(Yr,2));

    Serial.print("r_mod: ");    
    Serial.println(r_mod);  

    if(r_mod >= coordEps){
        //мы на в точке - поэтому разваорачиваемся и едем вперед

        //внутри есть отсекатель по минимальной ошибке,
        // поэтому повторный вызов функции разворота ни на что не повлияет
        turnAngle(point_azim);

        //Выставляем скорость пропорциональную расстоянию от бота до точки
        // map(r_mod,0,MAX_COORD_PID_VAL,0,MAX_PWM_VAL);  
        setV(FORWARD_V);
        Serial.print("Speed: ");
        Serial.println(V);

        //едем вперед
        //goForward();
        Serial.println("едем");

    }else{
        //прибыли
        stop();
    }
}


