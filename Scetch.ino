#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#include "botMovingManegment.h"
#include "HMC5883L_Simple.h"
#include "ESP8266.h"


// Create a compass
HMC5883L_Simple Compass;

//нуль компаса(направление на север в аудитории)
#define COMPASS_ZERO 315

//функця связи компаса и bmm
float get_ang_func(){
  return Compass.GetHeadingDegreesHQ();
}

//Создаем класс движения бота
BotMovingManegement bmm(&get_ang_func);

//параметры подключения к вафле и создания TCP-IP соединения
#define SSID        "SVR-RobotAccess"
#define PASSWORD    "RobotPassword"
#define HOST_NAME   "192.168.137.1"
#define HOST_PORT   (777)

//сюда подключен esp8266
SoftwareSerial wifiSerial(11,12);

//создание wif
ESP8266 wifi(wifiSerial,9600);

//таймаут ответа хоста (перезапуск TCP-соединения) в мсек
#define TIMEOUT 2000

//буфер принимаемых команд
uint8_t buffer[128] = {0};

//буфер формирования команды
String command = "";

//счетчик циклов для проверки состояния соединения
unsigned loopCounter = 0;

//каждые MAX_REF_TCP_COUTER циклов будет происходить попытка перезапуска соединения
#define MAX_REF_TCP_COUTER 650

void setup()
{
  //I2C для компаса
  Wire.begin();
  //настройка компаса
  Compass.SetDeclination(23, 35, 'E');
  Compass.SetSamplingMode(COMPASS_SINGLE);
  Compass.SetScale(COMPASS_SCALE_130);
  Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);
  //Compass.setUpZeroHeading();
  Compass.setZeroHeading(COMPASS_ZERO);

  // //настройка вафли
  // //для отладки
  Serial.begin(9600);
  Serial.print("setup begin\r\n");
  Serial.print("FW Version: ");
  Serial.println(wifi.getVersion().c_str());

  //переключение вайли в режим станции
  if (wifi.setOprToStationSoftAP()) {
    Serial.print("to station + softap ok\r\n");
  } else {
    Serial.print("to station + softap err\r\n");
  }

  //подключение к точке доступа
  if (wifi.joinAP(SSID, PASSWORD)) {
    Serial.print("Join AP success\r\n");
    Serial.print("IP: ");
    Serial.println(wifi.getLocalIP().c_str());
  } else {
      Serial.print("Join AP failure\r\n");
  }

  //включаем одиночный режим (выключаем MUX-режим)
  if (wifi.disableMUX()) {
    Serial.print("single ok\r\n");
  } else {
    Serial.print("single err\r\n");
  }

  //создание TCP соединения
  if (wifi.createTCP(HOST_NAME, HOST_PORT)) {
    Serial.print("create tcp ok\r\n");
  } else {
    Serial.print("create tcp err\r\n");
  }
  

  Serial.println("Ready.");
  digitalWrite(LED_BUILTIN, HIGH);
}

// Our main program loop.
void loop()
{

  

  // bmm.executeModeCommand("b2/0/0/-100/100/e");
  // //bmm.turnAngle(90);

  // delay(1500);

  //восстанавливаем соединение, если оно упало
  // if(loopCounter > MAX_REF_TCP_COUTER){
  //     if (wifi.createTCP(HOST_NAME, HOST_PORT)) {
  //         Serial.print("create tcp ok\r\n");
  //         digitalWrite(LED_BUILTIN, HIGH);
  //     }else {
  //         Serial.print("create tcp err\r\n");
  //         digitalWrite(LED_BUILTIN, LOW);
  //     }
  //     loopCounter = 0;
  // }

  //читаем команду от хоста
  uint32_t len = wifi.recv(buffer, sizeof(buffer), TIMEOUT);

    if (len > 0) {
        //Serial.print("Received:[");
        for(uint32_t i = 0; i < len; i++) {
            //Serial.print((char)buffer[i]);
            command+=(char)buffer[i];
        }

        //исполнение полученной команды
        bmm.executeModeCommand(command);

        //получение строки-состояния для отправки на хост
        bmm.getMessage(command);
        // Serial.println(command);       
        
        //отправка ответки на хост
        wifi.send((const uint8_t*)command.c_str(), strlen(command.c_str()));

        //сброс команды на исходную
        command = "";
    }
    if(len  == 404){
      ///получен код ошибки - превышен TIMEOUT. Поэтому предпринимаем попытку восстановления соединения
      Serial.println("Host TIMEOUT. Recovering TCP connection...");
          //убиваем соединение со стороны клиента
        if(wifi.releaseTCP()){
          //создаем заново
          if (wifi.createTCP(HOST_NAME, HOST_PORT)) {
              Serial.print("create tcp ok\r\n");
              digitalWrite(LED_BUILTIN, HIGH);
          }else {
              Serial.print("create tcp err\r\n");
              digitalWrite(LED_BUILTIN, LOW);
          }
        }else{
          Serial.println("Err 404 is in use.");
        }

        
      }
    
  delay(15);//70 30 20
}
