#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#include "botMovingManegment.h"
#include "HMC5883L_Simple.h"
#include "ESP8266.h"


// Create a compass
HMC5883L_Simple Compass;
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
#define TIMEOUT 10000

//буфер принимаемых команд
uint8_t buffer[128] = {0};

//буфер формирования команды
String command = "";

void setup()
{
  //I2C для компаса
  Wire.begin();
  //настройка компаса
  Compass.SetDeclination(23, 35, 'E');
  Compass.SetSamplingMode(COMPASS_SINGLE);
  Compass.SetScale(COMPASS_SCALE_130);
  Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);
  Compass.setUpZeroHeading();

  // //настройка вафли
  // //для отладки
  Serial.begin(9600);
  Serial.print("setup begin\r\n");
  // Serial.print("FW Version: ");
  // Serial.println(wifi.getVersion().c_str());

  // //переключение вайли в режим станции
  // if (wifi.setOprToStationSoftAP()) {
  //   Serial.print("to station + softap ok\r\n");
  // } else {
  //   Serial.print("to station + softap err\r\n");
  // }

  // //подключение к точке доступа
  // if (wifi.joinAP(SSID, PASSWORD)) {
  //   Serial.print("Join AP success\r\n");
  //   Serial.print("IP: ");
  //   Serial.println(wifi.getLocalIP().c_str());
  // } else {
  //     Serial.print("Join AP failure\r\n");
  // }

  // //включаем одиночный режим (выключаем MUX-режим)
  // if (wifi.disableMUX()) {
  //   Serial.print("single ok\r\n");
  // } else {
  //   Serial.print("single err\r\n");
  // }

  // //создание TCP соединения
  // if (wifi.createTCP(HOST_NAME, HOST_PORT)) {
  //   Serial.print("create tcp ok\r\n");
  // } else {
  //   Serial.print("create tcp err\r\n");
  // }

  Serial.println("Ready.");
  digitalWrite(LED_BUILTIN, HIGH);
}

// Our main program loop.
void loop()
{

  
  int coord[4];
  bmm.executeModeCommand("b2/12/-256/-39/4001/e");

  delay(1500);

  // //восстанавливаем соединение, если оно упало
  // if (wifi.createTCP(HOST_NAME, HOST_PORT)) {
  //   Serial.print("create tcp ok\r\n");
  //   digitalWrite(LED_BUILTIN, HIGH);
  // } else {
  //   Serial.print("create tcp err\r\n");
  //   digitalWrite(LED_BUILTIN, LOW);
  // }

  //читаем команду от хоста
  // uint32_t len = wifi.recv(buffer, sizeof(buffer), TIMEOUT);

  //   //выброс в Serial для отладки
  //   if (len > 0) {
  //       Serial.print("Received:[");
  //       for(uint32_t i = 0; i < len; i++) {
  //           Serial.print((char)buffer[i]);
  //           command+=(char)buffer[i];
  //       }
  //       Serial.print("] ");
  //       Serial.print(sizeof(buffer));
  //       Serial.print(" ");
  //       Serial.print(len);
  //       Serial.print("\r\n");
  //       bmm.executeModeCommand(command);

  //       //получение строки-состояния для отправки на хост
  //       bmm.getMessage(command);
  //       Serial.println(command);

  //       //отправка ответки на хост
  //       wifi.send((const uint8_t*)command.c_str(), strlen(command.c_str()));

  //       //сброс команды на исходную
  //       command = "";
  //   }




  //   float heading = bmm.getHeadingHQ();
  //  Serial.print("Heading: \t");
  //  Serial.print( heading );
  //  bmm.turnAngle(250);
   //delay(1000);
}
