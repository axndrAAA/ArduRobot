
#ifndef BOT_MOVING_MANEGMENT_h
#define BOT_MOVING_MANEGMENT_h

//пины двигателей
#define dir3 4//левые5
#define pwm3 6
#define dir2 2//правые
#define pwm2 5

#define MAX_PWM_VAL 255
#define MAX_PID_VAL 45
#define Kp 0.5

#include"HMC5883L_Simple.h"

class BotMovingManegement{

    private:
        //скорость вращения двигателей
        int V = 200;
        //компас
        HMC5883L_Simple &compas;
    public:
        BotMovingManegement();
        BotMovingManegement(const HMC5883L_Simple & _compas);
        

        void turnLeft();
        void turnRight();
        void goForward();
        void stop();
        void goBackward();
        void setV(int _v);

        void turnAngle(int new_angle);

                      
    
};
#endif