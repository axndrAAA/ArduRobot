
#ifndef BOT_MOVING_MANEGMENT_h
#define BOT_MOVING_MANEGMENT_h

//пины двигателей
#define dir3 4//левые5
#define pwm3 6
#define dir2 2//правые
#define pwm2 5

#define MAX_PWM_VAL 70
#define MAX_ANG_PID_VAL 180
#define MIN_V_PWM 35

#define Kp 0.25
#define coordEps 30
#define ANG_EPS 5
#define FORWARD_V 37

class BotMovingManegement{

    private:
        int V = 200;//скорость вращения двигателей
        byte Mode = 0;//указатель режима
        float (*get_ang_func)(void);//указатель на функцию получения текущего угла от компаса
        
        void mode1Execute(const String &command);

        void getCoordFromComma(const String &command,int *coords);
        float getVectAngle(float Xr,float Yr);
        
    public:
        BotMovingManegement();
        BotMovingManegement(float (*get_cur_ang)(void));
        

        void turnLeft();
        void turnRight();
        void goForward();
        void stop();
        void goBackward();

        void setV(int _v);
        float getHeadingHQ();
        void getMessage(String &command);

        void turnAngle(int new_angle);

        void executeModeCommand(const String &command);
        void mode2Execute(const String &command);

};
#endif