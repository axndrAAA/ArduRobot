#ifndef BOT_MOVING_MANEGMENT_h
#define BOT_MOVING_MANEGMENT_h

//пины двигателей
#define dir3 4//левые5
#define pwm3 6
#define dir2 2//правые
#define pwm2 5

class BotMovingManegement{

    private:
        //скорость вращения двигателей
        int V = 200;

    public:
        BotMovingManegement();

        void turnLeft();
        void turnLeftRight();
        void goForward();
        void stop();
        void goBackward();
};
#endif