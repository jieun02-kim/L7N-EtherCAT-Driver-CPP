#ifndef __MOTOR_H_
#define __MOTOR_H_
#include "driver.h"

class Motor
{
    private:
        int encoder;
        int gear_ratio;
        float joint;
        float position;

    public:
        bool InitMember(int enc, int gear, float jnt, float pos);
        int GetCnt() const;

};


#endif