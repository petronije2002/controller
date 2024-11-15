#ifndef DRIVER_H
#define DRIVER_H

#include "driver/mcpwm.h"
#include "PWMconfig.h"


class Driver {
public:
    Driver(int loPhase1, int hoPhase1, int loPhase2, int hoPhase2, int loPhase3, int hoPhase3);
    void init();
    void setPWMDutyCycle(float dutyA, float dutyB, float dutyC);

private:
    void setPWM(mcpwm_timer_t timer, int pin, float duty); // Change here

    int _loPhase1;
    int _hoPhase1;
    int _loPhase2;
    int _hoPhase2;
    int _loPhase3;
    int _hoPhase3;
};

#endif // DRIVER_H
