#include "Driver.h"

Driver::Driver(int loPhase1, int hoPhase1, int loPhase2, int hoPhase2, int loPhase3, int hoPhase3)
    : _loPhase1(loPhase1), _hoPhase1(hoPhase1),
      _loPhase2(loPhase2), _hoPhase2(hoPhase2),
      _loPhase3(loPhase3), _hoPhase3(hoPhase3) {}

void Driver::init() {
    // Initialize MCPWM for each phase


    // pinMode(LO1, OUTPUT);
    // pinMode(HO1, OUTPUT);

    // pinMode(LO2, OUTPUT);
    // pinMode(HO2, OUTPUT);

    // pinMode(LO3, OUTPUT);
    // pinMode(HO3, OUTPUT);


    this->pwm_config.frequency = 38400; // Set frequency to 20 kHz
    this->pwm_config.cmpr_a = 0; // Initialize duty cycle for high-side PWM
    this->pwm_config.cmpr_b = 0; // Initialize duty cycle for low-side PWM
    this->pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER;
    this->pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    

    // Phase 1
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, _hoPhase1); // High-side
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, _loPhase1); // Low-side
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, MCPWM_DUTY_MODE_1);
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 5, 5);

    // Phase 2
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, _hoPhase2); // High-side
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, _loPhase2); // Low-side
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_B, MCPWM_DUTY_MODE_1);
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 5, 5);

    // Phase 3
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, _hoPhase3); // High-side
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, _loPhase3); // Low-side
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_B, MCPWM_DUTY_MODE_1);
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 5, 5);


    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_1);
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_2);
    // MCPWM0.int_ena.timer0_tez_int_ena = 1;
}

// void Driver::setPWM(mcpwm_timer_t timer, int pin, float duty) {
//     int pwmValue = static_cast<int>(duty * 8191); // Assuming 12-bit resolution (0-8191)
//     mcpwm_set_duty(MCPWM_UNIT_0, timer, MCPWM_OPR_A, pwmValue); // Set duty for high-side
//     mcpwm_set_duty(MCPWM_UNIT_0, timer, MCPWM_OPR_B, pwmValue); // Set duty for low-side
// }

void Driver::setPWMDutyCycle(float dutyA, float dutyB, float dutyC) {

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, dutyA); // Set duty for high-side
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, dutyA); // Set duty for low-side

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, dutyB); // Set duty for high-side
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, dutyB); // Set duty for low-side

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, dutyC); // Set duty for high-side
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, dutyC); // Set duty for low-side



}
