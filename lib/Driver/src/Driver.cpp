#include "Driver.h"

Driver::Driver(int loPhase1= LO1, int hoPhase1=HO1, int loPhase2=LO2, int hoPhase2=HO2, int loPhase3=LO3, int hoPhase3=HO3)
    : _loPhase1(loPhase1), _hoPhase1(hoPhase1),
      _loPhase2(loPhase2), _hoPhase2(hoPhase2),
      _loPhase3(loPhase3), _hoPhase3(hoPhase3) {}

void Driver::init() {
    // Initialize MCPWM for each phase
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 20000; // Set frequency to 20 kHz
    pwm_config.cmpr_a = 0; // Initialize duty cycle for high-side PWM
    pwm_config.cmpr_b = 0; // Initialize duty cycle for low-side PWM
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

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
}

void Driver::setPWM(mcpwm_timer_t timer, int pin, float duty) {
    int pwmValue = static_cast<int>(duty * 8191); // Assuming 12-bit resolution (0-8191)
    mcpwm_set_duty(MCPWM_UNIT_0, timer, MCPWM_OPR_A, pwmValue); // Set duty for high-side
    mcpwm_set_duty(MCPWM_UNIT_0, timer, MCPWM_OPR_B, pwmValue); // Set duty for low-side
}

void Driver::setPWMDutyCycle(float dutyA, float dutyB, float dutyC) {
    setPWM(MCPWM_TIMER_0, _hoPhase1, dutyA);
    setPWM(MCPWM_TIMER_0, _loPhase1, dutyA);
    setPWM(MCPWM_TIMER_1, _hoPhase2, dutyB);
    setPWM(MCPWM_TIMER_1, _loPhase2, dutyB);
    setPWM(MCPWM_TIMER_2, _hoPhase3, dutyC);
    setPWM(MCPWM_TIMER_2, _loPhase3, dutyC);
}
