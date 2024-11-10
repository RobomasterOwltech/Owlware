#include "ControllerPWM.hpp"

ControllerPWM::ControllerPWM(TIM_HandleTypeDef* _timer, uint16_t _channel, uint16_t _prescaler) {
    timer = _timer;
    channel = _channel;

    /*
    _timer->Init.Prescaler = _prescaler - 1;
    prescaler = _timer->Init.Prescaler;
    clock_frequency = 1 / (_timer->Init.Period * _timer->Init.Prescaler);*/
}

ControllerPWM::~ControllerPWM() {}

void ControllerPWM::set_clock_frequency(uint32_t _clock_frequency) {
    prescaler = 1 / (timer->Init.Period * _clock_frequency);
    timer->Init.Prescaler = prescaler;
}
void ControllerPWM::set_frequency(uint16_t pwm_frequency) {
    timer->Init.AutoReloadPreload = clock_frequency / pwm_frequency;
    arr = timer->Init.AutoReloadPreload;
}
void ControllerPWM::set_duty_cycle(uint16_t _duty_cycle) { ccr = (int)(_duty_cycle * arr) / 100; }

void ControllerPWM::start_pwm(int _channel) {
    if (_channel == 1) {
        timer->Instance->CCR1 = ccr;
    } else if (_channel == 2) {
        timer->Instance->CCR2 = ccr;
    } else if (_channel == 3) {
        timer->Instance->CCR3 = ccr;
    } else if (_channel == 4) {
        timer->Instance->CCR4 = ccr;
    }
}

void ControllerPWM::set_arr(uint32_t _arr) {
    arr = _arr;
    timer->Init.AutoReloadPreload = _arr;
}
uint16_t ControllerPWM::get_frequency() { return clock_frequency / timer->Init.AutoReloadPreload; }
uint16_t ControllerPWM::get_duty_cycle() { return (arr / ccr) * 100; }
uint16_t ControllerPWM::get_prescaler() { return timer->Init.Prescaler; }
void ControllerPWM::calibrateSignal() {}
#include "ControllerPWM.hpp"

    ControllerPWM::ControllerPWM(TIM_HandleTypeDef* _timer, uint16_t _channel) {
    timer = _timer;
    channel = _channel;

    /*
    _timer->Init.Prescaler = _prescaler - 1;
    prescaler = _timer->Init.Prescaler;
    clock_frequency = 1 / (_timer->Init.Period * _timer->Init.Prescaler);*/
}

ControllerPWM::~ControllerPWM() {}

void ControllerPWM::set_clock_frequency(uint32_t _clock_frequency) {
    timer_frequency = 1 / timer->Init.Period;
    prescaler = timer_frequency / _clock_frequency;
    timer->Init.Prescaler = prescaler;
    // TO DO:
    // Buscar clock source frequency
}
void ControllerPWM::set_frequency(uint16_t pwm_frequency) {
    arr = clock_frequency / pwm_frequency;
    timer->Init.AutoReloadPreload = arr;
}
void ControllerPWM::set_duty_cycle(uint16_t _duty_cycle) { ccr = (int)(_duty_cycle * arr) / 100; }

void ControllerPWM::start_pwm(int _channel) {
    if (_channel == 1) {
        timer->Instance->CCR1 = ccr;
    } else if (_channel == 2) {
        timer->Instance->CCR2 = ccr;
    } else if (_channel == 3) {
        timer->Instance->CCR3 = ccr;
    } else if (_channel == 4) {
        timer->Instance->CCR4 = ccr;
    }
}

void ControllerPWM::set_arr(uint32_t _arr) {
    arr = _arr;
    timer->Init.AutoReloadPreload = _arr;
}
uint16_t ControllerPWM::get_frequency() { return clock_frequency / timer->Init.AutoReloadPreload; }
uint16_t ControllerPWM::get_duty_cycle() { return (arr / ccr) * 100; }
uint16_t ControllerPWM::get_prescaler() { return timer->Init.Prescaler; }
void ControllerPWM::calibrateSignal() {}
