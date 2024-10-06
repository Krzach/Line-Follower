/*
 * Motor.cc
 *
 *  Created on: Mar 12, 2024
 *      Author: morda
 */

#include "../Inc/Motor.hpp"
#include "main.h"

Motor::Motor(uint32_t PwmPin, uint32_t In1, uint32_t In2): PwmPin_(PwmPin), In1_(In1), In2_(In2) {}

void Motor::driveClocwise(uint16_t speed){}

void Motor::driveCounterClocwise(uint16_t speed){}

void Motor::fastStop(){}


