/*
 * MotorDriver.h
 *
 *  Created on: Mar 12, 2024
 *      Author: morda
 */

#ifndef INC_MOTOR_HPP_
#define INC_MOTOR_HPP_

#include <cstdint>

class Motor{
	uint32_t PwmPin_;
	uint32_t In1_;
	uint32_t In2_;
public:
	Motor(uint32_t PwmPin, uint32_t In1, uint32_t In2);
	void driveClocwise(uint16_t speed);
	void driveCounterClocwise(uint16_t speed);
	void fastStop();
};





#endif /* INC_MOTOR_HPP_ */
