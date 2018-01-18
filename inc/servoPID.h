/*
 * servoPID.h
 *
 *  Created on: 2018¦~1¤ë18¤é
 *      Author: Jake
 */

#ifndef SERVOPID_H_
#define SERVOPID_H_
#include <libsc/system.h>
using libsc::System;
class servoPID {
public:
	servoPID(float KP, float KI, float KD):kP(KP),kI(KI),kD(KD){};
	float getPID(float error, uint32_t currentTime);
	virtual ~servoPID();
private:
	float kP = 1.0;
	float kI = 0.0;
	float kD = 0.01;
	uint32_t lastTime = 0, dTime = 0;
	float currentAngle = 0.0;
	float accumlateError = 0.0;
	float lastError = 0.0;
	float currentError = 0.0;
};

#endif /* SERVOPID_H_ */
