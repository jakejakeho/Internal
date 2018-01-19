/*
 * motorPID.h
 *
 *  Created on: 2018¦~1¤ë19¤é
 *      Author: Jake
 */

#ifndef MOTORPID_H_
#define MOTORPID_H_
#include <libsc/system.h>
#include "libsc/dir_encoder.h"

using libsc::DirEncoder;
using libsc::System;
class motorPID {
public:
	motorPID(float KP, float KI, float KD, DirEncoder* Encoder):kP(KP),kI(KI),kD(KD),encoder(Encoder){};
	float getPID(int32_t desiredVelocity, int32_t encoderCount);
	virtual ~motorPID();
private:
	float kP = 0.0;
	float kI = 0.0;
	float kD = 0.0;
	uint32_t lastTime = 0, dTime = 0;
	int32_t currentVelocity = 0;
	int32_t accumlateError = 0;
	int32_t lastError = 0;
	int32_t currentError = 0;
	DirEncoder * encoder;
};

#endif /* MOTORPID_H_ */
