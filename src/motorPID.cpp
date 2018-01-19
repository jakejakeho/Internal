/*
 * motorPID.cpp
 *
 *  Created on: 2018¦~1¤ë19¤é
 *      Author: Jake
 */

#include "motorPID.h"

float motorPID::getPID(int32_t desiredVelocity, int32_t encoderCount){
	dTime = System::Time() - lastTime;
	encoder->Update();
	currentError = desiredVelocity + encoderCount;
	float output = ((currentError) * kP) + (accumlateError) * kI * (dTime) + ((currentError - lastError) * kD) / (dTime);

	lastTime = System::Time();
	accumlateError += currentError;
	lastError = currentError;
	return output;
}

motorPID::~motorPID() {
	// TODO Auto-generated destructor stub
}

