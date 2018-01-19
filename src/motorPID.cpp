/*
 * motorPID.cpp
 *
 *  Created on: 2018¦~1¤ë19¤é
 *      Author: Jake
 */

#include "motorPID.h"

uint16_t motorPID::getPID(int32_t desiredVelocity , DirEncoder* encoder){
	dTime = System::Time() - lastTime;
	encoder->Update();
	currentError = desiredVelocity + encoder->GetCount();
	uint16_t output = currentError * kP + accumlateError * kI * dTime + ((currentError - lastError) * kD) / (dTime);

	lastTime = System::Time();
	accumlateError += currentError;
	lastError = currentError;
	return output;
}

motorPID::~motorPID() {
	// TODO Auto-generated destructor stub
}

