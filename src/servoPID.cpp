/*
 * servoPID.cpp
 *
 *  Created on: 2018¦~1¤ë18¤é
 *      Author: Jake
 */

#include "servoPID.h"

float servoPID::getPID(float error, uint32_t currentTime){
	dTime = currentTime - lastTime;
	currentError = error;
	float output = kP * currentError + kI * accumlateError * dTime + kD * ((currentError - lastError) / dTime);
	lastTime = currentTime;
	lastError = error;
	accumlateError += currentError;
	return output;
}
servoPID::~servoPID() {
	// TODO Auto-generated destructor stub
}

