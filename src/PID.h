/*
 * PID.h
 *
 *  Created on: 2018¦~1¤ë2¤é
 *      Author: Jake
 */

#ifndef SRC_PID_H_
#define SRC_PID_H_

class PID {
public:
	PID();
	~PID();
private:
	const float Kp = 0.0;
	const float Ki = 0.0;
	const float Kd = 0.0;
	float previous_error = 0.0;
	float integral = 0.0;
};

#endif /* SRC_PID_H_ */
