/*
 * main.cpp
 *
 * Author: 
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cassert>
#include <cstring>
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include <libsc/led.h>
#include "config.h"

namespace libbase
{
	namespace k60
	{

		Mcg::Config Mcg::GetMcgConfig()
		{
			Mcg::Config config;
			config.external_oscillator_khz = 50000;
			config.core_clock_khz = 100000;
			return config;
		}

	}
}

using libsc::System;
using namespace libsc;
using namespace libbase::k60;

const uint8_t imageHeight = 60;
const uint8_t imageWidth = 80;
bool image[imageHeight][imageWidth];

bool getImage(const Byte* buff, int x, int y);
int main(void)
{
	System::Init();
	uint32_t previousTime = 0;


	// LED Test
    Led led0(Config::GetLedConfig(0));
    Led led1(Config::GetLedConfig(1));
    Led led2(Config::GetLedConfig(2));
    Led led3(Config::GetLedConfig(3));

    led0.SetEnable(0);
    led1.SetEnable(0);
    led2.SetEnable(0);
    led3.SetEnable(0);

	System::DelayMs(200);

	led0.Switch();
	led1.Switch();
	led2.Switch();
	led3.Switch();

	System::DelayMs(10);

	// LCD Test
	St7735r lcd(Config::GetLcdConfig());
	LcdTypewriter writer(Config::GetWriterConfig(&lcd));
	LcdConsole console(Config::GetConsoleConfig(&lcd));
	lcd.SetRegion(Lcd::Rect(0,0,128,160));
	for(int i = 0; i < 10; i++){
		lcd.SetRegion(Lcd::Rect(0,(i*15),100,15));
		writer.WriteString("LCDTESTING!!!");
	}
	System::DelayMs(200);
	lcd.Clear();


	// Motor init
	DirMotor motor(Config::GetMotorConfig());
	motor.SetClockwise(true);
	motor.SetPower(100);

	// Servo init
	Servo servo(Config::GetServoConfig());
	int servoValue = 900;
	bool toRight = true;

	// Camera init
	Ov7725 camera(Config::GetCameraConfig());
	camera.Start();


	while (true){
		if(System::Time() != previousTime){
			previousTime = System::Time();
			if(previousTime % 16 == 0 && camera.IsAvailable()){
				// Image read from camera
				led0.Switch();
				const Byte* buff = camera.LockBuffer();
				lcd.SetRegion(Lcd::Rect(0,0,imageWidth,imageHeight));
				lcd.FillBits(0x0000, 0xFFFF, buff, imageWidth * imageHeight);
				camera.UnlockBuffer();
			}
			if(previousTime % 10 == 0){
				if(servoValue == 1800){
					toRight = false;
				}else if(servoValue == 0){
					toRight = true;
				}
				if(toRight){
					servoValue += 10;
				}
				else{
					servoValue -= 10;
				}
				servo.SetDegree(servoValue);
			}
		}
	}
	return 0;
}

bool getImage(const Byte* buff, int x, int y){
	return buff[(10 * y) + (x >> 3)] & (1 << ((8 - (x % 8)) - 1));
}
