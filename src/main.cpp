/*
 * main.cpp
 *
 * Author: 
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cassert>
#include <cstring>
#include <cmath>
#include <vector>
#include <libbase/misc_types.h>
#include <cstdint>
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


// constant variable
static const Byte imageHeight = 60;
static const Byte imageWidth = 80;
static const Byte middleHeight = imageHeight / 2;
static const Byte bottomHeight = imageHeight - 1;
static bool image[imageHeight][imageWidth];
static bool edgeImage[imageHeight][imageWidth];
static bool cornerImage[imageHeight][imageWidth];
static int numOfCorner;
// function prototype
void imageRead(const Byte* buff);
void edgeDetection();
void edgeDisplay(St7735r*);
float getAngle();
void cornerDetection();
void cornerDisplay(St7735r*);

// read the image to the memory
void imageRead(const Byte* buff){
	int numOfShift = 0;
	int numberOfByte = 0;
	for(int y = 0; y < imageHeight; y++){
		for(int x = 0; x < imageWidth; x++){
			image[y][x] = (buff[numberOfByte]) & (1 << (7 - numOfShift));
			numOfShift++;
			if(numOfShift % 8 == 0){
				numOfShift = 0;
				numberOfByte++;
			}
		}
	}
	// mean filter
	uint8_t offset = 3 / 2;
	for(int y = offset; y < imageHeight - offset; y++){
		for(int x = offset; x < imageWidth - offset; x++){
			int sum = 0;
			for(int j = y - offset; j <= y + offset; j++){
				for(int i = x - offset; i <= x + offset; i++){
					sum += image[j][i];
				}
			}
			if(sum >= 5){
				image[y][x] = 1;
			}else{
				image[y][x] = 0;
			}
		}
	}
}

void edgeDetection(void){
	for(int j = imageHeight - 2; j >= 1; j--){
		for(int i = 1; i < imageWidth - 1; i++){
			edgeImage[j][i] = (sqrt(pow(image[j - 1][i - 1] + 2 * image[j][i - 1] + image[j + 1][i - 1]        // +1 +2 +1
									 - image[j - 1][i + 1] - 2 * image[j][i + 1] - image[j + 1][i + 1], 2)    // -1 -2 -1
								 + pow(image[j - 1][i - 1] + 2 * image[j - 1][i] + image[j - 1][i + 1]   	  // +1 +2 +1
									 - image[j + 1][i - 1] - 2 * image[j + 1][i] - image[j + 1][i + 1], 2))) != 0 ? 1 : 0;  // -1 -2 -1
		}
	}
}

void edgeDisplay(St7735r* lcd){
	for(int j = imageHeight - 1; j >= 1; j--){
		for(int i = 1; i < imageWidth - 1; i++){
			if(edgeImage[j][i]){
				lcd->SetRegion(Lcd::Rect(i,j,1,1));
				lcd->FillColor(lcd->kRed);
			}
		}
	}
}

float getAngle(){
	int leftx = 0;
	int lefty = 0;
	int rightx = 0;
	int righty = 0;
	for(int j = imageHeight - 1; j >= 0; j--) {
		for(int i = 0; i < imageWidth / 2; i++) {
			if(edgeImage[j][i]) {
				leftx = i;
				lefty = j;
				j = -1;
				break;
			}
		}
	}
	for(int j = imageHeight - 1; j >= 0; j--) {
		for(int i = imageWidth - 1; i > imageWidth / 2; i--) {
			if(image[j][i]) {
				rightx = i;
				righty = j;
				j = -1;
				break;
			}
		}
	}
    double angle_left = atan((imageHeight - lefty) / sqrt(pow(leftx - imageWidth / 2, 2) + pow(lefty - imageHeight, 2)));
    double angle_right = atan((imageHeight - righty) / sqrt(pow(rightx - imageWidth / 2, 2) + pow(righty - imageHeight, 2)));
	return (angle_left - angle_right) * 57.2957795131;
}

void cornerDetection(void){
	int differenceTh = 25;
	int geometricalTh = 18;
	int rowRadius[] = { 1, 2, 3, 3, 3, 2, 1 };
	int size1 = 3;
	int susanMap[imageHeight][imageWidth] = {0};
	for (int x = size1; x < imageHeight - size1; x++) {
		 for (int y = size1; y < imageWidth - size1; y++) {
			 int nucleusValue = (int)image[x][y];
			 int usan = 0;
			 int cx = 0, cy = 0;
			 for (int i = -size1; i <= size1; i++) {
				 int r = rowRadius[i + size1];
				 for ( int j = -r; j <= r; j++ ){
					 int gray = image[x+i][y+j];
					 // differenceThreshold
					 if ( abs( nucleusValue - gray ) <= differenceTh ){
						 usan++;
						 cx += x + j;
						 cy += y + i;
					 }
				 }
			 }
				 // check usan size
			 if ( usan < geometricalTh ){
				 cx /= usan;
				 cy /= usan;
				 if ( ( x != cx ) || ( y != cy ) ){
					 usan = ( geometricalTh - usan );
				 }
				 else{
					 usan = 0;
				 }
			 }
			 else{
				 usan = 0;
			 }
				 // usan = ( usan < geometricalThreshold ) ? ( geometricalThreshold - usan ) : 0;
			 susanMap[x][y] = usan;
		 }
	 }
		 // for each row
	 int size2 = 3;
	 for ( int x = size2; x < imageHeight - size2; x++ ){
		 // for each pixel
		 for ( int y = size2; y < imageWidth - size2; y++ ){
			 int currentValue = susanMap[x][y];
				 // for each windows' row
			 for ( int i = -size2; ( currentValue != 0 ) && ( i <= 2 ); i++ ){
				 // for each windows' pixel
				 for ( int j = -size2; j <= size2; j++ ){
					 if ( susanMap[x+i][y+j] > currentValue ){
						 currentValue = 0;
						 break;
					 }
				 }
			 }
				 // check if this point is really interesting
			 // check if all the pixel around is all white or black
			 if ( currentValue != 0){
				cornerImage[y][x] = 1;
				numOfCorner++;
			 }else{
				 cornerImage[y][x] = 0;
			 }
		 }
	 }
}

void cornerDisplay(St7735r* lcd){
	for(int j = imageHeight - 1; j >= 1; j--){
		for(int i = 1; i < imageWidth - 1; i++){
			if(cornerImage[j][i]){
				lcd->SetRegion(Lcd::Rect(i,j,10,10));
				lcd->FillColor(lcd->kGreen);
			}
		}
	}
}

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
	AlternateMotor motor(Config::GetMotorConfig());
	motor.SetPower(90); // 10 %
	motor.SetClockwise(true);

	// Servo init
	Servo servo(Config::GetServoConfig());
	servo.SetDegree(0);
	System::DelayMs(500);

	// Camera init
	Ov7725 camera(Config::GetCameraConfig());
	camera.Start();

	// Battery Meter init
	//BatteryMeter bMeter(Config::GetBatteryMeterConfig());


	while (true){
		if(System::Time() != previousTime){
			previousTime = System::Time();
			if(previousTime % 16 == 0 && camera.IsAvailable()){
				// Image read from camera
				led0.Switch();
				const Byte* buff = camera.LockBuffer();
				imageRead(buff);
				camera.UnlockBuffer();
				lcd.SetRegion(Lcd::Rect(0,0,imageWidth,imageHeight));
				lcd.FillBits(0x0000, 0xFFFF, buff, imageWidth * imageHeight);
				edgeDetection();
				edgeDisplay(&lcd);
//				cornerDetection();
//				cornerDisplay(&lcd);
				char c[15];
				lcd.SetRegion(Lcd::Rect(0,80,100,15));
				lcd.SetRegion(Lcd::Rect(0,95,100,15));
				float temp = getAngle() * 1.48;
				sprintf(c,"angle: %f!", temp);
				writer.WriteBuffer(c,15);
				lcd.SetRegion(Lcd::Rect(0,110,100,15));
				sprintf(c,"numofC: %d", numOfCorner);
				writer.WriteBuffer(c,15);
				servo.SetDegree((uint16_t)((temp * 10) + 825));
				//servo.SetDegree((uint16_t)800);
			}
		}
	}
	return 0;
}
