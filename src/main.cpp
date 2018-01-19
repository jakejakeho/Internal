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
#include "servoPID.h"
#include "motorPID.h"

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
static const uint16_t middleServo = 790;

static bool image[imageHeight][imageWidth];
static bool edgeImage[imageHeight][imageWidth];
static bool cornerImage[imageHeight][imageWidth];
static Byte midPoint[imageHeight];
static int numOfCorner = 0;
static bool lastHasRCorner = false;
static int rCounter = 0;
static int rCorner = 0;
static int circleCounter = 0;
static uint32_t lastRCornerTime = 0;
static bool clockwise = true;
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
float fsqrt(float x){
	unsigned int i = *(unsigned int*) &x;
	// adjust bias
	i  += 127 << 23;
	// approximation of square root
	i >>= 1;
	return *(float*) &i;
}

void edgeDetection(void){
	for(int j = imageHeight - 2; j >= 1; j--){
		for(int i = 1; i < imageWidth - 1; i++){
			edgeImage[j][i] = (fsqrt(pow(image[j - 1][i - 1] + 2 * image[j][i - 1] + image[j + 1][i - 1]        // +1 +2 +1
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
	uint8_t kernel[5][5] = {{0,-1,2,1,0},
							{-1,-2,4,2,1},
							{-2,-4,8,4,2},
							{1,2,-4,-2,-1},
							{0,1,-2,-1,0}};
	uint8_t offset = 5/2;
	uint8_t min = 18;
	uint8_t max = 24;
	numOfCorner = 0;
	for(int j = imageHeight - offset; j >= offset; j--){
		for(int i = offset; i <imageWidth - offset; i++){
			if(edgeImage[j][i]){
				int temp = 0;
				for(int y = 0 - offset; y <= offset; y++){
					for(int x = 0 - offset; x <= offset; x++){
						temp += edgeImage[j + y][i + x] * kernel[offset + y][offset + x];
					}
				}
				if(temp >= min && temp <= max){
					edgeImage[j][i] = 1;
					numOfCorner++;
				}else{
					edgeImage[j][i] = 0;
				}

			}
		}
	}

}

void cornerDetection2(){
	int differenceTh = 25;
		int geometricalTh = 18;
		int rowRadius[] = { 1, 2, 3, 3, 3, 2, 1 };
		uint8_t susanMap[imageHeight][imageWidth] = {0};
		 for (int x = 3; x < imageHeight - 3; x++) {
			 for (int y = 3; y < imageWidth - 3; y++) {
				 int nucleusValue = (int)image[x][y];
				 int usan = 0;
				 int cx = 0, cy = 0;
				 for (int i = -3; i <= 3; i++) {
					 int r = rowRadius[i + 3];
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
		 for ( int x = 2; x < imageHeight - 2; x++ ){
			 // for each pixel
			 for ( int y = 2; y < imageWidth - 2; y++ ){
				 int currentValue = susanMap[x][y];

				 // for each windows' row
				 for ( int i = -2; ( currentValue != 0 ) && ( i <= 2 ); i++ ){
					 // for each windows' pixel
					 for ( int j = -2; j <= 2; j++ ){
						 if ( susanMap[x+i][y+j] > currentValue ){
							 currentValue = 0;
							 break;
						 }
					 }
				 }

				 // check if this point is really interesting
				 // check if all the pixel around is all white or black
				 if ( currentValue != 0){
						cornerImage[x][y] = 1;
						numOfCorner++;
				 }else{
					 cornerImage[x][y] = 0;
				 }
			 }
		 }
}

void cornerDisplay(St7735r* lcd){
	for(int j = imageHeight - 1; j >= 1; j--){
		for(int i = 1; i < imageWidth - 1; i++){
			if(cornerImage[j][i]){
				lcd->SetRegion(Lcd::Rect(i,j,15,15));
				lcd->FillColor(lcd->kGreen);
			}
		}
	}
}

void findMidPoint(){
	for(int j = 0; j < imageHeight; j++){
		uint8_t left = 0;
		uint8_t right = imageWidth - 1;

		// find x value of left edge
		for(int i = imageWidth / 2; i >= 0; i--){
			if(edgeImage[j][i]){
				left = i;
				break;
			}
		}

		// find x value of right edge
		for(int i = imageWidth / 2; i < imageWidth; i++){
			if(edgeImage[j][i]){
				right = i;
				break;
			}
		}
		if(left != 0 && right != imageWidth - 1) {
			midPoint[j] = (left + right) / 2;
		}else{
			midPoint[j] = 81;
		}
	}
}

void midPointDisplay(St7735r *lcd){
	for(int j = imageHeight - 1; j >= 1; j--){
		if(midPoint[j] != 81){
			lcd->SetRegion(Lcd::Rect(midPoint[j], j, 1, 1));
			lcd->FillColor(lcd->kGreen);
		}
	}
}

bool hasLeftEdge(uint8_t height){
	for(int i = imageWidth / 2; i >= 0; i++){
		if(edgeImage[height][i]){
			return true;
		}
	}
	return false;
}

uint8_t numOfLeftEdge(){
	bool lastHasEdge = false;
	uint8_t numOfMidLine = 0;
	uint8_t numOfPixel = 0;
	uint8_t threshold = 8;
	for(int j = imageHeight -1; j >= 1; j--){
		if(hasLeftEdge(j) && !lastHasEdge){
			lastHasEdge = true;
			numOfPixel++;
		}else if(!hasLeftEdge(j) && lastHasEdge){
			lastHasEdge = false;
			numOfPixel = 0;
		}else if(hasLeftEdge(j) && lastHasEdge){
			numOfPixel++;
			if(numOfPixel == threshold){
				numOfMidLine++;
				break;
			}
		}
	}
	return numOfMidLine;
}

bool hasRightEdge(uint8_t height){
	for(int i = imageWidth / 2; i < imageWidth; i++){
		if(edgeImage[height][i]){
			return true;
		}
	}
	return false;
}

uint8_t numOfRightEdge(){
	bool lastHasEdge = false;
	uint8_t numOfMidLine = 0;
	uint8_t numOfPixel = 0;
	uint8_t threshold = 12;
	for(int j = imageHeight / 2; j < imageHeight; j++){
		if(hasRightEdge(j) && !lastHasEdge){
			lastHasEdge = true;
			numOfPixel++;
		}else if(!hasRightEdge(j) && lastHasEdge){
			lastHasEdge = false;
			numOfPixel = 0;
		}else if(hasRightEdge(j) && lastHasEdge){
			numOfPixel++;
			if(numOfPixel == threshold){
				numOfMidLine++;
				break;
			}
		}
	}
	return numOfMidLine;
}

uint8_t numOfMidLine(){
	bool lastHasEdge = false;
	uint8_t numOfMidLine = 0;
	uint8_t numOfPixel = 0;
	uint8_t threshold = 12;
	for(int j = imageHeight -1; j >= 1; j--){
		if(midPoint[j] != 81 && !lastHasEdge){
			lastHasEdge = true;
			numOfPixel++;
		}else if(midPoint[j] == 81 && lastHasEdge){
			lastHasEdge = false;
			numOfPixel = 0;
		}else if(midPoint[j] != 81 && lastHasEdge){
			numOfPixel++;
			if(numOfPixel == threshold){
				numOfMidLine++;
				break;
			}
		}
	}
	return numOfMidLine;
}

int numOfRCorner(){
	if(numOfLeftEdge() == 1 && numOfRightEdge() == 0 && numOfMidLine() == 0){
		if(!lastHasRCorner)
			lastHasRCorner = true;
		rCounter++;
		if(rCounter == 6 && rCorner == 0){
			lastRCornerTime = System::Time();
			rCorner++;
		}
		else if(rCounter == 6 && rCorner == 1){
			if(System::Time() - lastRCornerTime <= 5000){
				lastRCornerTime = System::Time();
				rCorner++;
			}
		}
		else if(rCounter == 2 && rCorner == 2) {
			if(System::Time() - lastRCornerTime >= 5500){
				lastRCornerTime = System::Time();
				rCorner++;
			}
		}
		else if(rCounter == 3 && rCorner >= 3) {
			if(System::Time() - lastRCornerTime >= 500){
				lastRCornerTime = System::Time();
				rCorner++;
			}
		}
	}else{
		lastHasRCorner = false;
		rCounter = 0;
		if(System::Time() - lastRCornerTime > 3000 && rCorner <= 2){
			rCorner = 0;
		}
	}
	return rCorner;
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

	// LCD Test
	St7735r lcd(Config::GetLcdConfig());
	LcdTypewriter writer(Config::GetWriterConfig(&lcd));
	LcdConsole console(Config::GetConsoleConfig(&lcd));
	lcd.SetRegion(Lcd::Rect(0,0,128,160));
	lcd.Clear();

	// Encoder init
	DirEncoder dirEncoder(Config::GetEncoderConfig());
	// Motor init
	AlternateMotor motor(Config::GetMotorConfig());
	motor.SetClockwise(true);
//	motorPID mPID(0.04, 0.0000, 0.000005);
	motorPID mPID(0.024, 0.0, 12, &dirEncoder);

	// Servo init
	Servo servo(Config::GetServoConfig());
	servoPID sPID(1.40,0.0,0.00);

	// Battery Meter init
	BatteryMeter bMeter(Config::GetBatteryMeterConfig());



	// Button init
//	Button button(Config::GetButtonConfig(Button::Listener([](const uint8_t id){
//		if(id == 0)
//			clockwise = !clockwise;
//	})));



	// Camera init
	Ov7725 camera(Config::GetCameraConfig());
	camera.Start();

	int lastrCorner = 0;
	motor.SetPower(110); // 10 %
	while (true){
		if(System::Time() != previousTime){
			previousTime = System::Time();
			if(previousTime % 16 == 0 && camera.IsAvailable()){
				// Image read from camera
				led0.Switch();
				const Byte* buff = camera.LockBuffer();
				imageRead(buff);
				camera.UnlockBuffer();
				bool enableDisplay = false;
				char c[15];
				edgeDetection();
				findMidPoint();
				if(enableDisplay){
					lcd.SetRegion(Lcd::Rect(0,0,imageWidth,imageHeight));
					lcd.FillBits(0x0000, 0xFFFF, buff, imageWidth * imageHeight);
					edgeDisplay(&lcd);
					midPointDisplay(&lcd);
				}


				float angle = sPID.getPID(getAngle(), previousTime);
				lcd.SetRegion(Lcd::Rect(0,60,128,15));
				sprintf(c,"voltage: %f", bMeter.GetVoltage());
				writer.WriteBuffer(c,15);
				lcd.SetRegion(Lcd::Rect(0,75,128,15));
				sprintf(c,"angle: %.2f!", angle);
				writer.WriteBuffer(c,15);
				lcd.SetRegion(Lcd::Rect(0,90,128,15));
//				sprintf(c,"L:%d R:%d M:%d", numOfLeftEdge(), numOfRightEdge(), numOfMidLine());
				writer.WriteBuffer(c,15);
				lcd.SetRegion(Lcd::Rect(0,105,128,15));

				int encoderCount = dirEncoder.GetCount();
				float power = motor.GetPower() + mPID.getPID(350, encoderCount);
				sprintf(c,"Motor: %.2f", power);

				writer.WriteBuffer(c,15);
				lcd.SetRegion(Lcd::Rect(0,120,128,15));
				sprintf(c,"Encoder: %d", dirEncoder.GetCount());
				writer.WriteBuffer(c,15);
				lcd.SetRegion(Lcd::Rect(0,135,128,15));
				if(clockwise){
					sprintf(c,"Corner: %d",numOfRCorner());
					numOfCorner = rCorner;
				}//else {
//					sprintf(c,"Lcorner: %d",numOfRCorner());
//					numOfCorner = lCorner;
//				}
				writer.WriteBuffer(c,15);

				servo.SetDegree((uint16_t)((angle * 10) + middleServo));
				if(numOfCorner == 2){
					if(circleCounter >= 3 && circleCounter <= 10)
						servo.SetDegree((uint16_t) middleServo - 550);
					circleCounter++;
				}else if (numOfCorner == 3){
					if(circleCounter <= 8)
						servo.SetDegree((uint16_t) middleServo - 620);
					circleCounter++;
				}else if (numOfCorner == 4){
					if(circleCounter <= 12)
						servo.SetDegree((uint16_t) middleServo + 375);
					circleCounter++;
				}
				if(lastrCorner != numOfCorner){
					lastrCorner = numOfCorner;
					circleCounter = 0;
				}
//				servo.SetDegree((uint16_t)middleServo);
				if(power > 500)
					power = 500;
				if(power < 0){
					power = 0;
				}
				motor.SetPower((uint16_t)power);
			}
		}
	}
	return 0;
}
