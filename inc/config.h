/*
 * config.h
 *
 * configure all the peripherals here
 *
 *  Created on: Dec 23, 2017
 *      Author: dipsy
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include <functional>

#include "libsc/led.h"
#include "libsc/joystick.h"
#include "libsc/st7735r.h"
#include "libsc/lcd_typewriter.h"
#include "libsc/lcd_console.h"
#include "libsc/battery_meter.h"
#include "libsc/k60/jy_mcu_bt_106.h"
#include "libbase/k60/pit.h"
#include "libbase/k60/uart.h"
#include "libsc/k60/ov7725.h"
#include "libsc/alternate_motor.h"
#include "libsc/servo.h"
#include "libsc/battery_meter.h"
#include "libsc/button.h"
#include "libsc/dir_encoder.h"
#include "libbase/k60/adc.h"
#include "libbase/k60/pin.h"

using libsc::Led;
using libsc::Lcd;
using libsc::Joystick;
using libsc::St7735r;
using libsc::LcdTypewriter;
using libsc::LcdConsole;
using libsc::BatteryMeter;
using libsc::k60::JyMcuBt106;
using libbase::k60::Pit;
using libbase::k60::Uart;
using libsc::k60::Ov7725;
using libsc::k60::Ov7725Configurator;
using libsc::AlternateMotor;
using libsc::Servo;
using libsc::BatteryMeter;
using libsc::Button;
using libsc::DirEncoder;
using libbase::k60::Adc;
using libbase::k60::Pin;

class Config{
public:

	static Led::Config GetLedConfig(int id){
		Led::Config config;
		config.id = id;
		return config;
	}
	static LcdTypewriter::Config GetWriterConfig(St7735r *lcd){
		LcdTypewriter::Config config;
		config.lcd = lcd;
		return config;
	}
	static LcdConsole::Config GetConsoleConfig(St7735r *lcd){
		LcdConsole::Config config;
		config.lcd = lcd;
		config.region = Lcd::Rect(0,0,128,160);
		return config;
	}
    static Joystick::Config GetJoystickConfig() {
        //TODO: finish it
    	Joystick::Config config;
    	// ADD!
    	return config;
    }

    static St7735r::Config GetLcdConfig() {
        //TODO: finish it
    	St7735r::Config config;
    	config.fps = 20;
    	return config;
    }

    static JyMcuBt106::Config GetBluetoothConfig(std::function<bool(const Byte *data, const size_t size)> isr) {
        //TODO: finish it
    	JyMcuBt106::Config config;
    	config.id = 0;
    	config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
    	config.rx_isr = isr;
    	return config;
    }

    static Pit::Config GetBluetoothPitConfig(std::function<void(Pit*)> isr){
    	//TODO: finish it
    	Pit::Config pitConfig;
    	pitConfig.channel = 0;
    	pitConfig.count = 75000*10; //job executed once per 10ms
    	pitConfig.isr = isr;
    	return pitConfig;
    }

    static Ov7725::Config GetCameraConfig(){
    	Ov7725::Config config;
    	config.id = 0;
    	config.w = 80;
    	config.h = 60;
    	config.contrast = 74;
    	config.fps = Ov7725Configurator::Config::Fps::kHigh;
    	return config;
    }

    static AlternateMotor::Config GetMotorConfig(){
    	AlternateMotor::Config config;
    	config.id = 0;
    	return config;
    }

    static Servo::Config GetServoConfig(){
    	Servo::Config config;
    	config.id = 0;
    	config.period = 20000;
    	config.max_pos_width = 2100;
    	config.min_pos_width = 900;
    	return config;
    }

    static BatteryMeter::Config GetBatteryMeterConfig(){
    	BatteryMeter::Config config;
    	config.voltage_ratio = 0.4;
    	return config;
    }

    static Button::Config GetButtonConfig(Button::Listener listener){
    	Button::Config config;
    	config.id = 0;
    	config.is_active_low = true;
    	config.listener = listener;
    	config.listener_trigger = Button::Config::Trigger::kUp;

    }

    static DirEncoder::Config GetEncoderConfig(){
    	DirEncoder::Config config;
    	config.id = 0;
    	return config;
    }

    static Adc::Config GetAdc1Config(){
    	Adc::Config config;
    	config.pin = Pin::Name::kPtc10;
    	config.speed = Adc::Config::SpeedMode::kExSlow;
    	config.is_continuous_mode = true;
    	config.avg_pass = Adc::Config::AveragePass::k32;
    	return config;
    }

    static Adc::Config GetAdc2Config(){
    	Adc::Config config;
    	config.pin = Pin::Name::kPtc11;
    	config.speed = Adc::Config::SpeedMode::kExSlow;
    	config.is_continuous_mode = true;
    	config.avg_pass = Adc::Config::AveragePass::k32;
    	return config;
    }
};


#endif /* INC_CONFIG_H_ */
