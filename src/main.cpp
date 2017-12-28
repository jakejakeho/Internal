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
#include <libbase/k60/gpio.h>
#include <libbase/k60/ftm_pwm.h>
#include <libbase/k60/pit.h>
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

Gpo *led1;
Gpo *led2;

void GPIListener(Gpi *gpi) {
	if (gpi->Get()) { // get state of GPI
		// if high
		led1->Turn();
	} else {
		// if low
		led1->Turn();
	}
}

void Job(Pit*){
/*code which will execute periodically*/
	led2->Turn();
}


int main(void)
{
	System::Init();

	//type your code here
	Gpo::Config ConfigLED0;
	ConfigLED0.pin = Pin::Name::kPta14;
	ConfigLED0.is_high = false;
	Gpo led0(ConfigLED0);

	Gpo::Config ConfigLED1;
	ConfigLED1.pin = Pin::Name::kPta15;
	ConfigLED1.is_high = false;
	led1 = new Gpo(ConfigLED1);

	Gpo::Config ConfigLED2;
	ConfigLED2.pin = Pin::Name::kPta16;
	ConfigLED2.is_high = false;
	led2 = new Gpo(ConfigLED2);

//	FtmPwm::Config ConfigPWM;
//	ConfigPWM.pin = Pin::Name::kPta8; // need Pin with Ftm functionality
//	ConfigPWM.period = 100000;
//	ConfigPWM.pos_width = 50000;
//	ConfigPWM.alignment = FtmPwm::Config::Alignment::kCenter;
//	FtmPwm pwm(ConfigPWM);

	Gpi::Config ConfigGPI;
	ConfigGPI.pin = Pin::Name::kPta9;
	ConfigGPI.interrupt = Pin::Config::Interrupt::kBoth;
	ConfigGPI.config.set(Pin::Config::kPassiveFilter);
	ConfigGPI.isr = GPIListener;
	Gpi gpi(ConfigGPI);

	Pit::Config pitConfig;
	pitConfig.channel = 0;
	pitConfig.count = 75000*50; //job executed once per 250ms
	pitConfig.isr = Job;
	Pit pit(pitConfig);

    while (true){
    	if(System::Time() % 50 == 0)
    		led0.Turn();
    	System::DelayMs(1);
    }
	return 0;
}
