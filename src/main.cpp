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


int main(void)
{
	System::Init();

	Led::Config led_config;
	led_config.id = 0;
	Led led0(led_config);

	Led::Config led_config1;
	led_config1.id = 1;
	Led led1(led_config1);

	Led::Config led_config2;
	led_config2.id = 2;
	Led led2(led_config2);

	Led::Config led_config3;
	led_config3.id = 3;
	Led led3(led_config3);

	while (true){
		if(System::Time() % 250 == 0){
			led0.Switch();
			System::DelayMs(1);
			led1.Switch();
			System::DelayMs(1);
			led2.Switch();
			System::DelayMs(1);
			led3.Switch();
			System::DelayMs(1);
		}
	}
	return 0;
}
