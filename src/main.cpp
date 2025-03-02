/* Reflow Toaster Oven
 * http://frank.circleofcurrent.com/reflowtoasteroven/
 * Copyright (c) 2011 Frank Zhao
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 *
 * This file handles a lot of initialization, low level timer tasks, and automatic temperature control
 *
 */

#define F_CPU 8000000UL // 8 MHz internal oscilator

#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <util/delay.h>

#include <Arduino.h>
#include <TimerOne.h> // Paul Stofregen's TimerOne Library, to use for the ovf interrupt. This breaks Tone() and arduino builtin PWM output which are not used here anyway.
#include "lcd.h"
#include "U8glib.h"
#include "userinput.h"
#include "nvm.h" // settings_load etc.

#include "reflowtoasteroven.h"
#include "temperaturemeasurement.h"
#include "heatingelement.h"
#include "menu.h"

#if 0 // set fuses to this, or #if 1 to include them in the .elf. Note: these are for Atmega328 -- TODO: use fuse names instead of magic numbers
FUSES = 
{
    0xD2, // .low -- CKDIV8 disabled, so 8MHz instead of 1 MHz. Startup time disabled.
    HFUSE_DEFAULT, // .high
    0xFC, // .extended -- bodlevel 4.5V (BOD is recommended to be enablen when disabling startup time, startup time needs to be disabled for WDT else it times out before startup is complete)
};
#endif

settings_t settings;					 // store this globally so it's easy to access

uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
void get_mcusr(void) \
  __attribute__((naked)) \
  __attribute__((section(".init3")));
void get_mcusr(void)
{
  mcusr_mirror = MCUSR;
  MCUSR = 0;
  wdt_disable(); 
  PORTD|=_BV(PD7); // set debugled ON (does this even get called?) -- no, not unless I let main call it, but that makes no sense as this needs to be called directly at reset / restart. TODO: Why is this never run?
}

U8GLIB_ST7920_128X64_1X u8g(A3, A5, A4); // SPI Com: SCK = en = LCD4 = PC3 = A3, MOSI = rw = SID = LCDE = PC5 = A5, CS = di = RS =LCDRS =PC4 = A4
FILE log_stream;						 // different in cpp from c = FDEV_SETUP_STREAM(log_putchar_stream, NULL, _FDEV_SETUP_WRITE);
static int log_putchar_stream(char c, FILE *stream);

int main()
{	
	wdt_reset();
	//try it inline, then
	//mcusr_mirror = MCUSR;
	//MCUSR = 0;
	//wdt_disable(); 
	// /try it inline, then (Hum. Leaving it out makes no difference either...)

	// inline, it does not cause a hang, so the hang is caused by calling a function with attribute naked, as these have no return adres.
	// still, after a WDT time-out triggers, the MCU just 'hangs' with whatever message was on the LCD at the time on the LCD. The 'error: wdt reset' message is never shown...
	// if I make debugled blink on startup, it does not blink after a (deliberate) WDT time-out

	DDRD|=(1<<PORTD7); // set PD7 output for debug LED
	PORTD|=_BV(PD7); // set debugled ON
	PORTD&=~_BV(PD7); // set debugled OFF
	wdt_enable(WDTO_2S);
	wdt_reset();

	fdev_setup_stream(&log_stream, log_putchar_stream, NULL, _FDEV_SETUP_WRITE);

	// initialize stuff here
	wdt_reset();
	init(); // init function from arduino. Sets up ADC and timers etc. for their default arduino-usage
	// that means the reflow oven can't use a timer interrupt for PWM, like Frank Zhao originaly did.
	Serial.begin(9600); // output stream / log / debug
	wdt_reset();
	fprintf_P(&log_stream, PSTR("hello world,\n"));
	button_init();
	wdt_reset();
	u8g.begin();
	u8g.setFont(u8g_font_unifont);
	adc_init(); // sets up adc for reflow oven thermocouple
	wdt_reset();
	heat_init();
	Timer1.initialize(TMR_OVF_TIMESPAN * 1000000); // microseconds of timer period... So for 490 Hz, about 2048 (TIMER_OVF_TIMESPAN is the same thing, but in seconds)
	Timer1.attachInterrupt(heat_isr);
	Timer1.start();

	fprintf_P(&log_stream, PSTR("reflow toaster oven,\n"));

	wdt_reset();

	// initialization has finished here

	if(mcusr_mirror & _BV(WDRF)){ // if the reset was caused by the WDT display a message
	//if(MCUSR & _BV(WDRF)){ // if the reset was caused by the WDT display a message
		u8g.firstPage();
		do
		{
			wdt_reset();
			u8g.drawStr(0, 14, "TEST");
			u8g.drawStr(0, 28, "WDT timeout !");
			u8g.drawStr(0, 44, "Have you tried turning");
			u8g.drawStr(0, 60, "it off and on again?");
		} while (u8g.nextPage());
		while(1){
			delay(400);
			wdt_reset();
		}; // if something hapens, it will be obvious
		}
	wdt_reset();
	main_menu(); // enter the menu system

	return 0;
}

// this estimates the PWM duty cycle needed to reach a certain steady temperature
// if the toaster is capable of a maximum of 300 degrees, then 100% duty cycle is used if the target temperature is 300 degrees, and 0% duty cycle is used if the target temperature is room temperature.
double approx_pwm(double target)
{
	return 65535.0 * ((target * THERMOCOUPLE_CONSTANT) / settings.max_temp);
}

uint16_t pid(double target, double current, double *integral, double *last_error)
{
	double error = target - current;
	if (target == 0)
	{
		// turn off if target temperature is 0

		(*integral) = 0;
		(*last_error) = error;
		return 0;
	}
	else
	{
		if (target < 0)
		{
			target = 0;
		}

		// calculate PID terms

		double p_term = settings.pid_p * error;
		double new_integral = (*integral) + error;
		double d_term = ((*last_error) - error) * settings.pid_d;
		(*last_error) = error;
		double i_term = new_integral * settings.pid_i;

		double result = approx_pwm(target) + p_term + i_term + d_term;

		// limit the integral so it doesn't get out of control
		if ((result >= 65535.0 && new_integral < (*integral)) || (result < 0.0 && new_integral > (*integral)) || (result <= 65535.0 && result >= 0))
		{
			(*integral) = new_integral;
		}

		// limit the range and return the rounded result for use as the PWM OCR value
		return (uint16_t)lround(result > 65535.0 ? 65535.0 : (result < 0.0 ? 0.0 : result));
	}
}

volatile uint16_t tmr_ovf_cnt = 0;
volatile char tmr_checktemp_flag = 0;
volatile char tmr_drawlcd_flag = 0;
volatile char tmr_writelog_flag = 0;

// store the temperature history for graphic purposes
uint8_t temp_history[LCD_WIDTH];
uint16_t temp_history_idx;
uint8_t temp_plan[LCD_WIDTH]; // also store the target temperature for comparison purposes

// this function runs an entire reflow soldering profile
// it works like a state machine
void auto_go(profile_t *profile)
{
	uint32_t prevmilis;
	uint8_t tick = 0;

	sensor_filter_reset();

	settings_load(&settings); // load from eeprom

	// validate the profile before continuing
	if (!profile_valid(profile))
	{
		u8g.firstPage();
		do
		{
			u8g.drawStr(0, 28, "Error");
			u8g.drawStr(0, 44, "in profile !");
		} while (u8g.nextPage());
		wdt_reset();
		delay(400);
		wdt_reset();
		delay(400);
		wdt_reset();
		delay(200);
		wdt_reset();
		return;
	}

	fprintf_P(&log_stream, PSTR("auto mode session start,\n"));

	// this will be used for many things later
	double max_heat_rate = settings.max_temp / settings.time_to_max;

	// reset the graph
	for (int i = 0; i < LCD_WIDTH; i++)
	{
		temp_history[i] = 0;
		temp_plan[i] = 0;
	}
	temp_history_idx = 0;

	// total duration is calculated so we know how big the graph needs to span
	// note, this calculation is only an worst case estimate
	// it is also aware of whether or not the heating rate can be achieved
	double total_duration = (double)(profile->soak_length + profile->time_to_peak) +
							((profile->soak_temp1 - ROOM_TEMP) / min(profile->start_rate, max_heat_rate)) +
							((profile->peak_temp - ROOM_TEMP) / profile->cool_rate) +
							10.0; // some extra just in case
	double graph_tick = total_duration / LCD_WIDTH;
	double graph_timer = 0.0;

	// some more variable initialization
	double integral = 0.0, last_error = 0.0;
	char stage = 0;			 // the state machine state
	uint32_t total_cnt = 0;	 // counter for the entire process
	uint16_t length_cnt = 0; // counter for a particular stage
	uint16_t pwm_ocr = 0;	 // temporary holder for PWM duty cycle
	double tgt_temp = sensor_to_temperature(sensor_read());
	double start_temp = tgt_temp;
	uint16_t cur_sensor = sensor_read();
	while (1)
	{
		if (millis() - prevmilis > 500)
		{ // this is a bit of a kludge to replace the flags set in timer ISR. And since worst case all 3 are done anyway, why not do all 3 always? As long as it takes less then 500 ms...
			prevmilis = millis();
			tick++;
			tmr_checktemp_flag = 1; // 0.5s
			#if 0
			if (0 != (tick & 0x01))
			{
				tmr_writelog_flag = 1; // 1s
			}
			if (0 != (tick & 0x02))
			{
				tmr_drawlcd_flag = 1; // 2s
			}
			#else // test what happens if all is done always -- still has time left over, good!
			tmr_writelog_flag = 1; 
			tmr_drawlcd_flag = 1;
			#endif
			PORTD|=(1<<7); // debugled ON
		}
		else
		{
			PORTD&=~(1<<7); // debugled OFF. 
		}


		if (tmr_checktemp_flag)
		{
			tmr_checktemp_flag = 0;

			total_cnt++;

			cur_sensor = sensor_read();

			if (DEMO_MODE)
			{
				// in demo mode, we fake the reading
				cur_sensor = temperature_to_sensor(tgt_temp);
			}

			if (stage == 0) // preheat to thermal soak temperature
			{
				length_cnt++;
				if (sensor_to_temperature(cur_sensor) >= profile->soak_temp1)
				{
					// reached soak temperature
					stage++;
					integral = 0.0;
					last_error = 0.0;
					length_cnt = 0;
				}
				else
				{
					// calculate next temperature by increasing current temperature
					tgt_temp = max(ROOM_TEMP, start_temp) + (profile->start_rate * TMR_OVF_TIMESPAN * 256 * length_cnt);

					if (length_cnt % 8 == 0)
					{
						start_temp = sensor_to_temperature(cur_sensor);
						length_cnt = 0;
					}

					tgt_temp = min(tgt_temp, profile->soak_temp1);

					// calculate the maximum allowable PWM duty cycle because we already know the maximum heating rate
					// uint32_t upperlimit = lround((1.125 * 65535.0 * profile->start_rate) / max_heat_rate);
					// upperlimit = max(upperlimit, approx_pwm(temperature_to_sensor(tgt_temp)));

					// calculate and set duty cycle
					uint16_t pwm = pid((double)temperature_to_sensor(tgt_temp), (double)cur_sensor, &integral, &last_error);
					pwm_ocr = pwm;
					// pwm_ocr = pwm > upperlimit ? upperlimit : pwm;
				}
			}

			if (stage == 1) // thermal soak stage, ensures entire PCB is evenly heated
			{
				length_cnt++;
				if (((uint16_t)lround(length_cnt * TMR_OVF_TIMESPAN * 256) > profile->soak_length))
				{
					// has passed time duration, next stage
					length_cnt = 0;
					stage++;
					integral = 0.0;
					last_error = 0.0;
				}
				else
				{
					// keep the temperature steady
					tgt_temp = (((profile->soak_temp2 - profile->soak_temp1) / profile->soak_length) * (length_cnt * TMR_OVF_TIMESPAN * 256)) + profile->soak_temp1;
					tgt_temp = min(tgt_temp, profile->soak_temp2);
					pwm_ocr = pid((double)temperature_to_sensor(tgt_temp), (double)cur_sensor, &integral, &last_error);
				}
			}

			if (stage == 2) // reflow stage, try to reach peak temp
			{
				length_cnt++;
				if (((uint16_t)lround(length_cnt * TMR_OVF_TIMESPAN * 256) > profile->time_to_peak))
				{
					// has passed time duration, next stage
					length_cnt = 0;
					stage++;
					integral = 0.0;
					last_error = 0.0;
				}
				else
				{
					// raise the temperature
					tgt_temp = (((profile->peak_temp - profile->soak_temp2) / profile->time_to_peak) * (length_cnt * TMR_OVF_TIMESPAN * 256)) + profile->soak_temp2;
					tgt_temp = min(tgt_temp, profile->peak_temp);
					pwm_ocr = pid((double)temperature_to_sensor(tgt_temp), (double)cur_sensor, &integral, &last_error);
				}
			}

			if (stage == 3) // make sure we've reached peak temperature
			{
				if (sensor_to_temperature(cur_sensor) >= profile->peak_temp)
				{
					stage++;
					integral = 0.0;
					last_error = 0.0;
					length_cnt = 0;
				}
				else
				{
					tgt_temp = profile->peak_temp + 5.0;
					pwm_ocr = pid((double)temperature_to_sensor(tgt_temp), (double)cur_sensor, &integral, &last_error);
				}
			}

			if (stage == 4) // cool down
			{
				length_cnt++;
				if (cur_sensor < temperature_to_sensor(ROOM_TEMP * 1.25))
				{
					pwm_ocr = 0; // turn off
					tgt_temp = ROOM_TEMP;
					stage++;
				}
				else
				{
					// change the target temperature
					tgt_temp = profile->peak_temp - (profile->cool_rate * TMR_OVF_TIMESPAN * 256 * length_cnt);
					uint16_t pwm = pid((double)temperature_to_sensor(tgt_temp), (double)cur_sensor, &integral, &last_error);

					// apply a upper limit to the duty cycle to avoid accidentally heating instead of cooling
					// uint16_t ap = approx_pwm(temperature_to_sensor(tgt_temp));
					// pwm_ocr = pwm > ap ? ap : pwm;
					pwm_ocr = pwm;
				}
			}

			heat_set(pwm_ocr); // set the heating element power

			graph_timer += TMR_OVF_TIMESPAN * 256;

			if ((stage != 5) && (graph_timer >= graph_tick))
			{
				graph_timer -= graph_tick;
				// it's time for a new entry on the graph
				// normaly the flag to redraw the graph would be set here, but that is not needed since it will always be redrawn

				if (temp_history_idx == (LCD_WIDTH - 1))
				{
					// the graph is longer than expected
					// so shift the graph
					for (int i = 0; i < LCD_WIDTH - 1; i++)
					{
						temp_plan[i] = temp_plan[i + 1];
						temp_history[i] = temp_history[i + 1];
					}
				}

				// shift the graph down a bit to get more room
				int32_t shiftdown = lround((ROOM_TEMP * 1.25 / settings.max_temp) * LCD_HEIGHT);

				// calculate the graph plot entries

				int32_t plan = lround((tgt_temp / settings.max_temp) * LCD_HEIGHT) - shiftdown;
				temp_plan[temp_history_idx] = plan >= LCD_HEIGHT ? LCD_HEIGHT : (plan <= 0 ? 0 : plan);

				int32_t history = lround((sensor_to_temperature(cur_sensor) / settings.max_temp) * LCD_HEIGHT) - shiftdown;
				temp_history[temp_history_idx] = history >= LCD_HEIGHT ? LCD_HEIGHT : (history <= 0 ? 0 : history);

				if (temp_history_idx < (LCD_WIDTH - 1) && (temp_plan[temp_history_idx] != 0 || temp_plan[temp_history_idx] != 0))
				{
					temp_history_idx++;
				}
			}
		}

		if (tmr_drawlcd_flag)
		{
			tmr_drawlcd_flag = 0;
			// print the graph and overlay current temperature, target temperature, and phase of reflow
			u8g.firstPage();
			do
			{	
				u8g.drawStr(38, 14, "\xb0"
									"C"); // 0xb0 is the degree sign in the unifont table
				u8g.drawStr(25, 29, "\xb0"
									 "C");
				u8g.setPrintPos(0, 14);
				u8g.print(sensor_to_temperature(cur_sensor), 1);
				u8g.setPrintPos(0, 29);
				u8g.print(tgt_temp, 0);
				switch (stage)
				{
				case 0:
					u8g.drawStr(0, 44, "Preheat");
					break;
				case 1:
					u8g.drawStr(0, 44, "Soak");
					break;
				case 2:
				case 3:
					u8g.drawStr(0, 44, "Reflow");
					break;
				case 4:
					u8g.drawStr(0, 44, "Cool");
					break;
				case 5:
					u8g.drawStr(0, 44, "Done");
					break;
				default:
					u8g.drawStr(0, 44, "oops!");
					break;
				}
				if (DEMO_MODE)
				{
					u8g.drawStr(0, 60, "DEMO");
				}

				//always draw graph, otherwise it gets erased next display refresh
				for (unsigned char x = 0; x < LCD_WIDTH; x++)
				{
					u8g.drawPixel(x, LCD_HEIGHT - temp_history[x]);
					// graph scaling is done when saving the values
				};

			} while (u8g.nextPage());
		}

		if (tmr_writelog_flag)
		{
			tmr_writelog_flag = 0;

			// print to CSV log format
			fprintf_P(&log_stream, PSTR("%d, "), stage);
			fprintf_P(&log_stream, PSTR("%s, "), str_from_double(total_cnt * TMR_OVF_TIMESPAN * 256, 1));
			fprintf_P(&log_stream, PSTR("%d, "), cur_sensor);
			fprintf_P(&log_stream, PSTR("%d, "), temperature_to_sensor(tgt_temp));

			fprintf_P(&log_stream, PSTR("%s,\n"), str_from_int(pwm_ocr));

			// fprintf_P(&log_stream, PSTR("%s, "), str_from_int(pwm_ocr));
			// fprintf_P(&log_stream, PSTR("%s,\n"), str_from_double(integral, 1));
		}

		// hold down mid button to stop
		if (button_enter())
		{
			if (stage != 5)
			{
				u8g.firstPage();
				do
				{
					u8g.drawStr(50, 28, "DONE!");
				} while (u8g.nextPage());
			}
			delay(25);
			while (button_enter())
				;
			delay(25);

			if (stage != 5)
			{
				stage = 5;
			}
			else
			{
				// release and hold down again to exit
				return;
			}
		}
		wdt_reset();
	}
}

void profile_setdefault(profile_t *profile)
{
	profile->start_rate = 1;
	profile->soak_temp1 = 150.0;
	profile->soak_temp2 = 185.0;
	profile->soak_length = 70;
	profile->peak_temp = 217.5;
	profile->time_to_peak = 45;
	profile->cool_rate = 2.0;
}

void settings_setdefault(settings_t *s)
{
	s->pid_p = 6000.0;
	s->pid_i = 20.00;
	s->pid_d = -0.00;
	s->max_temp = 225.0;
	s->time_to_max = 300.0;
}

char profile_valid(profile_t *profile)
{
	return (profile->start_rate > 0.0 &&
			profile->soak_temp1 > 0.0 &&
			profile->soak_temp2 >= profile->soak_temp1 &&
			profile->peak_temp >= profile->soak_temp2 &&
			profile->cool_rate > 0.0);
}

char settings_valid(settings_t *s)
{
	return (s->max_temp > 0.0 && s->time_to_max > 0.0);
}

static int log_putchar_stream(char c, FILE *stream)
{
	if (c == '\n')
	{
		log_putchar_stream('\r', stream);
	}

	Serial.write(c); /* uart instead of USB, using arduino */
	return 0;
}

#if 0 // for debug. It did not trigger, which is good
ISR(WDT_vect){
	PORTD|=_BV(PD7); // set debugled
	return;
}
#endif