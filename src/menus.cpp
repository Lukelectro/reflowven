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
 * This file handles the behaviours of the main menu and all submenues
 *
 */

 /* partly rewritten by Lucas Volwater 2025 for use with a u8glib display and a rotary encoder,
  * e.g a reprapdiscount full graphic smart controller or similar 
  * Licence same as above
  */

#include "reflowtoasteroven.h"
#include "lcd.h"
#include "heatingelement.h"
#include "temperaturemeasurement.h"
#include "userinput.h"
#include "lcd.h"
#include "nvm.h" // so settings can be loaded and saved
#include <string.h>
#include <stdlib.h>
#include <avr/pgmspace.h>

// this string is allocated for temporary use
char strbuf[(LCD_WIDTH / FONT_WIDTH) + 2];

//change a value, between limits, with the rotary encoder, for use in loops and with doubles.
double change_value_double(double oldvalue, double increment, double limit1, double limit2){
	double temp;
	double maxlimit = limit1 >= limit2 ? limit1 : limit2;
	double minlimit = limit1 < limit2 ? limit1 : limit2;

	temp = oldvalue + (RotEnc.readAndReset() / (ROTENC_PPS) * increment);

	return (temp > maxlimit) ? maxlimit : ((temp < minlimit) ? minlimit : temp);
}

//same as above, but for ints
int32_t change_value_int(int32_t oldvalue, int32_t increment, int32_t limit1, int32_t limit2){
	int32_t temp;
	int32_t maxlimit = limit1 >= limit2 ? limit1 : limit2;
	int32_t minlimit = limit1 < limit2 ? limit1 : limit2;

	temp = oldvalue + (RotEnc.readAndReset() / (ROTENC_PPS) * increment);

	return (temp > maxlimit) ? maxlimit : ((temp < minlimit) ? minlimit : temp);
}

char *str_from_int(signed long value)
{
	return ltoa(value, strbuf, 10);
}

char *str_from_double(double value, int decimalplaces)
{
	char *result = dtostrf(value, -(LCD_WIDTH / FONT_WIDTH), decimalplaces, strbuf);

	// trim trailing spaces
	int8_t len = strlen(result);
	len--;
	while (result[len] == ' ' && len >= 0)
	{
		result[len] = 0;
		len--;
	}
	return result;
}

void menu_manual_pwm_ctrl()
{
	uint16_t iteration = 0;
	int32_t rot_enc_val;
	uint16_t pwm = 0;
	uint16_t cur_sensor = sensor_read();
	uint16_t cur_temp = 0;
	unsigned long prevmillis;

	sensor_filter_reset();
	settings_load(&settings); // load from eeprom

	// signal start of mode in log
	fprintf_P(&log_stream, PSTR("manual PWM control mode,\n"));
	
	RotEnc.write(0);
	heat_set(0);
	
	while (1)
	{
		heat_set(pwm);
		rot_enc_val = 1024*RotEnc.read()/ROTENC_PPS;

		if(rot_enc_val>65535){
			RotEnc.write(1+65535*ROTENC_PPS/1024);
			rot_enc_val=65535;
		} 
		else if (rot_enc_val<0)
		{  
			RotEnc.write(0);
			rot_enc_val=0;
		}
		pwm=rot_enc_val;

		// picture loop
		u8g.firstPage();
		do
		{	
			u8g.drawStr(0, 12, "PWM");
			u8g.drawStr(0, 28, "SENSOR");
			u8g.drawStr(110, 28, "\xb0""C"); // 0xb0 is the degree-sign in the unifont table
			u8g.drawStr(0, 60, "Tick");
			u8g.setPrintPos(80, 28);
			u8g.print(cur_temp, DEC);
			u8g.setPrintPos(80, 12);
			u8g.print(pwm, DEC);
			u8g.setPrintPos(80, 28);
			u8g.print(cur_temp, DEC);
			u8g.setPrintPos(80, 60);
			u8g.print(iteration, DEC);
		} while (u8g.nextPage());

		if(cur_temp > settings.max_temp){ // very rudimentary overtemperature protection. ALSO USE A THERMO FUSE!
			pwm=0;
			RotEnc.write(0);			// also reset the rotary encoder to zero, so it needs human interaction to change the PWM from 0 even after oven cooled down a bit
		}

		// return on buttonpress, untill then loop and call PID at regular intervals
		if (button_enter())
		{
			delay(25); 
			while (button_enter())
				;
			delay(25);
			RotEnc.write(0); // reset rotary encoder on exit...
			return;
		}

		if (millis() - prevmillis > 500)
		{
			// every half a second, read temperature and run PID
			prevmillis = millis();
			iteration++;
			cur_sensor = sensor_read();
			cur_temp = sensor_to_temperature(cur_sensor);
		if(iteration&0x01)
			{
				// every second, write log too
				fprintf_P(&log_stream, PSTR("%s, "), str_from_double(iteration / 2, 1));
				fprintf_P(&log_stream, PSTR("%s, "), str_from_int(cur_sensor));
				fprintf_P(&log_stream, PSTR("%s,\n"), str_from_int(pwm));
			}
		}
	}
}

void menu_manual_temp_ctrl()
{
	uint16_t iteration = 0;
	uint16_t cur_pwm = 0;
	double tgt_temp = 0;
	double integral = 0.0, last_error = 0.0;
	uint16_t tgt_sensor = temperature_to_sensor((double)tgt_temp);
	uint16_t cur_sensor = sensor_read();
	uint16_t cur_temp = 0;
	unsigned long prevmillis;

	sensor_filter_reset();

	settings_load(&settings); // load from eeprom

	// signal start of mode in log
	fprintf_P(&log_stream, PSTR("manual temperature control mode,\n"));
	
	RotEnc.write(0);
	heat_set(0);
	
	while (1)
	{
		heat_set(cur_pwm);

		//todo: this can be done smarter/faster then with a lot of doubles, maybe look into later if needed. Or could use change_value_double()...); function...
		tgt_temp = RotEnc.read()/ROTENC_PPS;

		if(tgt_temp>settings.max_temp){
			tgt_temp = settings.max_temp;
			RotEnc.write(tgt_temp*ROTENC_PPS);
		} 
		else if (tgt_temp<0)
		{  
			tgt_temp = 0;
			RotEnc.write(0);
		}
		
		// picture loop
		u8g.firstPage();
		do
		{	
			u8g.drawStr(25, 14, "\xb0""C Set"); // 0xb0 is the degree sign in the unifont table
			u8g.drawStr(110, 14, "\xb0""C");
			u8g.drawStr(0, 29, "PWM @ ");
			u8g.drawStr(0, 43, "Tick");
			u8g.setPrintPos(0, 14);
			u8g.print(cur_temp, DEC);
			u8g.setPrintPos(85, 14);
			u8g.print(tgt_temp, 0);
			u8g.setPrintPos(85, 29);
			u8g.print(cur_pwm, DEC);
			u8g.setPrintPos(85, 43);
			u8g.print(iteration, DEC);
		} while (u8g.nextPage());
		//TODO: maybe draw a temperature graph below the text?

		// return on buttonpress, untill then loop and call PID at regular intervals
		if (button_enter())
		{
			delay(25); 
			while (button_enter())
				;
			delay(25);
			RotEnc.write(0); // reset rotary encoder on exit...
			return;
		}

		if (millis() - prevmillis > 500)
		{
			// every half a second, read temperature and run PID
			prevmillis = millis();
			iteration++;
			cur_sensor = sensor_read();
			cur_temp = sensor_to_temperature(cur_sensor);
			tgt_sensor = temperature_to_sensor((double)tgt_temp); // todo: maybe convert this just once after setting tgt?
			cur_pwm = pid((double)tgt_sensor, (double)cur_sensor, &integral, &last_error);
		if(iteration&0x01)
			{
				// every second, write log too
				// fprintf_P(&log_stream, PSTR("%s, "), str_from_double(iteration * TMR_OVF_TIMESPAN * 512, 1));
				fprintf_P(&log_stream, PSTR("%s, "), str_from_double(iteration / 2, 1));
				fprintf_P(&log_stream, PSTR("%s, "), str_from_int(cur_sensor));
				fprintf_P(&log_stream, PSTR("%s, "), str_from_int(tgt_temp));
				fprintf_P(&log_stream, PSTR("%s,\n"), str_from_int(cur_pwm));
			}
		}
	}
}

void menu_edit_profile(profile_t *profile)
{
	//profile_load(&profile); // load from eeprom -- already loaded in "auto reflow" menu, which is the only way to enter this.
	unsigned char selection = 0;
	fprintf_P(&log_stream, PSTR("Edit profile Menu,\n"));
	unsigned char selecting = 1;

	while (1)
	{
		heat_set(0); // turn off for safety
		
		// u8glib picture loop
		u8g.firstPage();
		do
		{
			if(selection<4){
			u8g.drawStr(0, 12 + 16 * selection, ">"); // mark selection
			u8g.drawStr(6, 12, "Start Rate");
			u8g.drawStr(6, 28, "Soak 1");
			u8g.drawStr(6, 44, "Soak 2");
			u8g.drawStr(6, 60, "Soak time"); 
			u8g.drawStr(110, 28, "\xb0""C");
			u8g.drawStr(110, 44, "\xb0""C");
			u8g.drawStr(120, 60, "Soak time"); 
			// the degree sign is 176 in the unifont font table, but without prefixed x the number is octal so x0b was simpler, but the the C is seen as hex too, so use string concatenation
			u8g.setPrintPos(100, 12);
			u8g.print(profile->start_rate, 1);
			u8g.setPrintPos(85, 28);
			u8g.print(profile->soak_temp1, 0);
			u8g.setPrintPos(85, 44);
			u8g.print(profile->soak_temp2, 0);
			u8g.setPrintPos(85, 60);
			u8g.print(profile->soak_length, 1);
			}
			else{
				u8g.drawStr(0, 12 + 16 * (selection-4), ">"); // mark selection
				u8g.drawStr(6, 14, "Peak T");
				u8g.drawStr(6, 29, "Peak t");
				u8g.drawStr(110, 14, "\xb0""C");
				u8g.drawStr(110, 29, "s");
				u8g.drawStr(6, 44, "Cool Rate");
				u8g.drawStr(6, 60, "Save & exit"); // TODO: option to exit without saving
				u8g.setPrintPos(85, 12);
				u8g.print(profile->peak_temp, 0);
				u8g.setPrintPos(85, 28);
				u8g.print(profile->time_to_peak, DEC);
				u8g.setPrintPos(85, 44);
				u8g.print(profile->cool_rate, 2);
			}
		} while (u8g.nextPage());
		
		if (button_enter())
		{
			delay(25);
			while (button_enter())
				;
			delay(25);
			RotEnc.write(0); // reset rotary encoder before entering next mode...
			selecting ^= 0x01 ; // exor = toggle selecting - if enter is pressed on a value edit that value untill enter is pressed again.

			// act on the selection (return to main or reset...)
			if(selection == 7){
				 // save settings and return to main
					if (profile_valid(profile))
					{
						profile_save(profile); // save to eeprom
						return;
					}
					else
					{
						u8g.firstPage();
						do
						{
						u8g.drawStr(0, 12, "Error in pro-");
						u8g.drawStr(0, 28, "file, please");
						u8g.drawStr(0, 28, "Review & fix");
						} while (u8g.nextPage());
						_delay_ms(1000);
						selecting = 1; // back to select which setting to correct
					}
				}
		}

		// edit values if not selecting
		if(selecting == 1){
			selection = (RotEnc.read() / ROTENC_PPS);
			if(selection>254){ 		// underflow
				RotEnc.write(7*ROTENC_PPS); 
			}else if (selection>7){ // overflow
				RotEnc.write(0);
			}
		}
		else
		{
			switch(selection){
			case 0:
				profile->start_rate = change_value_double(profile->start_rate, 0.1, 0.1, 5.0);
				break;
			case 1:
				profile->soak_temp1 = change_value_double(profile->soak_temp1, 1, 50, 300);
				break;
			case 2:
				profile->soak_temp2 = change_value_double(profile->soak_temp2, 1, 50, 300);
				break;
			case 3:
				profile->soak_length = change_value_int(profile->soak_length, 1, 60, 60*5);
				break;
			case 4:
				profile->peak_temp = change_value_double(profile->peak_temp, 1, 150, 350);
				break;
			case 5:
				profile->time_to_peak = change_value_int(profile->time_to_peak, 1, 0, 60*5);
				break;
			case 6:
				profile->cool_rate = change_value_double(profile->cool_rate, 0.1, 0.1, 5.0);
				break;
			default:
				break;
			}
		}
	}
}

void menu_auto_mode()
{

	unsigned char selection = 0;
	static profile_t profile;
	profile_load(&profile); // load from eeprom
	fprintf_P(&log_stream, PSTR("Auto Mode Menu,\n"));
	heat_set(0); // start with heater OFF

	while (1)
	{
		heat_set(0); // turn off for safety
		selection = (RotEnc.read() / ROTENC_PPS) & 0x03; // only allow 0,1,2,3 - mask instead of modulo so it wont go negative either.

		// u8glib picture loop
		u8g.firstPage();
		do
		{
			u8g.drawStr(0, 12 + 16 * selection, ">"); // mark selection (font heigt * selection modulo number of items, distance, marker) TODO: use font height and width read from font setting if possible
			u8g.drawStr(6, 12, "Start");
			u8g.drawStr(6, 28, "Edit profile");
			u8g.drawStr(6, 44, "Reset profile");
			u8g.drawStr(6, 60, "Back to main");
		} while (u8g.nextPage());
		
		if (button_enter())
		{
			delay(25); 
			while (button_enter())
				;
			delay(25);
			RotEnc.write(0); // reset rotary encoder before entering next mode...

			// enter the submenu that is selected
		switch(selection){
			case 0: // start
				auto_go(&profile); // TODO: maybe make something to select from multiple profiles?
				return;// go back to home menu when finished
				break;
			case 1: // edit profile
				menu_edit_profile(&profile);
			break;
			case 2: // reset profile
				profile_setdefault(&profile);
				profile_save(&profile); // save to eeprom
				u8g.firstPage();
				do
				{
					u8g.drawStr(0, 28, "Profile");
					u8g.drawStr(0, 44, "is reset !");
				} while (u8g.nextPage());
				delay(1000);
			break;
			case 3: // back to main menu
				return;
				break;
			default:
				break;
			}
		}
	}
}

void menu_edit_settings()
{
	heat_set(0); // heater off
	settings_load(&settings); // load from eeprom
	unsigned char selection = 0;
	fprintf_P(&log_stream, PSTR("Edit Settings Menu,\n"));
	unsigned char selecting = 1;

	while (1)
	{
		 heat_set(0); // turn off for safety
		
		// u8glib picture loop
		u8g.firstPage();
		do
		{
			if(selection<4){
			u8g.drawStr(0, 12 + 16 * selection, ">"); // mark selection
			u8g.drawStr(6, 12, "PID P =");
			u8g.drawStr(6, 28, "PID I =");
			u8g.drawStr(6, 44, "PID D =");
			//u8g.drawStr(6, 60, "Max °C"); // TODO: fix display of degree sign °, it shows as ã° -ish. 
			u8g.drawStr(6, 60, "Max\xb0""C"); 
			// the degree sign is 176 in the unifont font table, but without prefixed x the number is octal so x0b was simpler, but the the C is seen as hex too, so use string concatenation
			u8g.setPrintPos(70, 12);
			u8g.print(settings.pid_p, 2);
			u8g.setPrintPos(70, 28);
			u8g.print(settings.pid_i, 2);
			u8g.setPrintPos(70, 44);
			u8g.print(settings.pid_d, 2);
			u8g.setPrintPos(70, 60);
			u8g.print(settings.max_temp, 1);
			}
			else{
				u8g.drawStr(0, 12 + 16 * (selection-4), ">"); // mark selection
				u8g.drawStr(6, 12, "Time to Max");
				u8g.setPrintPos(100, 12);
				u8g.print(settings.time_to_max, 0);
				u8g.drawStr(6, 28, "Reset defaults");
				u8g.drawStr(6, 44, "Save & exit");
				u8g.drawStr(6, 60, "Cancel & exit");
			}
		} while (u8g.nextPage());
		
		if (button_enter())
		{
			delay(25);
			while (button_enter())
				;
			delay(25);
			RotEnc.write(0); // reset rotary encoder before entering next mode...
			selecting ^= 0x01 ; // exor = toggle selecting - if enter is pressed on a value edit that value untill enter is pressed again.

			// act on the selection (return to main or reset...)
			switch(selection){
				case 5: // reset default
						settings_setdefault(&settings);
						selecting = 1;
				break;
				case 6: // save settings and return to main
					if (settings_valid(&settings))
					{
						settings_save(&settings); // save to eeprom

						// back to main menu
						return;
					}
					else
					{
						u8g.firstPage();
						do
						{
						u8g.drawStr(0, 12, "Error in set-");
						u8g.drawStr(0, 28, "tings, please");
						u8g.drawStr(0, 28, "Review & fix");
						} while (u8g.nextPage());
						_delay_ms(1000);
						selecting = 1; // back to select which setting to correct
					}
					break;
				case 7: // discard changes and return to main
					return;
				break;
				default:
				break;
			}
		}

		// edit values if not selecting
		if(selecting == 1){
			selection = (RotEnc.read() / ROTENC_PPS);
			if(selection>254){ 		// underflow
				RotEnc.write(7*ROTENC_PPS); 
			}else if (selection>7){ // overflow
				RotEnc.write(0);
			}
		}
		else
		{
			switch(selection){
				case 0: // PID P
					settings.pid_p = change_value_double(settings.pid_p, 0.1, 0.0, 10000.0);
				break;
				case 1: // PID I
					settings.pid_i = change_value_double(settings.pid_i, 0.01, 0.0, 10000.0);
				break;
				case 2: // PID D
					settings.pid_d = change_value_double(settings.pid_d, 0.01, -10000.0, 10000.0);
				break;
				case 3:	// maximum temperature
				settings.max_temp = change_value_double(settings.max_temp, 1.0, 200.0, 350.0);
				break;
				case 4: // time to maximum
				settings.time_to_max = change_value_double(settings.time_to_max, 1.0, 0.0, (double) 60*20);
				break;
				default:
				break;
			}
		}
	}
}


void main_menu() // main menu is also main loop.
{
	unsigned char selection = 0;
	fprintf_P(&log_stream, PSTR("Main Menu,\n"));

	while (1)
	{
		heat_set(0); // turn off for safety
		selection = (RotEnc.read() / ROTENC_PPS) & 0x03; // only allow 0,1,2,3 - mask instead of modulo so it wont go negative either.

		// u8glib picture loop
		u8g.firstPage();
		do
		{
			u8g.drawStr(0, 12 + 16 * selection, ">"); // mark selection (font heigt * selection modulo number of items, distance, marker) TODO: use font height and width read from font setting if possible
			u8g.drawStr(6, 12, "Auto Reflow");
			u8g.drawStr(6, 28, "Set Temperature");
			u8g.drawStr(6, 44, "Set PWM");
			u8g.drawStr(6, 60, "Edit Settings");
			//u8g.setPrintPos(110, 12);
			//u8g.print(selection, DEC);
		} while (u8g.nextPage());
		// delay(500); // should instead todo: use millis() for polling and make a superloop that way that also reads temperature and does PID at a set rate. -- meh, for set manual temperature and for reflow. So best make the PID a reusable function

		if (button_enter())
		{
			delay(25); 
			while (button_enter())
				;
			delay(25);
			RotEnc.write(0); // reset rotary encoder before entering next mode...

			// enter the submenu that is selected

			if (selection == 0)
			{
				menu_auto_mode();
			}
			else if (selection == 1)
			{
				menu_manual_temp_ctrl();
			}
			else if (selection == 2)
			{
				menu_manual_pwm_ctrl();
			}
			else if (selection == 3)
			{
				menu_edit_settings();
			}
		}
	}
}