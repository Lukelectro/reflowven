#ifndef menu_h
#define menu_h

#include "reflowtoasteroven.h" // for profile_t

 // changes a value based on the up and down buttons
double button_change_double(double oldvalue, double increment, double limit1, double limit2);
long button_change_int(long oldvalue, long increment, long limit1, long limit2);
char* str_from_int(signed long value);
char* str_from_double(double value, int decimalplaces);
void menu_manual_pwm_ctrl();
void menu_manual_temp_ctrl();
void menu_edit_profile(profile_t* profile);
void menu_auto_mode();
void menu_edit_settings();
void main_menu();
 
 #endif