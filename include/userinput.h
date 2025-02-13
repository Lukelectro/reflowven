#ifndef userinput_h
#define userinput_h
#include <Arduino.h>
#include "Encoder.h"

extern Encoder RotEnc;
void buttons_init();

#define button_enter() bit_is_clear(PIND, 4)

#endif