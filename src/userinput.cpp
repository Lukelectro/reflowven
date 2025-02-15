#include "Encoder.h"
#include "userinput.h"
#include <Arduino.h>

Encoder RotEnc(2,3); // rotary encoder connected to interrupt pins, pins 2 and 3 on arduino uno, pd2 and pd3 on atmega328
void button_init(){
    pinMode(4,INPUT_PULLUP); // button on rotary encoder connected to pd4 = pin 4 on arduino uno
}