#include <Arduino.h>
#include "U8glib.h"
#include "Encoder.h" // Paul Stofregen encoder library

//test connection with LCD and LCD type on reprapdiscount full graphic smart controller -- works!
U8GLIB_ST7920_128X64_1X u8g(A3,A5,A4);	// SPI Com: SCK = en = LCD4 = PC3 = A3, MOSI = rw = SID = LCDE = PC5 = A5, CS = di = RS =LCDRS =PC4 = A4
//  The complete list of supported 
// devices with all constructor calls is here: https://github.com/olikraus/u8glib/wiki/device

Encoder RotEnc(2,3);

//u8glib draw demo
void draw(void) {
  // graphic commands to redraw the complete screen should be placed here  
  
  // assign default font
  u8g.setFont(u8g_font_unifont);
  
  // pointer to strings in flash memory can be stored in a special type
  const __FlashStringHelper *flash_ptr;
  
  // the result of the F() macro can be assigned to this pointer
  flash_ptr = F("Hello World!");
  
  // this pointer can be used as argument to the draw procedures
  u8g.drawStr( 0+1, 20+1, flash_ptr);
  u8g.drawStr( 0, 20, flash_ptr);
  
  // of course, the F() macro can be used directly
  u8g.drawStr( 0, 40, F("Rot. Enc. Test:"));

  int16_t pos = RotEnc.read();
  // Convert the integer to a string
  char buffer[10];
  sprintf(buffer, "%6d", pos);

  // Set the font and draw the string
  u8g.setFont(u8g_font_6x10);
  u8g.drawStr(0, 50, buffer);

}

void setup(void) {
  // flip screen, if required
  // u8g.setRot180();
}

void loop(void) {
  // picture loop
  u8g.firstPage();  
  do {
    draw();
  } while( u8g.nextPage() );

  // rebuild the picture after some delay
  delay(500);
}