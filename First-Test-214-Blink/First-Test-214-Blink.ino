//===========================================================================================
//
// Install megaTinyCore, use UDE 1.8.13 or 19.
//
// Build an adapter: 

// UDPI <---- ---------------< RX
//           |
//           --- R --- >| ---< TX          
//
// Diode: 1N5819 ( white ring to TX )
// R:     4K7
//
// Both serial adapter work. ( serial UDPI, high speed )
//
//=========================================================================================== 

//=========================================================================================== 
// first 214 test.... blink an LED
//
// Uses: 434 bytes of code, 10bytes of RAM.
//=========================================================================================== 
#include <Wire.h>
#define LED_PIN PIN_PB3

void setup() {
  pinMode(LED_PIN, OUTPUT);   
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
}
