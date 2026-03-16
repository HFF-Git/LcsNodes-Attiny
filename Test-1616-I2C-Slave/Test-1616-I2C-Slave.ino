//===========================================================================================
//
// Install megaTinyCore, use UDE 1.8.13 or 19.
//
// Build an adapter: 
//
// TX --1k--+
//         +---- UPDI
// RX ------+
//
// Both serial adapter work. ( serial UDPI, high speed )
//
//=========================================================================================== 

//=========================================================================================== 
// second 1616 test.... respond to our I2C address
//
//=========================================================================================== 
#include "LcsLib.h"
#include <Wire.h>

#define I2C_ADDRESS 0x33      // Just an I2C number
#define LED_PIN     PIN_PB3

void receiveEvent( int howMany ) {
  
  while ( Wire.available( )) {
     
    uint8_t cmd = Wire.read( );
  }
}

void requestEvent( ) {

  Wire.write( 0x00 );
}

void setup( ) {

  pinMode(LED_PIN, OUTPUT);

  Wire.begin( I2C_ADDRESS );   // Slave starten
  delay( 10 );
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop( ) {

  digitalWrite(LED_PIN, LOW);
  delay( 500 );
  digitalWrite(LED_PIN, HIGH);
  delay( 500 );
}
