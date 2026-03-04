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
// second 214 test.... receive on I2C, set an LED
//
// Uses: 1126 bytes of code, 50 bytes of RAM.
//=========================================================================================== 

#include <Wire.h>

#define I2C_ADDRESS 0x3C  // adapt to LCS lib Ext Nvm channel for test.
#define LED_PIN 3         // PB3 (physisch Pin 6)

void receiveEvent(int howMany)
{
  while (Wire.available())
  {
    uint8_t cmd = Wire.read();

    if (cmd == 1)
      digitalWrite(LED_PIN, HIGH);
    else if (cmd == 0)
      digitalWrite(LED_PIN, LOW);
  }
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Wire.begin(I2C_ADDRESS);   // Slave starten
  Wire.onReceive(receiveEvent);
}

void loop()
{
}
