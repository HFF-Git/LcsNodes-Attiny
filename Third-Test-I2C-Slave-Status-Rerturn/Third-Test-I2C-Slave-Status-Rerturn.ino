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
// third 214 test.... receive on I2C, set an LED, report LED status
//
// Uses: 1156 bytes of code, 51 bytes of RAM.
//=========================================================================================== 

#include <Wire.h>

#define I2C_ADDRESS 0x10
#define LED_PIN 3   // PB3 (physisch Pin 6)

volatile uint8_t led_status = 0;

void receiveEvent(int howMany)
{
  while (Wire.available())
  {
    uint8_t cmd = Wire.read();

    if (cmd == 1)
    {
      led_status = 1;
      digitalWrite(LED_PIN, HIGH);
    }
    else if (cmd == 0)
    {
      led_status = 0;
      digitalWrite(LED_PIN, LOW);
    }
  }
}

void requestEvent()
{
  Wire.write(led_status);   // Status zurücksenden
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Wire.begin(I2C_ADDRESS);      // Slave starten
  Wire.onReceive(receiveEvent); // Write Handler
  Wire.onRequest(requestEvent); // Read Handler
}

void loop()
{
}
