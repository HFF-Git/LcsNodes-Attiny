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

void setup() {
  pinMode(3, OUTPUT);   // PB4 (physical pin 6)
}

void loop() {
  digitalWrite(4, HIGH);
  delay(500);
  digitalWrite(4, LOW);
  delay(500);
}
