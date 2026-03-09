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
// Third test - collect all parts that we would need ... a place holder for now.
//
//
//=========================================================================================== 

#include <Wire.h>
#include <EEPROM.h>
#include "LcsLib.h"


struct Config {
    uint16_t field[16];
};

Config cfg;

const Config defaultConfig = {
    {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16}
};

const uint16_t EEPROM_MAGIC = 0x4A53;
const uint16_t EEPROM_VERSION = 1;

void loadConfig()
{
    uint16_t magic;
    uint16_t version;

    EEPROM.get(0, magic);
    EEPROM.get(2, version);

    if (magic != EEPROM_MAGIC || version != EEPROM_VERSION)
    {
        // EEPROM neu initialisieren
        cfg = defaultConfig;

        EEPROM.put(4, cfg);
        EEPROM.put(0, EEPROM_MAGIC);
        EEPROM.put(2, EEPROM_VERSION);
    }
    else
    {
        // EEPROM gültig
        EEPROM.get(4, cfg);
    }
}


void saveConfig() {
    EEPROM.put(0, cfg);
}

uint16_t readField(uint8_t index)
{
    uint16_t value;
    int addr = index * sizeof(uint16_t);
    EEPROM.get(addr, value);
    return value;
}

void updateField(uint8_t index, uint16_t value)
{
    cfg.field[index] = value;

    int addr = 4 + index * sizeof(uint16_t);
    EEPROM.put(addr, value);
}


//=========================================================================================== 
//
//
//
//=========================================================================================== 
struct ServoConfig {
    uint16_t lower;      // Untere Grenze
    uint16_t upper;      // Obere Grenze
    uint16_t transitionTime; // in ms
};

struct ServoState {
    uint16_t current;    // aktueller PWM Wert
    uint16_t target;     // Ziel PWM Wert
    uint32_t lastUpdate; // millis() der letzten Aktualisierung
};

ServoConfig servoConfig[2]; // RAM
ServoState servoState[2];   // RAM


//

// 0   Servo0.lower
// 2   Servo0.upper
// 4   Servo0.transitionTime
// 6   Servo1.lower
// 8   Servo1.upper
// 10  Servo1.transitionTime


void loadServoConfig() {
    for (uint8_t i=0; i<2; i++) {
        int addr = i*6;
        EEPROM.get(addr, servoConfig[i]);
        // sanity check
        if (servoConfig[i].lower >= servoConfig[i].upper) {
            servoConfig[i] = {1000, 2000, 1000}; // default
            EEPROM.put(addr, servoConfig[i]);
        }
        servoState[i].current = servoConfig[i].lower; // Default Position
        servoState[i].target = servoState[i].current;
        servoState[i].lastUpdate = millis();
    }
}

void saveServoConfig(uint8_t i) {
    int addr = i*6;
    EEPROM.put(addr, servoConfig[i]);
}

void setServoTarget(uint8_t i, uint16_t value) {
    if (value < servoConfig[i].lower) value = servoConfig[i].lower;
    if (value > servoConfig[i].upper) value = servoConfig[i].upper;
    servoState[i].target = value;
}

void updateServos() {
    uint32_t now = millis();
    static uint8_t lastMoved = 0;

    uint8_t i = (lastMoved == 0) ? 1 : 0; // alternierend
    ServoState &s = servoState[i];
    ServoConfig &c = servoConfig[i];

    if (s.current != s.target) {
        uint32_t dt = now - s.lastUpdate;
        if (dt == 0) return;

        int32_t diff = s.target - s.current;
        int32_t step = diff * dt / c.transitionTime;
        if (step == 0) step = (diff>0)?1:-1;

        s.current += step;
        if ((diff>0 && s.current > s.target) || (diff<0 && s.current < s.target))
            s.current = s.target;

        s.lastUpdate = now;

        // hier echten PWM setzen
        analogWrite(i, s.current);

        lastMoved = i;
    }
}

void setupServos() {
    pinMode(5, OUTPUT); // Servo0
    pinMode(6, OUTPUT); // Servo1
    loadServoConfig();
    analogWrite(5, servoState[0].current);
    analogWrite(6, servoState[1].current);
}

void loopServos() {
    updateServos();
    // Hier könnte I2C oder andere Logik laufen
}


//=========================================================================================== 
//
//
// choose after slowest part 

// WDT_PERIOD_8CLK_gc
// WDT_PERIOD_16CLK_gc
// WDT_PERIOD_32CLK_gc
// WDT_PERIOD_64CLK_gc
// WDT_PERIOD_128CLK_gc
// WDT_PERIOD_256CLK_gc
// WDT_PERIOD_512CLK_gc
// WDT_PERIOD_1KCLK_gc     about 1 ms
// WDT_PERIOD_2KCLK_gc
// WDT_PERIOD_4KCLK_gc
// WDT_PERIOD_8KCLK_gc     abut  8 ms
 

//=========================================================================================== 

#include <avr/wdt.h> // megaTinyCore supports AVR WDT

void watchdog_enable()
{
    _PROTECTED_WRITE( WDT.CTRLA, WDT_PERIOD_8KCLK_gc );
}


void watchdog_feed()
{
    __builtin_avr_wdr();
}



//=========================================================================================== 
//
//
//
//=========================================================================================== 
#define I2C_ADDRESS 0x22      // Just an I2C number
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
