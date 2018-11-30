/************ Some AVR-dude commands useful for copy/paste to set fuses appropriately and to program the ATMega328p via an Arduino as programer (modify as needed depending on particular hardware)  *************************************************/
/* avrdude -P COM5 -b 19200 -c avrisp -p m328p -u -U flash:w:"<<FILLIN: local location and name of hex.-file>>":i */
/* avrdude -c avrisp -p m328p -P com5 -b 19200 -U lfuse:w:0xE2:m */
/* avrdude -c avrisp -p m328p -P com5 -b 19200 -U hfuse:w:0xDD:m   */


#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <TinyGPS++.h>

// some testlines here to se where this edit ends up in branch, main etc...

// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; //FILLIN TTN's "Network session key" in the format: 0x00, 0x00,... 0x00

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; //FILLIN TTN's "Application session key" in the format: 0x00, 0x00,... 0x00

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x12345678 ; //FILLIN TTN's "Device address" in the format: 0x12345678 where "12345678" is the number shown on the corresponding TTN account WEB page

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// variable(s) to store up/downlinkdata and corresponding counters
uint8_t downlinkbytes[10]; // can hold up to 10 bytes
uint8_t uplinkbytes[10]; // can hold up to 10 bytes
uint16_t uplinkcounter = 100; // must exceed the corresponding counter at TTN for the message to be accepted
uint16_t downlinkcounter = 100;

// seconds counter feeded by GPS via ISR
volatile uint8_t Seconds = 0; 

//mic variables
uint8_t n; // general loop-counter

/********* Hardware wired as follows, * indicates essential pin to connect for LoRaWAN and GPS, additional connections are optional for further functionalities of this code...... 

ATMega328p(Arduino#)           RTM96         JUMPER         Board                     Display                        GPS-MTK3339

*PD2 (2) [INT0] <------------ IRQ/DIO0
*PD6 (6) <------------------> DIO1
*PD7 (7) <------------------> DIO2
*PB1 (9) -------------------> RST            
*PB2 (10) ------------------> CS/NSS 
*PB3 (11) ------------------> MOSI       <----ON/OFF      ISP-MOSI
*PB4 (12) <------------------ MISO       <----ON/OFF      ISP-MISO
*PB5 (13) ------------------> CLOCK/SCK  <----ON/OFF      ISP-SCK
 PC3 (17/A3) <--------------> DIO3       ------OR-----    "DS18B20-pin" 
 PC2 (16/A2) <--------------> DIO4       ------OR-----    "AUX2-pin" (5V close) 
 PC1 (15/A1) <--------------> DIO5       ------OR-----    "AUX1-pin" (3V3 close)-----> RST
 PC0 (14/A0) <----------------------------------------    "5VADC-pin" 
 PB0 (8) -------------------------------------------->    HC165-SL/*PL
 PB6 (20) <-------------------------------------------    HC165-DATA
 PB7 (21) ------------------------------------------->    HC165-CLK
 PC5 (19/A5) ---------------------------------------->    "DISPLAY-pin"-SCL ---------> I2C-CLOCK
 PC4 (18/A4) <----------------------------------------    "DISPLAY-pin"-SDA ---------> I2C-DATA
*PD5 (5) <------------------------------------------->    "JS1"(-DATA INorOUT)<--------------------------------> pin7/VBAT,RESET
*PD1 (1) [TX]<--------------------------------------->    "JS2"(-DATA INorOUT)---------------------------------> pin4/RX (input to GPS)
*PD3 (3) [INT1] <------------------------------------>    "JS3"(-ENABLE)<--------------------------------------- pin1/PPS (1Hz pulses)
*PD4 (4) <------------------------------------------->    "JS4"(-CLOCK INorOUT)<-------------------------------- pin6/3DFIX (LOW when locked)
*PD0 (0) [RX]<--------------------------------------->    "JS5"(-CLOCK INorOUT)<-------------------------------- pin5/TX (output from GPS) 
  
*/

// Pin mapping HopeRF
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7}, // DIO0, DIO1 and DIO2
};

//Pin mapping and initialization of GPS
 #define GPS_PPS 3
 #define GPS_3DFIX 4
 #define GPS_RESET 5
 //#define GPS_TX PD0 // greyed out since initiated by Serial.begin() herein below
 //#define GPS_RX PD1 // greyed out since initiated by Serial.begin() herein below
 TinyGPSPlus gps; // initialize GPS

void onEvent (ev_t ev) {  
    switch(ev) {
      case EV_TXCOMPLETE:
        if(LMIC.dataLen) {
              // TODO: code here to increment downlinkcounter and make permanent storage in EEPROM of next higher 10th or 100d
              for (int i = 0; i < LMIC.dataLen; i++) { // store downlink payload to array of bytes (max 10 bytes!)
                downlinkbytes[i] = LMIC.frame[LMIC.dataBeg + i];
              }
        } 
        // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
      break;
    } 
}
 

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        //TODO: code here to make the user aware of the fact that sending is pending / not currently proceeding
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, uplinkbytes, sizeof(uplinkbytes), 0);
        // TODO: code here to increment uplinkcounter and make permanent storage in EEPROM of next higher 10th or 100d
    }
    // Next TX is scheduled after TX_COMPLETE event.
}


void serialEvent()
/*
  SerialEvent occurs whenever new data comes in the hardware serial RX (from the GPS). This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/ 
{
  while (Serial.available()) 
  { 
   gps.encode(Serial.read()); 
  }
  //newSerial = true;
}


// counts up every sec as triggered by interrupt depending on the rising edge PPS-signal from the GPS
void SecsCounter() 
{
  Seconds++;
}

// union-stuff needed to convert from float to 4 bytes
typedef union
{
 float number;
 uint8_t bytes[4];
} FLOATUNION_t;

FLOATUNION_t myFloat;


void setup() {

  // setting GPS-pins to a suitable initial state............................................
  DDRD &= ~(1 << PD3);  
  PORTD &= ~(1 << PD3); // floating input
  DDRD &= ~(1 << PD4);  
  PORTD |= (1 << PD4); // pulled-up input
  DDRD &= ~(1 << PD5);  
  PORTD |= (1 << PD5); // pulled-up input

    Serial.begin(9600); // initialize serial for communication with GPS
    attachInterrupt(digitalPinToInterrupt(GPS_PPS), SecsCounter, RISING); // begin counting seconds on basis of the PPS-signal from GPS

    //TODO: confirm that code below is not needed and then remove
    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    // TODO: increase margin of error below to be on the safe side that downlink messages are withing the catching time-slot, shouldn't cost to much in battery-time etc...
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  // TODO: Catch up on the function of "PROGMEM" to understand if the code below is needed or not...
    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are to be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

   //TODO: Investigate further on how the TX-power may be regulated, to enable "full-power-out" if such should be found to be needed (e.g. for long range communication) 
    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

   // TODO: add further code here to set starting downlink/uplink-counters based on permanent storage in EEPROM
    LMIC.seqnoUp=uplinkcounter;

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();

    if (Seconds >= 10) // poll every 10 secs and check for new incomming GPS-data
    {
     if (gps.location.age() < 10000) // extract and show the latest confirmed location if updated within the last 10 seconds
     {

       myFloat.number = gps.location.lng(); // Assign LONGITUDE to the bufferfloat
        for (n=0; n<4; n++)
        {
         uplinkbytes[n] = myFloat.bytes[n]; // populate the first 4 bytes with a representation of the (float) LONGITUDE-data 
        }
       myFloat.number = gps.location.lat(); // Assign LATITUDE to the bufferfloat
        for (n=0; n<4; n++)
        {
         uplinkbytes[n+4] = myFloat.bytes[n]; // populate the following 4 bytes ([4] to [7]) with a representation of the (float) LATITUDE-data 
        }
      uplinkbytes[8] = (uint8_t) gps.speed.kmph();
      uplinkbytes[9] = downlinkbytes[0]; // populate an uplinkbyte with the last downlinkbyte to be able to verify receival (functional downlink)
      Seconds = 0; // reset the timer
     }
    }
}












