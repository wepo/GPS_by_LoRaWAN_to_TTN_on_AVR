/************ Some AVR-dude commands useful for copy/paste to set fuses appropriately and to program the ATMega328p via an Arduino as programer (modify as needed depending on particular hardware)  *************************************************/
/* avrdude -P COM5 -b 19200 -c avrisp -p m328p -u -U flash:w:"<<hex.-file>>":i */
/* avrdude -c avrisp -p m328p -P com5 -b 19200 -U lfuse:w:0xE2:m */
/* avrdude -c avrisp -p m328p -P com5 -b 19200 -U hfuse:w:0xDD:m   */


/******************************************************************************************************************************************************************************************************************************************************/
/****************************** PRE-SETUP SECTION BELOW ***********************************************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************************************************************************************************/
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <EEPROM.h>

/**************************** VARIOUS SETTINGS AND KEYS AFFECTING THE BEHAVIOUR OF THE NODE THIS CODE RUNS ON *********************************************/

// LoRaWAN NwkSKey, network session key
//static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; //FILLIN TTN's "Network session key" in the format: 0x00, 0x00,... 0x00

// LoRaWAN AppSKey, application session key
//static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; //FILLIN TTN's "Application session key" in the format: 0x00, 0x00,... 0x00

// LoRaWAN end-device address (DevAddr)
//static const u4_t DEVADDR = 0x12345678 ; //FILLIN TTN's "Device address" in the format: 0x12345678 where "12345678" is the number shown on the corresponding TTN account WEB page

// Schedule GPS-data capture and TX every this many seconds, when in default LowPwrMode (might become longer due to duty cycle limitations)
const unsigned TX_INTERVAL_LONG = 60;

// Schedule GPS-data capture and TX every this many seconds, when in high alert mode (i.e. LowPwrMode == 0) (might become longer due to duty cycle limitations)
const unsigned TX_INTERVAL_SHORT = 5;

// Number of uplinkbytes (i.e. suitable array size, with optional margin, to store the payload) 
#define NO_UPLINKBYTES 10

// Number of downlinkbytes (i.e. suitable array size, with optional margin, to store the payload) 
#define NO_DOWNLINKBYTES 10

// percentage value that the clock may vary. Should be larger the more the clock may vary to be able to catch the "downlink time slot". "1" seems to work fine, but may be larger, e.g. 2-50.
#define CLOCK_ERROR_MARGIN 1

// maximal internal age (within the GPS-chip) in seconds for GPS-data to be considered for copying over to related array-variables
#define GPS_MAX_INTERNAL_AGE 2

// maximal age in seconds for GPS-data stored in related array-variables to be considered valid, should be greater than GPS_MAX_INTERNAL_AGE  
#define GPS_MAX_TOTAL_AGE 10

/************************** END - VARIOUS SETTINGS AND KEYS AFFECTING THE BEHAVIOUR OF THE NODE THIS CODE RUNS ON *****************************************/

/**************************** PIN-MAPPING *******************************************************************************************************************************************************/

//Hardware wired as follows, * indicates essential pin to connect for LoRaWAN and GPS, additional connections are optional for further functionalities of this code.
/*
Hardware wired as follows, * indicates essential pin to connect for LoRaWAN and GPS, additional connections are optional for further functionalities of this code.

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
*PB0 (8) -------------------------------------------->    HC165-SL/*PL
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

//Pin mapping GPS
 #define GPS_PPS 3
 #define GPS_3DFIX 4
 #define GPS_RESET 5
 //#define GPS_TX 0 // greyed out since initiated by Serial.begin() herein below
 //#define GPS_RX 1 // greyed out since initiated by Serial.begin() herein below

// Pin mapping mic pins
 #define EEPROM_RST 8 // Will reset EEPROM if grounded at startup

/************************** END - PIN-MAPPING ***************************************************************************************************************************************************/

/**************************** LoRaWAN PRE-SETUP ***********************************************************************************************************************/

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t checknsendjob;

/************************** END - LoRaWAN PRE-SETUP *******************************************************************************************************************/

/**************************** GPS PRE-SETUP ***********************************************************************************************************************/

TinyGPSPlus gps; // initialize GPS

typedef union // Needed to convert GPS float data to 4 bytes suitable for inclusion in LoRa-payload
{
 float number;
 uint8_t bytes[4];
} FLOATUNION_t;
FLOATUNION_t gpsFloat;

/************************** END - GPS PRE-SETUP *******************************************************************************************************************/

/**************************** INITIALIZATION OF VARIABLES ***********************************************************************************************************************/

// variable(s) to store up/downlinkdata and corresponding counters
uint8_t uplinkbytes[NO_UPLINKBYTES]; // array can hold up to NO_UPLINKBYTES bytes of payload
uint8_t downlinkbytes[NO_DOWNLINKBYTES]; // array can hold up to NO_DOWNLINKBYTES bytes of payload
uint16_t uplinkcounter; // must exceed the corresponding counter at TTN for the corresponding LoRa-message to be accepted
uint16_t LS_uplinkcounter; // last saved (to EEPROM) value of uplinkcounter

// GPS-related variables
volatile uint8_t Seconds = 0; // seconds counter fed by GPS via ISR
unsigned long gps_last_running_ms; // point in time of last stored GPS-data 

//mic variables
uint16_t n; // general loop-counter
uint8_t LowPwrMode = 1; // when set, will set LoRa and GPS in low power modes when not in use (i.e. in the inactive period of the TX_INTERVAL_LONG) 

/************************** END - INITIALIZATION OF VARIABLES *******************************************************************************************************************/
 
/******************************************************************************************************************************************************************************************************************************************************/
/****************************** SETUP BELOW ***********************************************************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************************************************************************************************/
void setup() {

  // setting various pins to a suitable initial state
  pinMode(GPS_PPS, INPUT); //floating input
  pinMode(GPS_3DFIX,INPUT_PULLUP); //pulled-up input
  pinMode(GPS_RESET,INPUT_PULLUP); //pulled-up input
  pinMode(EEPROM_RST,INPUT_PULLUP); //pulled-up input to avoid reset of EEPROM during startup, unless connected to ground

  // TODO: optional code here to feed power to the GPS- and LoRa-units (so that these are started up)

  // sanitary delay
  delay(1000);

  // reset EEPROM if the EEPROM_RST pin is connected to ground (by the user)
  if(!digitalRead(EEPROM_RST)){ reset_eeprom(); }

  // pick the last saved uplink countervalue from/of EEPROM
  EEPROM.get(0, LS_uplinkcounter);
  uplinkcounter = LS_uplinkcounter;
  // TODO: Check if a corresponding process is needed for a "downlinkcounter", i.e. if it is needed to analogously keep track of the downlink-counter

  // LoRaWAN (LMIC): initialization, reset and setting of margin of the time slot to catch dowlink messages 
  os_init();
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * CLOCK_ERROR_MARGIN / 100);  

  // Set static session parameters. Seemingly needed for code to run properly! 
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

  // LoRaWAN (LMIC) setting of channels to use
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

  // LoRaWAN (LMIC): furter and final settings and start of the same
  LMIC_setLinkCheckMode(0); // Disable link check validation
  LMIC.dn2Dr = DR_SF9; // TTN uses SF9 for its RX2 window.
  //TODO: Investigate further on how the TX-power may be regulated, to enable "full-power-out" if such should be found to be needed (e.g. for long range communication)  
  LMIC_setDrTxpow(DR_SF7,14); // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC.seqnoUp = uplinkcounter; // the uplinkcounter variable has just been assigned the present EEPROM stored value + 100 (to make sure that the next LoRa-msg will be accepted by TTN)  
  do_checknsend(&checknsendjob); // Start job

  // initialize serial for communication with the GPS
  Serial.begin(9600); 

  // begin counting seconds on basis of the PPS-signal from the GPS (assuming that this signal begins when GPS has a lock. TODO: check/confirm this)
  attachInterrupt(digitalPinToInterrupt(GPS_PPS), SecsCounter, RISING); 
  
}

/******************************************************************************************************************************************************************************************************************************************************/
/****************************** LOOP BELOW ************************************************************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************************************************************************************************/
void loop() {
  
  os_runloop_once();

  // TODO: code here to determine conditions and set low power- or default mode (GPS- and LoRa-units always on))

  // TODO: code here to reboot e.g. once per day, e.g. by checking millis()
    
}

/******************************************************************************************************************************************************************************************************************************************************/
/****************************** SUB-ROUTINES BELOW ****************************************************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************************************************************************************************/
void onEvent (ev_t ev) {  
    switch(ev) {
      case EV_TXCOMPLETE:
        // check if there was a downlinkmessage for this node
        if(LMIC.dataLen) {
              // store downlink payload to array of bytes and take appropriate actions depending on the specific downlinkmessage            
              for (n = 0; n < LMIC.dataLen; n++) { 
                downlinkbytes[n] = LMIC.frame[LMIC.dataBeg + n]; // store each byte of the downlinkmessage in a separate array
                analyze_and_act(downlinkbytes[n], n); // analyze each and one of the downlinkbytes for thereof dependant actions
              }             
        } 
        // Schedule next transmission
        if (LowPwrMode){
          os_setTimedCallback(&checknsendjob, os_getTime()+sec2osticks(TX_INTERVAL_LONG), do_checknsend);
          // TODO: code here to put the LoRa-transiever in standby, sleep or similar (perhaps even cutting power to it?!)
        }
        else {
          os_setTimedCallback(&checknsendjob, os_getTime()+sec2osticks(TX_INTERVAL_SHORT), do_checknsend);
        }
        
      break;
    } 
} 

// extracting GPS-data, sending it of by LoRa and handling the uplink-counter
void do_checknsend(osjob_t* j){
    // increment uplinkcounter and make permanent storage in EEPROM of/when next higher 100d
    uplinkcounter++;
    if(uplinkcounter >= LS_uplinkcounter){
      LS_uplinkcounter = LS_uplinkcounter + 100;
      EEPROM.put(0, LS_uplinkcounter);
    }
    // 
    // start up the GPS- and LoRa-units, if currently running in low power mode
    if (LowPwrMode){
      // TODO: code here to start up the GPS by providing power to it
      // TODO: code here to put the LoRa-transiever in normal mode (perhaps by providing power to it)
    }
    // TODO: code here to wait a while for the GPS to collect data, e.g. by collecting a few PPS-seconds
    // get GPS data from the GPS-unit/chip and populate the uplinkbytes
    get_GPScrds_and_speed(GPS_MAX_INTERNAL_AGE);
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        //TODO: code here to make the user aware of the fact that sending is pending / not currently proceeding  
    } // send only if GPS-data is acceptably fresh
    else if ( (millis() - gps_last_running_ms) <  GPS_MAX_TOTAL_AGE*1000 ) {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, uplinkbytes, sizeof(uplinkbytes), 0);
        // testcode that will restet the 10th next uplinkbyte, should it have been populated with a previous downlink-value, for debug purposes. TODO: remove when debugging completed
        uplinkbytes[9] = 0;
    }
    if (LowPwrMode){
      // TODO: code here to turn off the GPS by choking power to it (the LoRa-unit is put to sleep right after transmission in the "onEvent"-function)
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

/* SerialEvent occurs whenever new data comes in the hardware serial RX (from the GPS). This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available. */
void serialEvent() 
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

// get, store and time-tag valid GPS-data
uint8_t get_GPScrds_and_speed(uint8_t gps_max_internal_age) { 
     if (gps.location.age() < (gps_max_internal_age * 1000) ) // extract and show the latest confirmed location if updated within the last gps_max_internal_age seconds
     {
       gpsFloat.number = gps.location.lng(); // Assign LONGITUDE to the bufferfloat
        for (n=0; n<4; n++)
        {
         uplinkbytes[n] = gpsFloat.bytes[n]; // populate the first 4 bytes with a representation of the (float) LONGITUDE-data 
        }
       gpsFloat.number = gps.location.lat(); // Assign LATITUDE to the bufferfloat
        for (n=0; n<4; n++)
        {
         uplinkbytes[n+4] = gpsFloat.bytes[n]; // populate the following 4 bytes ([4] to [7]) with a representation of the (float) LATITUDE-data 
        }
      uplinkbytes[8] = (uint8_t) gps.speed.kmph();
      gps_last_running_ms = millis() - (gps_max_internal_age * 1000); // keep track of point in time of the last valid stored GPS data
      return 1; 
     }
     else {
      return 0;
     }
 }

 // resets the EEPROM static memory
void reset_eeprom(void){
  for (n = 0 ; n < EEPROM.length() ; n++) { EEPROM.write(n, 0); }
}

// analysis of a particular byte the downlink-msg and initiation of actions as dependant on the data of the same
void analyze_and_act(uint8_t data, uint8_t byteno){
  switch(byteno) {
      case 0:
       // testcode that will populate the 10th next uplinkbyte with the value of the last first downlinkbyte, for debug purposes. TODO: remove when debugging completed
       uplinkbytes[9] = data;
       switch(data) {
        case 0:
        // TODO: code here to take care of dataload == 0 in the first byte (byteno == 0)
        break;
        case 1:
        // TODO: code here to take care of dataload == 1 in the first byte (byteno == 0)
        break;
        /*
         .
         .
         .
        */
        case 255:
        // reset local counter (eeprom memory to 0) and reboot when the message-counter has been/will be reset to 0 on the TTN-side, as indicated by downlink-msg == 255
        reset_eeprom();
        // TODO: code here to reboot
        break;
       }
      break;
     case 1:
      // TODO: code here to take care of the data of the second byte (byteno == 1)
     break;
  }
}










