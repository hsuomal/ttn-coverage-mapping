/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman,
 * Copyright (c) 2019 Harri Suomalainen
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This gets location by GPS and sends a lorawan package containing
 * position info.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which limits
 * transmitting time to 30s in 24 hour period).
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * Do not forget to define the radio type correctly in config.h in LMIC library.
 * NeoGPS also need to be configured to give enough messages and combine results.
 *
 * Libraries used:
 * name=LMIC-Arduino
 * version=1.5.0+arduino-2
 * url=https://github.com/matthijskooijman/arduino-lmic
 * 
 * name=NeoGPS
 * version=4.2.9
 * url=https://github.com/SlashDevin/NeoGPS
 *
 * This sketch work with TTGO T-Beam (both version with neo-6 or neo-8 series gps)
 *
 * Issues: When Neo-N8M is configured for multiple GNSS (like GPS+Glonass+Galileo)
 * satellite info comes in multiple messages per GNSS. ($GN...$GPV... $GLV... $GAV...).
 * NeoGPS combines satellite info but sometimes some message is missed and library
 * still thinks info is complete. In that case satellite count is low.
 * Sometimes we have perfect fix value but incomplete sat info, in that case we send 0 sats.
 * Also GPS talker (last GNSS that send info) can be incomplete, should be GN for 
 * combined info, sometimes shows GP (gps), GL (Glonass) or GA (Galileo)
 * Satellite count seems high and may be unrealibale, make it zero if you want to remove it.
 * Currently ttn mapper prefers HDOP over sat count anyway.
 *
 * Sample Decoder script included in comments of code.
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include "batvoltage.h"
#include "globals.h"
#include "gps.h"



///////////////////////////////////////////////////////// prototype code for fute mods
/*
const char *devAddr = "26011171";
const char *nwkSKey = "B768BC5637E6D602B708D3C6BE32C6D0";
const char *appSKey = "DB1260B7B0F4C4BC3C3DA5F54DA7956C";

// Convert hex string to raw values in buffer
// Beware: buffer sizes not checked, make sure instring is null-terminated and outbuffer has enough room
void hex2buffer(char *instring, u1_t *outbuffer) {
  unsigned i=0; // input location
  unsigned k=0; // output location
  u1_t val;
  for(i=0; i<strlen(instring); i+=2) {
    val = ((instring[i]-'0') << 4) | instring[i+1]; // decode hex chars
    outbuffer[k] = val;
  }
}

//u4_t myDEVADDR = (devAddr[0]-'0') << 28 | (devAddr[1]-'0')<<24 | (devAddr[2]-'0')<<20 |(devAddr[3]-'0')<<16 |(devAddr[4]-'0')<<12 | \
//(devAddr[5]-'0')<<8 | (devAddr[6]-'0')<<4 |(devAddr[7]-'0')
*/

//////////////////////////////////////////////////////////
#define DEVICE 1

//---------------------
#if DEVICE==1
// MSB first
static const PROGMEM u1_t NWKSKEY[16] = { 0xA7, 0xC5, 0xBC, 0x20, 0x9D, 0xB7, 0x95, 0xD4, 0xE1, 0xC1, 0x57, 0xFC, 0x9D, 0xD2, 0xB2, 0x26 };
// MSB first
static const u1_t PROGMEM APPSKEY[16] = { 0x99, 0x49, 0xC6, 0x6D, 0x1E, 0x6A, 0x20, 0x01, 0x67, 0xEA, 0x09, 0xD0, 0x9E, 0x1F, 0xF4, 0x53 };
// MSB first
static const u4_t DEVADDR = 0x26011FD9 ; // <-- Change this address for every node!
uint TX_INTERVAL=20; // 0,5%
uint8_t Fport=100; // chosen port 100 for tracker

//-------------------ls 
#elif DEVICE==2
// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN network. MSB first byte order
static const PROGMEM u1_t NWKSKEY[16] = { 0xB7, 0x68, 0xBC, 0x56, 0x37, 0xE6, 0xD6, 0x02, 0xB7, 0x08, 0xD3, 0xC6, 0xBE, 0x32, 0xC6, 0xD0 };
// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN network. MSB first byte order
static const u1_t PROGMEM APPSKEY[16] = { 0xDB, 0x12, 0x60, 0xB7, 0xB0, 0xF4, 0xC4, 0xBC, 0x3C, 0x3D, 0xA5, 0xF5, 0x4D, 0xA7, 0x95, 0x6C };
// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x26011171 ; // <-- Change this address for every node!
uint TX_INTERVAL=180;                    // 30s/day transmission time with sf7, just within ttn fair use policy.
uint8_t Fport=1;                         // Port 1 for testing, Port 100 for forwarding. Port must match decoder script on ttn decoder.
                                         // can use different port for debugging and forward only some port via ttn-mapper integration

#endif



// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;


// Pin mapping
const lmic_pinmap lmic_pins = {
.nss = 18,
.rxtx = LMIC_UNUSED_PIN,
.rst = LMIC_UNUSED_PIN, // ttgo real pin 23 but it seems to work better without it (hangs if defined)
.dio = {26, 33, LMIC_UNUSED_PIN}, // ttgo t-beam pcb has table of 26, 33, 32
};


// Schedule next data transmission based on interval time in seconds
void scheduleTransmission(uint8_t seconds) {
    // Schedule next transmission
    os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(seconds), do_send);
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            scheduleTransmission(TX_INTERVAL);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}



static osjob_t gpsjob;
void do_pollGPS(osjob_t* j){
  pollGpsOutput();  // read any gps data and process it

  // Schedule next gps polling time. If time too high, this starves cpu, if time too low, lots of packages are lost
  os_setTimedCallback(&gpsjob, os_getTime()+ms2osticks(213), do_pollGPS); // every 213ms to avoid syncing to full seconds
}


/*****************************************************
 *   TTN Payload format decoder script for custom format below
 *   
function Decoder(bytes){
  //function Decoder(bytes, Fport){
  // Fport can be varied for different apps or different data
  var lat = bytes[0]<<16 | bytes[1]<<8 | bytes[2];
  var lon = bytes[3]<<16 | bytes[4]<<8 | bytes[5];
  var alt = bytes[6]<<8 | bytes[7];
  var hdop = bytes[8] /10;
  var nsat = bytes[9];

  lat = lat / 16777215 * 180 -90;
  lon = lon / 16777215 * 360-180;

  if(nsat===0)  // to allow packets without sat count when not available at the time
    return {
      latitude: lat,
      longitude: lon,
      altitude: alt,
      hdop: hdop,
   }
  else return {
      latitude: lat,
      longitude: lon,
      altitude: alt,
      hdop: hdop,
      satellites: nsat
   }
}
*******************************************************/
        
        
//        ///////////////////// code prototype for future mods
//        Perhaps something like this would be more clear code?
//
//        struct {
//          uint8_t  latitude[3];
//          uint8_t  longitude[3];
//          uint16_t altitude;
//          uint8_t  hdop;
//          uint8_t  satellites;
//        } my_tx_buff;
//
//        my_tx_buff.satellites = ...
//        my_tx_buff.hdop=...
//        LMIC_setTxData2(1, (void *)tx_buff, sizeof(tx_buff));


// Sender job. This checks we have valid data and if so, sends it via radio
//
void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running. This can happen only if sending very often.
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } 
    else

      if(fix.valid.location) {  // We need a valid fix

        while(!fix.valid.location  || !gps.satellites_valid() || gps.sat_count<4)  // if everything is not in sync, poll gps untill they are
          pollGpsOutput();

        
        printGpsData();         // print at serial for debugging
        printBatteryVoltage();  // print battery level too (and shut down if batt low)


        // Prepare data for sending
        // Based on code at https://github.com/ttnmapper/gps-node-examples/blob/master/Sodaq/sodaq-one-ttnmapper/sodaq-one.ino
        //
        // Converts data to a tight binary packet. Smaller packet = less transmission time
        //
        uint8_t txData[10];
        uint32_t LatitudeBinary, LongitudeBinary;
        uint16_t altitudeGps;
        uint8_t hdopGps;
        uint8_t satellitesGps;

        LatitudeBinary = ((fix.latitude() + 90) / 180) * 16777215.0;
        LongitudeBinary = ((fix.longitude() + 180) / 360) * 16777215.0;
        altitudeGps = (uint16_t)fix.altitude(); // altitude in meters
        hdopGps = fix.hdop/100>255 ? 255 : fix.hdop/100;              // hdop*10, max 255 (gix.hdop returns hdop*1000)
        satellitesGps = gps.sat_count;

        if(!gps.satellites_valid()) satellitesGps=0;  // send zero if not valid count

        txData[0] = ( LatitudeBinary >> 16 ) & 0xFF;  // Store as 24 bit value
        txData[1] = ( LatitudeBinary >> 8 ) & 0xFF;
        txData[2] = LatitudeBinary & 0xFF;

        txData[3] = ( LongitudeBinary >> 16 ) & 0xFF; // Store as 24 bit value
        txData[4] = ( LongitudeBinary >> 8 ) & 0xFF;
        txData[5] = LongitudeBinary & 0xFF;

        txData[6] = ( altitudeGps >> 8 ) & 0xFF;  // 16 bits
        txData[7] = altitudeGps & 0xFF;
        txData[8] = hdopGps;  // hdop*10
        txData[9] = satellitesGps;  // 8bits


        pinMode(14,OUTPUT);
        digitalWrite(14,HIGH);  // LED ON as an indicator (no led in the new model though, works only on old model)
        
        // Print payload in hex
        Serial.print("Payload: ");
        for (uint i=0; i<sizeof(txData); i++) {
          if(txData[i]<0x10) Serial.print("0"); // leading zero
          Serial.print(txData[i],HEX);
        }
        Serial.println("");
        delay(100);
        digitalWrite(14,LOW);

        // Prepare upstream data transmission at the next possible time. LMIC_setTxData(port, buffer, lenght, confirmed)
        // FPort 0 recerved for mac, can use FPorts 1...199 for different applications, Fport>=200 sometimes in use/recerved
        LMIC_setTxData2(Fport, txData, sizeof(txData), 0);
        Serial.println(F("Packet queued"));
        // When transmitting, next TX is scheduled after TX_COMPLETE event.
      }
      else {
        // There was no fix and no transmission, schedule transmission here
        Serial.print( fix.valid.location ? "Have fix" : "No fix");
        Serial.print( gps.satellites_valid()? ", Sat valid" : ", No valid sat");
        Serial.print(", num_sat="+String(gps.sat_count));
        Serial.print(", Batt ");
        Serial.println(getBatteryVoltage(),3); // poll battery voltage in case we need to shut down with empty battery

        scheduleTransmission(1);  // schedule next session sooner than normally (TX_INTERVAL/8) or even the next second
      }
      
} // void do_send()



void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting"));

    setupGpsPort();
    
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    //Make room for 5% clock error
    LMIC_setClockError(MAX_CLOCK_ERROR * 5 / 100);

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
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

    // enable and configure channels used. If none given (or less than 3), library defaults to channels 0,1,2
    
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
/*    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band 1%
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band 0.1%
*/
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

    // TTN uses SF9 for its RX2 window on EU868 band. This may be SF12 in US915
    LMIC.dn2Dr = DR_SF9;

    LMIC_disableChannel(0); // You can disable channels if you want to measure only some channel(s)

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    // TTN Mapper is based on worst case ie. SF7
    LMIC_setDrTxpow(DR_SF7,14);
    
    // Start jobs
    do_pollGPS(&gpsjob);
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
