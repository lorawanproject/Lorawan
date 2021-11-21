

#include <TinyGPS.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <SoftwareSerial.h>

TinyGPS gps;
SoftwareSerial ss(3, 4); // Arduino RX, TX to conenct

static void smartdelay(unsigned long ms);
unsigned int count = 0;        //For times count

float longitude,latitude;
float flat,flon,falt;

static uint8_t mydata[11] ={0x03,0x88,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0xBE, 0xC0, 0x77, 0xEF, 0xCC, 0x54, 0x14, 0x4A, 0x0D, 0xB1, 0xC2, 0xFD, 0x7F, 0xB5, 0x1C, 0xC7 };

static const u1_t PROGMEM APPSKEY[16] = { 0xC8, 0x5C, 0xA1, 0x0A, 0xB4, 0x6B, 0x56, 0x19, 0x87, 0x1B, 0x85, 0x90, 0x57, 0xC0, 0xB9, 0x97 };

static const u4_t DEVADDR = 0x260A3C7F;


// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leavåe them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
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
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

// World Geodetic System ==> Mars Geodetic System
double transformLat(double x, double y)  
{  
    double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * sqrt(abs(x));  
    ret += (20.0 * sin(6.0 * x * M_PI) + 20.0 * sin(2.0 * x * M_PI)) * 2.0 / 3.0;  
    ret += (20.0 * sin(y * M_PI) + 40.0 * sin(y / 3.0 * M_PI)) * 2.0 / 3.0;  
    ret += (160.0 * sin(y / 12.0 * M_PI) + 320 * sin(y * M_PI / 30.0)) * 2.0 / 3.0;  
    return ret;  
}  
  
 double transformLon(double x, double y)  
{  
    double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * sqrt(abs(x));  
    ret += (20.0 * sin(6.0 * x * M_PI) + 20.0 * sin(2.0 * x * M_PI)) * 2.0 / 3.0;  
    ret += (20.0 * sin(x * M_PI) + 40.0 * sin(x / 3.0 * M_PI)) * 2.0 / 3.0;  
    ret += (150.0 * sin(x / 12.0 * M_PI) + 300.0 * sin(x / 30.0 * M_PI)) * 2.0 / 3.0;  
    return ret;  
}  
    
void WGS2GCJTransform(float wgLon, float wgLat, float &mgLon, float &mgLat)  
{  
    const double a = 6378245.0;  
    const double ee = 0.00669342162296594323;  
  
    double dLat = transformLat(wgLon - 105.0, wgLat - 35.0);  
    double dLon = transformLon(wgLon - 105.0, wgLat - 35.0);  
  
    double radLat = wgLat / 180.0 * M_PI;  
    double magic = sin(radLat);  
    magic = 1 - ee * magic * magic;  
  
    double sqrtMagic = sqrt(magic);  
    dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * M_PI);  
    dLon = (dLon * 180.0) / (a / sqrtMagic * cos(radLat) * M_PI);  
  
    mgLat = wgLat + dLat;  
    mgLon = wgLon + dLon;  
}

void GPSRead()
{
  unsigned long age;
  gps.f_get_position(&flat, &flon, &age);
  falt=gps.f_altitude();  //get altitude     
  flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6;//save six decimal places 
  flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6;
  falt == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : falt, 2;//save two decimal places
  if((flon < 72.004 || flon > 137.8347)&&(flat < 0.8293 || flat >55.8271))  //out of China
  {
    longitude=flon;
    latitude=flat;
  // Serial.println("Out of China");
  }
  else
  {
    WGS2GCJTransform(flon,flat,longitude,latitude);
//   //Serial.println("In China");
  }
  int32_t lat = latitude * 10000;
  int32_t lon = longitude * 10000;
  int32_t alt = falt * 100;

  mydata[2] = lat >> 16;
  mydata[3] = lat >> 8;
  mydata[4] = lat;
  mydata[5] = lon >> 16;
  mydata[6] = lon >> 8;
  mydata[7] = lon;
  mydata[8] = alt >> 16;
  mydata[9] = alt >> 8;
  mydata[10] = alt;  
}

void printdata(){
       Serial.print(F("###########    "));
       Serial.print(F("NO."));
       Serial.print(count);
       Serial.println(F("    ###########"));
       if(flon!=1000.000000)  //Successfully positioning
  {  
       Serial.println(F("The longtitude and latitude and altitude are:"));
       Serial.print(F("["));
       Serial.print(longitude,4);
       Serial.print(F(","));
       Serial.print(latitude,4);
       Serial.print(F(","));
      Serial.print(falt);
       Serial.print(F("]"));
     Serial.println(F(""));
       count++;
  }
  else
   {
   Serial.println(F("Fail positioning"));
   }
  }

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
    {
      gps.encode(ss.read());
    }
  } while (millis() - start < ms);
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        smartdelay(1000);
        GPSRead();
        printdata();
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(9600);
    Serial.println(F("Starting"));
    ss.begin(9600);
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

    for (int channel=0; channel<8; ++channel) {
    LMIC_disableChannel(channel);
  }
  for (int channel=16; channel<72; ++channel) {
     LMIC_disableChannel(channel);
     }


    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
