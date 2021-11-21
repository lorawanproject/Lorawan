#include <lmic.h>
#include <dht.h>
#include <hal/hal.h>
#include <SPI.h>
dht DHT;
#define DHT11_PIN 5
#define PIN_A A0


float temperature,humidity;      
float tem,hum;

unsigned int count = 1;        //For times count

String datastring1="";        
String datastring2="";        
String datastring3="";


static uint8_t mydata[11] = {0x01,0x67,0x00,0x00,0x02,0x68,0x00,0x03,0x65,0x00,0x00};
/* LoRaWAN NwkSKey, network session key
   This is the default Semtech key, which is used by the prototype TTN
   network initially.
   ttn*/
static const PROGMEM u1_t NWKSKEY[16] = { 0x88, 0xD1, 0xFC, 0x1C, 0xB9, 0x24, 0xB4, 0x9F, 0xF0, 0xA3, 0xDF, 0x21, 0x9A, 0x67, 0x80, 0x83 };

/* LoRaWAN AppSKey, application session key
   This is the default Semtech key, which is used by the prototype TTN
   network initially.
   ttn*/
static const u1_t PROGMEM APPSKEY[16] = { 0x86, 0x43, 0x7D, 0xB1, 0x31, 0x6E, 0x18, 0xC5, 0xBC, 0x21, 0xAF, 0xCE, 0xC0, 0xD9, 0x61, 0xF8 };

/*
 LoRaWAN end-device address (DevAddr)
 See http://thethingsnetwork.org/wiki/AddressSpace
 ttn*/
static const u4_t DEVADDR = 0x260AC489;


/* These callbacks are only used in over-the-air activation, so they are
  left empty here (we cannot leave them out completely unless
   DISABLE_JOIN is set in config.h, otherwise the linker will complain).*/
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }


static osjob_t initjob,sendjob,blinkjob;

/* Schedule TX every this many seconds (might become longer due to duty
 cycle limitations).*/
const unsigned TX_INTERVAL = 20;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println("OP_TXRXPEND, not sending");
    } else {
        
        dhtTem();
        light();
        // Prepare upstream data transmission at the next possible time.
        //  LMIC_setTxData2(1,datasend,sizeof(datasend)-1,0);
        LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
        Serial.println("Packet queued");
        Serial.print("LMIC.freq:");
        Serial.println(LMIC.freq);
        Serial.println("Receive data:");
      
        
    } 
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    Serial.println(ev);
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
            if(LMIC.dataLen) {
                // data received in rx slot after tx
                Serial.print(F("Data Received: "));
                Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
                Serial.println();
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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

void setup() {
     // initialize digital pin  as an output.
   
    Serial.begin(9600);
    while(!Serial);
    Serial.println("Connect to TTN and Send data to mydevice(Use DHT11 Sensor):");
   
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
    /*LMIC_setClockError(MAX_CLOCK_ERROR * 1/100);
     Set static session parameters. Instead of dynamically establishing a session
     by joining the network, precomputed session parameters are be provided.*/
    #ifdef PROGMEM
    /* On AVR, these values are stored in flash and only copied to RAM
       once. Copy them to a temporary buffer here, LMIC_setSession will
       copy them into a buffer of its own again.*/
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

   
    
    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}
void dhtTem()
{
       int16_t tem1;
       temperature = DHT.read11(DHT11_PIN);    //Temperature detection
       tem = DHT.temperature*1.0;      
      float humidity = DHT.read11(DHT11_PIN);
      float hum = DHT.humidity* 1.0;

      Serial.print(F("###########    "));
       Serial.print(F("NO."));
       Serial.print(count);
       Serial.println(F("    ###########"));
       Serial.println(F("The temperautre and humidity :"));
       Serial.print(F("["));
       Serial.print(tem);
       Serial.print(F("℃"));
       Serial.print(F(","));
       Serial.print(hum);
       Serial.print(F("%"));
       Serial.print(F("]"));
       Serial.println("");
       count++;
       tem1=(tem*10);
       mydata[2] = tem1>>8;
       mydata[3]= tem1;
       mydata[6] = hum * 2;
       
}

void light(){
      int16_t lux;
      int val,val1;
      val=analogRead(PIN_A);
     // Serial.print(F("a:"));
      //Serial.println(val);
      delay(500);
      val1=val*1.0;
      lux=val1;
      mydata[9]=lux>>8;
      mydata[10]=lux;
       //Serial.print(lux);
}
void loop() {
    os_runloop_once();
       
}
