#include "PinChangeInterrupt.h"
#include "LowPower.h"
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <EEPROM.h>

#define RFM_NSS 9
#define RFM_RST 8
#define RFM_DIO_0 2
#define RFM_DIO_1 3
#define RFM_DIO_2 4

// Never put that pin to Input!
#define PSU_SWITCH 10

#define BTN_PCB A0

#define LED0 7 //PD7
#define SWITCH0 6 //PD6
#define LED1 5 //PD5
#define SWITCH1 A1 //PC1
#define LED2 A2 //PC2
#define SWITCH2 A3 //PC3
#define LED3 A4 //PC4
#define SWITCH3 A5 //PC5

typedef struct {
  uint8_t snr;
  uint8_t version;
  uint8_t appeui[8];
  uint8_t deveui[8];
  uint8_t appkey[16];
} taz_configuration_t;

taz_configuration_t myconfig;

void os_getArtEui (u1_t* buf) { memcpy(buf, myconfig.appeui, 8);}
void os_getDevEui (u1_t* buf) { memcpy(buf, myconfig.deveui, 8);}
void os_getDevKey (u1_t* buf) {  memcpy(buf, myconfig.appkey, 16);}

uint8_t switch0_counter = 0;
uint8_t switch1_counter = 0;
uint8_t switch2_counter = 0;
uint8_t switch3_counter = 0;

// Get's reseted to 0x00 after first lorawan transmit
uint8_t device_status = 0x01;

static uint8_t mypayload[6];
static osjob_t sendjob;

const lmic_pinmap lmic_pins = {
    .nss = RFM_NSS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = RFM_RST,
    .dio = {RFM_DIO_0, RFM_DIO_1, RFM_DIO_2},
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

            for(int i = 0; i <= 100; i++) {
              digitalWrite(LED0, 1);  digitalWrite(LED1, 1);  digitalWrite(LED2, 1);  digitalWrite(LED3, 1);  delayMicroseconds(250000);
              digitalWrite(LED0, 0);  digitalWrite(LED1, 0);  digitalWrite(LED2, 0);  digitalWrite(LED3, 0);  delayMicroseconds(250000); 
            }

            attachPCINT(digitalPinToPCINT(SWITCH0), btnint, FALLING);
            attachPCINT(digitalPinToPCINT(SWITCH1), btnint, FALLING);
            attachPCINT(digitalPinToPCINT(SWITCH2), btnint, FALLING);
            attachPCINT(digitalPinToPCINT(SWITCH3), btnint, FALLING);
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

            LMIC_shutdown();

            Serial.println("Going to sleep now\n");
            
            Serial.end();
            // SPI.end() doesn't work reliable, have I already noted, I hate Arduino!
            SPCR &= ~_BV(SPE);
            pinMode(RFM_NSS, INPUT);
            pinMode(RFM_RST, INPUT);
            digitalWrite(PSU_SWITCH, 1);

            // Happy sleeping for ~15min
            for(int i = 0; i <= 112; i++) {
              LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
            }
            
            digitalWrite(PSU_SWITCH, 0);
            Serial.begin(9600);
            Serial.println("Finished sleeping\n");
            pinMode(RFM_NSS, OUTPUT);
            //pinMode(PSU_SWITCH, OUTPUT);            
            SPI.begin();

            os_init();
            LMIC.opmode       =  OP_NONE;
            
            do_send(&sendjob);
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Do a Arduino conform meassurement of the Voltage, Arduino doesn't work, sorry!
        analogReference(INTERNAL);
        delay(25);
        int meassurement = analogRead(7);
        delay(25);
        meassurement = analogRead(7);

        
        mypayload[0] = device_status << 2;
        mypayload[0] |= (meassurement & 0x300) >> 8;
        mypayload[1] = meassurement & 0xFF;

        mypayload[2] = switch0_counter;
        mypayload[3] = switch1_counter;
        mypayload[4] = switch2_counter;
        mypayload[5] = switch3_counter;

        device_status = 0;
        
        // LoRaWAN Port defines protocoll/SW Version
        LMIC_setTxData2(1, mypayload, sizeof(mypayload), 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void btnint() {
  uint8_t switch0_debouncer = 0, switch1_debouncer = 0, switch2_debouncer = 0, switch3_debouncer = 0;
  
  // Do some sort of button debouncing
  for(int i = 0; i <= 100; i++) {
    if(!digitalRead(SWITCH0)) {
      switch0_debouncer++;
    }
    if(!digitalRead(SWITCH1)) {
      switch1_debouncer++;
    }
    if(!digitalRead(SWITCH2)) {
      switch2_debouncer++;
    }
    if(!digitalRead(SWITCH3)) {
      switch3_debouncer++;
    }
    delayMicroseconds(10);
  }
  
  if(switch0_debouncer >= 100) {
    switch0_counter++;
    
    for(int i = 0; i <= 10; i++) {
      digitalWrite(LED0, 1);  delayMicroseconds(250000);
      digitalWrite(LED0, 0);  delayMicroseconds(250000); 
    }
  }
  if(switch1_debouncer >= 100) {
    switch1_counter++;
    
    for(int i = 0; i <= 10; i++) {
      digitalWrite(LED1, 1);  delayMicroseconds(250000);
      digitalWrite(LED1, 0);  delayMicroseconds(250000); 
    }
  }
  if(switch2_debouncer >= 100) {
    switch2_counter++;
    
    for(int i = 0; i <= 10; i++) {
      digitalWrite(LED2, 1);  delayMicroseconds(250000);
      digitalWrite(LED2, 0);  delayMicroseconds(250000); 
    }
  }
  if(switch3_debouncer >= 100) {
    switch3_counter++;
    
    for(int i = 0; i <= 10; i++) {
      digitalWrite(LED3, 1);  delayMicroseconds(250000);
      digitalWrite(LED3, 0);  delayMicroseconds(250000); 
    }
  }
}

void lmic_setup() {
  LMIC_setAdrMode(1);
  
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  
  LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
}

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, INPUT);

  pinMode(PSU_SWITCH, OUTPUT);
  digitalWrite(PSU_SWITCH, 0);

  pinMode(BTN_PCB, INPUT);

  pinMode(SWITCH0, INPUT);
  pinMode(SWITCH1, INPUT);  
  pinMode(SWITCH2, INPUT);  
  pinMode(SWITCH3, INPUT);

  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  Serial.print("Hello\n");

  EEPROM.get(0, myconfig);

  char mybuffer[100];
  sprintf(mybuffer, "SNR: %d\nVersion: %d\nAppeui: %X%X%X%X%X%X%X%X\nDeveui: %X%X%X%X%X%X%X%X\nAppkey: %X%X%X%X%X%X%X%X%X%X%X%X%X%X%X%X", 
              myconfig.snr, myconfig.version,
              myconfig.appeui[0], myconfig.appeui[1], myconfig.appeui[2], myconfig.appeui[3], myconfig.appeui[4], myconfig.appeui[5], myconfig.appeui[6], myconfig.appeui[7],
              myconfig.deveui[0], myconfig.deveui[1], myconfig.deveui[2], myconfig.deveui[3], myconfig.deveui[4], myconfig.deveui[5], myconfig.deveui[6], myconfig.deveui[7],
              myconfig.appkey[0], myconfig.appkey[1], myconfig.appkey[2], myconfig.appkey[3], myconfig.appkey[4], myconfig.appkey[5], myconfig.appkey[6], myconfig.appkey[7], myconfig.appkey[8], myconfig.appkey[9], myconfig.appkey[10], myconfig.appkey[11], myconfig.appkey[12], myconfig.appkey[13], myconfig.appkey[14], myconfig.appkey[15]
              );

  Serial.println(mybuffer);

  os_init();
  LMIC_reset();

  LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
  
  lmic_setup();

  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}
