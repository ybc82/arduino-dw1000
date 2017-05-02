/**
 * 
 * @todo
 *  - move strings to flash (less RAM consumption)
 *  - fix deprecated convertation form string to char* startAsAnchor
 *  - give example description
 */

#include <SPI.h>
#include "DW1000Ranging.h"

#define NODE_ANCHOR3

#if defined(NODE_ANCHOR1)
  #define SHORT_ADDRESS {0x00, 0x01}
  #define PIN_NUM_IRQ 3
  #define UNIQUE_ID "82:17:5B:D5:A9:9A:E2:9E"
#elif defined(NODE_ANCHOR2)
  #define SHORT_ADDRESS {0x00, 0x02}
  #define PIN_NUM_IRQ 2
  #define UNIQUE_ID "82:17:5B:D5:A9:9A:E2:9F"
#elif defined(NODE_ANCHOR3)
  #define SHORT_ADDRESS {0x00, 0x03}
  #define PIN_NUM_IRQ 2
  #define UNIQUE_ID "82:17:5B:D5:A9:9A:E2:9D"
#elif defined(NODE_TAG1)
  #define SHORT_ADDRESS {0x01, 0x01}
#endif // NODE

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = PIN_NUM_IRQ; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

char short_address[] = SHORT_ADDRESS;

// const byte tag_address[] = {0x9C, 0x3B, 0x60, 0x82, 0xEA, 0x22, 0x00, 0x7D};
byte tag_address[] = {0x7D, 0x00, 0x22, 0xEA, 0x82, 0x60, 0x3B, 0x9C};
byte tag_short_address[] = {0x01, 0x01};

void setup() {
  Serial.begin(115200);
  delay(1000);

  //init the configuration
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  //define the sketch as anchor. It will be great to dynamically change the type of module
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachBlinkDevice(newBlink);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  //Enable the filter to smooth the distance
  //DW1000Ranging.useRangeFilter(true);
  
  //we start the module as an anchor
  DW1000Ranging.startAsAnchor(UNIQUE_ID, short_address, DW1000.MODE_LONGDATA_RANGE_ACCURACY);
// DW1000Ranging.startAsAnchor(UNIQUE_ID, DW1000.MODE_LONGDATA_RANGE_ACCURACY);

  // add tag information
  DW1000Device myTag(tag_short_address, true);
  DW1000Ranging.addNetworkDevices(&myTag);
//  DW1000Ranging.setExpectedMsgId(0); //POLL // not necessary, can be solved by checkforreset
}

void loop() {
  DW1000Ranging.loop();
}

void newRange() {
  Serial.print("from: "); Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  Serial.print("\t Range: "); Serial.print(DW1000Ranging.getDistantDevice()->getRange()); Serial.print(" m");
  Serial.print("\t RX power: "); Serial.print(DW1000Ranging.getDistantDevice()->getRXPower()); Serial.println(" dBm");
  
}

void newBlink(DW1000Device* device) {
  Serial.print("blink; 1 device added ! -> ");
  Serial.print(" short:");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device* device) {
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}

