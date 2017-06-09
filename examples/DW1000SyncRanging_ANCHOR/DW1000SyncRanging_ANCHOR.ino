/**
 * 
 * @todo
 *  - move strings to flash (less RAM consumption)
 *  - fix deprecated convertation form string to char* startAsAnchor
 *  - give example description
 */

#include <SPI.h>
#include "DW1000SyncRanging.h"

#define NODE_ANCHOR4
#include "address.h"

// connection pins
const uint8_t PIN_RST = PIN_NUM_RST; // reset pin
const uint8_t PIN_IRQ = PIN_NUM_IRQ; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

char short_address[] = SHORT_ADDRESS;

// const byte tag_address[] = {0x9C, 0x3B, 0x60, 0x82, 0xEA, 0x22, 0x00, 0x7D};
byte base_address[] = BASE_ADDRESS;
byte base_short_address[] = BASE_SHORT_ADDRESS;

void setup() {
  Serial.begin(115200);
  delay(1000);

  //init the configuration
  DW1000SyncRanging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  //define the sketch as anchor. It will be great to dynamically change the type of module
  DW1000SyncRanging.attachNewRange(newRange);
  DW1000SyncRanging.attachBlinkDevice(newBlink);
  DW1000SyncRanging.attachInactiveDevice(inactiveDevice);
  //Enable the filter to smooth the distance
  //DW1000SyncRanging.useRangeFilter(true);
  
  //we start the module as an anchor
  DW1000SyncRanging.startAsAnchor(UNIQUE_ID, short_address, DW1000.MODE_LONGDATA_RANGE_ACCURACY);
// DW1000SyncRanging.startAsAnchor(UNIQUE_ID, DW1000.MODE_LONGDATA_RANGE_ACCURACY);

  // add neighbor information
  DW1000Device myBase(base_short_address, true);
  DW1000SyncRanging.addNetworkDevices(&myBase);
//  DW1000SyncRanging.setExpectedMsgId(0); //POLL // not necessary, can be solved by checkforreset
}

void loop() {
  DW1000SyncRanging.loop();
}

/// the following may be changed
void newRange() {
  Serial.print("from: "); Serial.print(DW1000SyncRanging.getDistantDevice()->getShortAddress(), HEX);
  Serial.print("\t Range: "); Serial.print(DW1000SyncRanging.getDistantDevice()->getRange()); Serial.print(" m");
  Serial.print("\t RX power: "); Serial.print(DW1000SyncRanging.getDistantDevice()->getRXPower()); Serial.println(" dBm");

  
  DW1000Device* myDistantDevice = DW1000SyncRanging.getDistantDevice();
  printLong(myDistantDevice->timePollSent.getTimestamp());
  printLong(myDistantDevice->timePollAckReceived.getTimestamp());
  printLong(myDistantDevice->timeRangeSent.getTimestamp());
  printLong(myDistantDevice->timePollReceived.getTimestamp());
  printLong(myDistantDevice->timePollAckSent.getTimestamp());
  printLong(myDistantDevice->timeRangeReceived.getTimestamp());

  Serial.println();
}

/* 
 *  printLong
 *  To print int64_t number in serial port
 */
void printLong(int64_t num_in)
{
  uint64_t num = (uint64_t)(num_in);
  char buf[18];
  for (int i = 0; i < 4; i++)
  {
    sprintf(buf, "%04X", (uint16_t)(0xFFFF & (num >> (16*(3-i)))));
    Serial.print(buf);
//    Serial.print((uint8_t)(0xFF & (num >> (8*(7-i)))), HEX);
  }
  Serial.print("\t");
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

