/**
 * 
 * @todo
 *  - move strings to flash (less RAM consumption)
 *  - fix deprecated convertation form string to char* startAsTag
 *  - give example description
 */
#include <SPI.h>
#include "DW1000SyncRanging.h"

#define NODE_TAG1
#include "address.h"

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

const char short_address[] = SHORT_ADDRESS;
byte anchor1_short_address[] = ANCHOR1_SHORT_ADDRESS;
byte anchor2_short_address[] = ANCHOR2_SHORT_ADDRESS;
byte anchor3_short_address[] = ANCHOR3_SHORT_ADDRESS;

void setup() {
  Serial.begin(115200);
  delay(1000);

  //init the configuration
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  //define the sketch as anchor. It will be great to dynamically change the type of module
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  //Enable the filter to smooth the distance
  //DW1000Ranging.useRangeFilter(true);
  
  //we start the module as a tag
  DW1000Ranging.startAsTag(TAG_UNIQUE_ID, short_address, DW1000.MODE_LONGDATA_RANGE_ACCURACY);
  DW1000Device anchor1(anchor1_short_address, true);
  DW1000Device anchor2(anchor2_short_address, true);
  DW1000Device anchor3(anchor3_short_address, true);
  DW1000Ranging.addNetworkDevices(&anchor1);
  DW1000Ranging.addNetworkDevices(&anchor2);
  DW1000Ranging.addNetworkDevices(&anchor3);
}

void loop() {
  DW1000Ranging.loop();
  
}

/// The following may be changed
void newRange() {
//  Serial.print("from: "); Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
//  Serial.print("\t Range: "); Serial.print(DW1000Ranging.getDistantDevice()->getRange()); Serial.print(" m");
//  Serial.print("\t RX power: "); Serial.print(DW1000Ranging.getDistantDevice()->getRXPower()); Serial.println(" dBm");

  Serial.print("$ ");
  Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  Serial.print("\t");
  Serial.print(DW1000Ranging.getDistantDevice()->getRange());
  Serial.print("\t");
  Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
  Serial.print("\t");
  Serial.print(DW1000Ranging.getDistantDevice()->getFPPower());
  Serial.print("\t");
  Serial.print(DW1000Ranging.getDistantDevice()->getPeakAmpl(), 0);
  Serial.print("\t");
  Serial.print(DW1000Ranging.getDistantDevice()->getFPAmpl(), 0);
  Serial.print("\t");
  Serial.print(DW1000Ranging.getDistantDevice()->getPPIndx(), 0);
  Serial.print("\t");
  Serial.println(DW1000Ranging.getDistantDevice()->getFPIndx(), 0);
}

void newDevice(DW1000Device* device) {
//  Serial.print("ranging init; 1 device added ! -> ");
//  Serial.print(" short:");
//  Serial.println(device->getShortAddress(), HEX);
  Serial.println('N');
}

void inactiveDevice(DW1000Device* device) {
//  Serial.print("delete inactive device: ");
//  Serial.println(device->getShortAddress(), HEX);
  Serial.println('D');
}

