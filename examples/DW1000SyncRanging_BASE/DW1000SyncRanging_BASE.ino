/**
 * 
 * @todo
 *  - move strings to flash (less RAM consumption)
 *  - fix deprecated convertation form string to char* startAsTag
 *  - give example description
 */
#include <SPI.h>
#include "DW1000SyncRanging.h"
#include "DW1000Time.h"
//#include <MPU9250.h>

#define NODE_BASE
#include "address.h"

// connection pins
const uint8_t PIN_RST = PIN_NUM_RST; // reset pin
const uint8_t PIN_IRQ = PIN_NUM_IRQ; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

const char short_address[] = SHORT_ADDRESS;
byte anchor1_short_address[] = ANCHOR1_SHORT_ADDRESS;
byte anchor2_short_address[] = ANCHOR2_SHORT_ADDRESS;
byte anchor3_short_address[] = ANCHOR3_SHORT_ADDRESS;
byte anchor4_short_address[] = ANCHOR4_SHORT_ADDRESS;
byte anchor5_short_address[] = ANCHOR5_SHORT_ADDRESS;

#define SPI_CLOCK 8000000  // 8MHz clock works.

#define SS_PIN   8
#define INT_PIN  3
#define LED      14
//MPU9250 mpu(SPI_CLOCK, SS_PIN);

void setup() {
  Serial.begin(115200);

//  pinMode(INT_PIN, INPUT);
//  pinMode(LED, OUTPUT);
//  digitalWrite(LED, HIGH);
//
//  pinMode(10, OUTPUT);
//  digitalWrite(10, 1);
//
//  SPI.begin();
//
//  mpu.init(true); // to disable I2C for MPU9250
//  delay(1000);

  //init the configuration
  DW1000SyncRanging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  //define the sketch as anchor. It will be great to dynamically change the type of module
  DW1000SyncRanging.attachNewRange(newRange);
  DW1000SyncRanging.attachNewDevice(newDevice);
  DW1000SyncRanging.attachInactiveDevice(inactiveDevice);
  //Enable the filter to smooth the distance
  //DW1000SyncRanging.useRangeFilter(true);
  
  //we start the module as a Base
  DW1000SyncRanging.startAsBase(UNIQUE_ID, short_address, DW1000.MODE_LONGDATA_RANGE_ACCURACY);
  DW1000Device anchor1(anchor1_short_address, true);
  DW1000Device anchor2(anchor2_short_address, true);
  DW1000Device anchor3(anchor3_short_address, true);
  DW1000Device anchor4(anchor4_short_address, true);
  DW1000Device anchor5(anchor5_short_address, true);
  
  DW1000SyncRanging.addNetworkDevices(&anchor1);
  DW1000SyncRanging.addNetworkDevices(&anchor2);
  DW1000SyncRanging.addNetworkDevices(&anchor3);
  DW1000SyncRanging.addNetworkDevices(&anchor4);
  DW1000SyncRanging.addNetworkDevices(&anchor5);
}

void loop() {
  DW1000SyncRanging.loop();
  
}

/// The following may be changed
void newRange() {
//  Serial.print("from: "); Serial.print(DW1000SyncRanging.getDistantDevice()->getShortAddress(), HEX);
//  Serial.print("\t Range: "); Serial.print(DW1000SyncRanging.getDistantDevice()->getRange()); Serial.print(" m");
//  Serial.print("\t RX power: "); Serial.print(DW1000SyncRanging.getDistantDevice()->getRXPower()); Serial.println(" dBm");

//  Serial.print("$ ");
//  Serial.print(DW1000SyncRanging.getDistantDevice()->getShortAddress(), HEX);
//  Serial.print("\t");
//  Serial.print(DW1000SyncRanging.getDistantDevice()->getRange());
//  Serial.print("\t");
//  Serial.print(DW1000SyncRanging.getDistantDevice()->getRXPower());
//  Serial.print("\t");
//  Serial.print(DW1000SyncRanging.getDistantDevice()->getFPPower());
//  Serial.print("\t");
//  Serial.print(DW1000SyncRanging.getDistantDevice()->getPeakAmpl(), 0);
//  Serial.print("\t");
//  Serial.print(DW1000SyncRanging.getDistantDevice()->getFPAmpl(), 0);
//  Serial.print("\t");
//  Serial.print(DW1000SyncRanging.getDistantDevice()->getPPIndx(), 0);
//  Serial.print("\t");
//  Serial.println(DW1000SyncRanging.getDistantDevice()->getFPIndx(), 0);
 
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

