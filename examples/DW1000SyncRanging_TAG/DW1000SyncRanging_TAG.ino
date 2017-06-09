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
#include <MPU9250.h>
#include <MatrixMath.h>

#define NODE_TAG1
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
byte base_short_address[] = BASE_SHORT_ADDRESS;

#define SPI_CLOCK 8000000  // 8MHz clock works.

#define SS_PIN   8
#define INT_PIN  3
#define LED      14
MPU9250 mpu(SPI_CLOCK, SS_PIN);

void setup() {
    Serial.begin(115200);

  pinMode(INT_PIN, INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  pinMode(10, OUTPUT);
  digitalWrite(10, 1);

  SPI.begin();

  mpu.init(true);

  uint8_t wai = mpu.whoami();
  if (wai == 0x71){
    Serial.println("Successful connection");
  }
  else{
    Serial.print("Failed connection: ");
    Serial.println(wai, HEX);
  }
//
//  uint8_t wai_AK8963 = mpu.AK8963_whoami();
//  if (wai_AK8963 == 0x48){
//    Serial.println("Successful connection to mag");
//  }
//  else{
//    Serial.print("Failed connection to mag: ");
//    Serial.println(wai_AK8963, HEX);
//  }
//
//  mpu.calib_acc();
//  mpu.calib_mag();
//
//  Serial.println("Send any char to begin main loop.");


  
//  Serial.begin(115200);
  delay(1000);

  //init the configuration
  DW1000SyncRanging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  //define the sketch as anchor. It will be great to dynamically change the type of module
  DW1000SyncRanging.attachNewRange(newRange);
  DW1000SyncRanging.attachNewDevice(newDevice);
  DW1000SyncRanging.attachInactiveDevice(inactiveDevice);
  //Enable the filter to smooth the distance
  //DW1000SyncRanging.useRangeFilter(true);
  
  //we start the module as a tag
  DW1000SyncRanging.startAsTag(UNIQUE_ID, short_address, DW1000.MODE_LONGDATA_RANGE_ACCURACY);
  DW1000Device anchor1(anchor1_short_address, true);
  DW1000Device anchor2(anchor2_short_address, true);
  DW1000Device anchor3(anchor3_short_address, true);
  DW1000Device anchor4(anchor4_short_address, true);
  DW1000Device base(base_short_address, true);
   // BASE must go first! See class DW1000SyncRangingClass::loop:_receivedAck:RANGE_ALL branch
  DW1000SyncRanging.addNetworkDevices(&base);
  DW1000SyncRanging.addNetworkDevices(&anchor1);
  DW1000SyncRanging.addNetworkDevices(&anchor2);
  DW1000SyncRanging.addNetworkDevices(&anchor3);
  DW1000SyncRanging.addNetworkDevices(&anchor4);
}

void loop() {
  DW1000SyncRanging.loop();
}

/// The following may be changed
void newRange() {/*
//  Serial.print("from: "); Serial.print(DW1000SyncRanging.getDistantDevice()->getShortAddress(), HEX);
//  Serial.print("\t Range: "); Serial.print(DW1000SyncRanging.getDistantDevice()->getRange()); Serial.print(" m");
//  Serial.print("\t RX power: "); Serial.print(DW1000SyncRanging.getDistantDevice()->getRXPower()); Serial.println(" dBm");
//  Serial.print(millis());
  Serial.print("$ ");
  Serial.print(DW1000SyncRanging.getDistantDevice()->getShortAddress(), HEX);
  Serial.print("\t");
  Serial.print(DW1000SyncRanging.getDistantDevice()->getRange());
  Serial.print("\t");
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

  printLong(DW1000SyncRanging.getBaseDevice()->timePollSent.getTimestamp());
  printLong(DW1000SyncRanging.getBaseDevice()->timePollReceived.getTimestamp());
  printLong(DW1000SyncRanging.getBaseDevice()->timeRangeSent.getTimestamp());
  printLong(DW1000SyncRanging.getBaseDevice()->timeRangeReceived.getTimestamp());
  printLong(DW1000SyncRanging.getDistantDevice()->timePollReceived.getTimestamp());
  printLong(DW1000SyncRanging.getDistantDevice()->timeRangeReceived.getTimestamp());
  printLong(DW1000SyncRanging.getDistantDevice()->timeRangeAllSent.getTimestamp());
  printLong(DW1000SyncRanging.getDistantDevice()->timeRangeAllReceived.getTimestamp());
  Serial.println();
//  Serial.println(millis());
//mpu.read_all();
  */
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

