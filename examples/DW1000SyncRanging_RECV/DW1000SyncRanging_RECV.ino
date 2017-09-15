#include <SPI.h>
#include <DW1000.h>
#include <ros.h>
#include <geometry_msgs/Pose.h>

#define NODE_RECV
#include "address.h"

ros::NodeHandle nh;
geometry_msgs::Pose geo_msg;
ros::Publisher chatter("UWB/Pose2", &geo_msg);

// connection pins
const uint8_t PIN_RST = PIN_NUM_RST; // reset pin
const uint8_t PIN_IRQ = PIN_NUM_IRQ; // irq pin
const uint8_t PIN_SS = SS; // spi select pin


// DEBUG packet sent status and count
volatile boolean received = false;
volatile boolean error = false;
volatile int16_t numReceived = 0; // todo check int type
byte message[100];

float pitch, roll, yaw;
float q_output[4];

void setup() {
  nh.initNode();
  nh.advertise(chatter);
  
  // DEBUG monitoring
  Serial.begin(115200);
  Serial.println(F("### DW1000-arduino-receiver-test ###"));
  // initialize the driver
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  Serial.println(F("DW1000 initialized ..."));
  // general configuration
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(6);
  DW1000.setNetworkId(0xDECA);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_ACCURACY);
  DW1000.commitConfiguration();
  Serial.println(F("Committed configuration ..."));
  // DEBUG chip info and registers pretty printed
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000.getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000.getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000.getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);
  // attach callback for (successfully) received messages
  DW1000.attachReceivedHandler(handleReceived);
  DW1000.attachReceiveFailedHandler(handleError);
  DW1000.attachErrorHandler(handleError);
  // start reception
  receiver();
}

void handleReceived() {
  // status change on reception success
  received = true;
}

void handleError() {
  error = true;
}

void receiver() {
  DW1000.newReceive();
  DW1000.setDefaults();
  // so we don't need to restart the receiver manually
  DW1000.receivePermanently(true);
  DW1000.startReceive();
}

float position[3] = {0, 0, 0};
float q[4] = {1, 0, 0, 0};
uint16_t lenPacket = 0;
void loop() {
  // enter on confirmation of ISR status change (successfully received)
  if (received) {
    numReceived++;
    // get data as string
    lenPacket = DW1000.getDataLength();
    DW1000.getData(message, lenPacket);
    if (message[0] == 0x10)
    {
      for (int i = 0; i < 3; i++)
      {
        memcpy(&position[i], message+2+i*4, 4);
        Serial.print(position[i]);  Serial.print('\t');
      }
      for (int i = 0; i < 4; i++)
      {
        memcpy(&q[i], message+2+3*4+i*4, 4);
        Serial.print(q[i]);         Serial.print('\t');
      }
      Serial.println();
      yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
      pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
      roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
      pitch *= 180.0f / PI;
      yaw   *= 180.0f / PI; 
      yaw   -= 9.335;//13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
      yaw   += 120.0;//highbay offset
      roll  *= 180.0f / PI;
      EulerToQuaternion(pitch * PI/180.0, roll* PI/180.0, yaw* PI/180.0, q_output);

      geo_msg.position.x = position[0];
      geo_msg.position.y = position[1];
      geo_msg.position.z = 0;
      geo_msg.orientation.w = q_output[0];
      geo_msg.orientation.x = q_output[1];
      geo_msg.orientation.y = q_output[2];
      geo_msg.orientation.z = q_output[3];
      chatter.publish( &geo_msg );
      nh.spinOnce();
    }
//    Serial.print("Received message ... #"); Serial.println(numReceived);
//    Serial.print("Data is ... "); Serial.println(message);
//    Serial.print("FP power is [dBm] ... "); Serial.println(DW1000.getFirstPathPower());
//    Serial.print("RX power is [dBm] ... "); Serial.println(DW1000.getReceivePower());
//    Serial.print("Signal quality is ... "); Serial.println(DW1000.getReceiveQuality());
    received = false;
  }
  if (error) {
    Serial.println("Error receiving a message");
    error = false;
    lenPacket = DW1000.getDataLength();
    DW1000.getData(message, lenPacket);
//    Serial.print("Error data is ... "); Serial.println(message);
  }
}

static void EulerToQuaternion(double pitch, double roll, double yaw, float* q_output)
{
  double t0 = cos(yaw * 0.5);
  double t1 = sin(yaw * 0.5);
  double t2 = cos(roll * 0.5);
  double t3 = sin(roll * 0.5);
  double t4 = cos(pitch * 0.5);
  double t5 = sin(pitch * 0.5);

  q_output[0] = t0 * t2 * t4 + t1 * t3 * t5;
  q_output[1] = t0 * t3 * t4 - t1 * t2 * t5;
  q_output[2] = t0 * t2 * t5 + t1 * t3 * t4;
  q_output[3] = t1 * t2 * t4 - t0 * t3 * t5;
}
