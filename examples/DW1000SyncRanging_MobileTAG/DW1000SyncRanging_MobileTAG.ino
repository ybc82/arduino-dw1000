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

#define NODE_TAG2
#include "address.h"
const double mag_bias[3] = TAG2_MAG_BIAS;

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
byte base_short_address[] = BASE_SHORT_ADDRESS;

#define SPI_CLOCK 8000000  // 8MHz clock works.

#define MPU_SSn  8
#define intPin   12
#define myLed    13
#define adoPin   15
MPU9250 IMU(0x69, 0);
float q_output[4] = {1.0, 0.0, 0.0, 0.0};

int beginStatus;
int now, then;

//**************************************fusion settings**********************************************

#define AHRS true         // set to false for basic data read
#define SerialDebug true   // set to true to get Serial output for debugging
// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0, sumCount = 0; // used to control display output rate
float pitch, yaw, roll;
float deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval
float magbias[3] = {43, 25.5, -9};

float ax, ay, az, gx, gy, gz, mx, my, mz, t; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method
float g = 9.807;

//**************************************fusion settings**********************************************



void setup() {
    Serial.begin(115200);

// MPU part
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);
  pinMode(MPU_SSn, OUTPUT);
  digitalWrite(MPU_SSn, HIGH);
  pinMode(adoPin, OUTPUT);
  digitalWrite(adoPin, HIGH);

  beginStatus = IMU.begin(ACCEL_RANGE_4G, GYRO_RANGE_2000DPS);
  if(beginStatus < 0) {
    delay(1000);
    Serial.println("IMU failed");
  }
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
  DW1000Device anchor5(anchor5_short_address, true);
  DW1000Device base(base_short_address, true);
   // BASE must go first! See class DW1000SyncRangingClass::loop:_receivedAck:RANGE_ALL branch
  DW1000SyncRanging.addNetworkDevices(&base);
  DW1000SyncRanging.addNetworkDevices(&anchor1);
  DW1000SyncRanging.addNetworkDevices(&anchor2);
  DW1000SyncRanging.addNetworkDevices(&anchor3);
  DW1000SyncRanging.addNetworkDevices(&anchor4);
  DW1000SyncRanging.addNetworkDevices(&anchor5);
  then = micros();
//  readIMU();
}

void loop() {
  DW1000SyncRanging.loop();
  DW1000SyncRanging.loop();
   DW1000SyncRanging.loop();
   DW1000SyncRanging.loop();
//     DW1000SyncRanging.loop();


  now = micros();
  if((now - then) >= 22000) {
    then = now;
    readIMU();
    fusion();

    static int cnt_print = 0;
    cnt_print++;
    if (cnt_print >= 5)
    {
      cnt_print = 0;

      Serial.print('O');  Serial.print('\t');
      Serial.print(roll); Serial.print('\t');
      Serial.print(pitch);Serial.print('\t');
      Serial.print(yaw);  Serial.print('\t');
      
      Serial.print('A');  Serial.print('\t');
      Serial.print(ax);   Serial.print('\t');
      Serial.print(ay);   Serial.print('\t');
      Serial.print(az);   Serial.print('\t');
      Serial.println();
//    printIMU();
    }
    
  }
  
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
//mpu.read_all();
//Serial.println(millis());
#ifdef SEND_TO_RECV // This is designed for sending data to a specific data receiving module
  DW1000.newTransmit();
  DW1000.setDefaults();
  DW1000SyncRangingClass::data[0] = 0x10;
  DW1000SyncRangingClass::data[1] = 7;
  float position[3]; 
  for (int i = 0; i < 3; i++)
  {
    position[i] = DW1000SyncRanging.getPositionOutput()[i];
    memcpy(DW1000SyncRangingClass::data+2+i*4, &position[i], 4);
  }
  for (int i = 0; i < 4; i++)
  {
    memcpy(DW1000SyncRangingClass::data + 2 + 3*4 + i*4, &q[i], 4);
  }
  DW1000.setData(DW1000SyncRangingClass::data, LEN_DATA);
  DW1000.startTransmit();
#endif // SEND_TO_RECV
}

/* 
 *  printLong
 *  To print int64_t number in serial port
 */
//void printLong(int64_t num_in)
//{
//  uint64_t num = (uint64_t)(num_in);
//  char buf[18];
//  for (int i = 0; i < 4; i++)
//  {
//    sprintf(buf, "%04X", (uint16_t)(0xFFFF & (num >> (16*(3-i)))));
//    Serial.print(buf);
////    Serial.print((uint8_t)(0xFF & (num >> (8*(7-i)))), HEX);
//  }
//  Serial.print("\t");
//}

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

void readIMU() {
  IMU.getAccel(&ax, &ay, &az);
  ax /= g;
  ay /= g;
  az /= g;
  IMU.getGyro(&gx, &gy, &gz);
  gx = gx / PI * 180.0f;
  gy = gy / PI * 180.0f;
  gz = gz / PI * 180.0f;
  IMU.getMag(&mx, &my, &mz);
  mx -= mag_bias[0];
  my -= mag_bias[1];
  mz -= mag_bias[2];
  IMU.getTemp(&t);
}

void printIMU(){

  // print the data
  Serial.print("M");
  Serial.print(ax,6);
  Serial.print("\t");
  Serial.print(ay,6);
  Serial.print("\t");
  Serial.print(az,6);
  Serial.print("\t");

  Serial.print(gx,6);
  Serial.print("\t");
  Serial.print(gy,6);
  Serial.print("\t");
  Serial.print(gz,6);
  Serial.print("\t");

  Serial.print(mx,6);
  Serial.print("\t");
  Serial.print(my,6);
  Serial.print("\t");
  Serial.print(mz,6);
  Serial.print("\t");

  Serial.println(t,6);
}

void fusion() {
  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;

   // MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
  // MahonyQuaternionUpdate(ax, -ay, -az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f, my, -mx, -mz);
  //MahonyQuaternionUpdate(az, -ay, ax, gz*PI/180.0f, -gy*PI/180.0f, gx*PI/180.0f, mz, -mx, my);
  MahonyQuaternionUpdate(-az, ay, ax, -gz*PI/180.0f, gy*PI/180.0f, gx*PI/180.0f,  -mz,  my, mx);

  if (!AHRS) {
    delt_t = millis() - count;
    if(delt_t > 500) {

      if(SerialDebug) {
        // Print acceleration values in milligs!
        Serial.print("X-acceleration: "); Serial.print(1000*ax); Serial.print(" mg ");
        Serial.print("Y-acceleration: "); Serial.print(1000*ay); Serial.print(" mg ");
        Serial.print("Z-acceleration: "); Serial.print(1000*az); Serial.println(" mg ");
     
        // Print gyro values in degree/sec
        Serial.print("X-gyro rate: "); Serial.print(gx / PI * 180.0f, 3); Serial.print(" degrees/sec "); 
        Serial.print("Y-gyro rate: "); Serial.print(gy/ PI * 180.0f, 3); Serial.print(" degrees/sec "); 
        Serial.print("Z-gyro rate: "); Serial.print(gz/ PI * 180.0f, 3); Serial.println(" degrees/sec"); 
        
        // Print mag values in degree/sec
        Serial.print("X-mag field: "); Serial.print(mx); Serial.print(" mG "); 
        Serial.print("Y-mag field: "); Serial.print(my); Serial.print(" mG "); 
        Serial.print("Z-mag field: "); Serial.print(mz); Serial.println(" mG"); 
     
        //temperature = ((float) tempCount) / 333.87 + 21.0; // Temperature in degrees Centigrade
       // Print temperature in degrees Centigrade      
        Serial.print("Temperature is ");  Serial.print((float)t, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
      }
    
      count = millis();
      digitalWrite(myLed, !digitalRead(myLed));  // toggle led
    }
  }
  else {
  
    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;
    //if (delt_t > 500)
    { // update LCD once per half-second independent of read rate

      if(SerialDebug) {
//        Serial.print("ax = "); Serial.print((int)1000*ax);  
//        Serial.print(" ay = "); Serial.print((int)1000*ay); 
//        Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
//        Serial.print("gx = "); Serial.print( gx, 2); 
//        Serial.print(" gy = "); Serial.print( gy, 2); 
//        Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
//        Serial.print("mx = "); Serial.print( (int)mx ); 
//        Serial.print(" my = "); Serial.print( (int)my ); 
//        Serial.print(" mz = "); Serial.print( (int)mz ); Serial.println(" mG");
//        
//        Serial.print("q0 = "); Serial.print(q[0]);  
//        Serial.print(" qx = "); Serial.print(q[1]); 
//        Serial.print(" qy = "); Serial.print(q[2]); 
//        Serial.print(" qz = "); Serial.println(q[3]); 
      }               

      // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
      // In this coordinate system, the positive z-axis is down toward Earth. 
      // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
      // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
      // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
      // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
      // applied in the correct order which for this configuration is yaw, pitch, and then roll.
      // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
      yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
      pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
      roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
      pitch *= 180.0f / PI;
      yaw   *= 180.0f / PI; 
      // yaw   += 120.0;//highbay offset
      roll  *= 180.0f / PI;
      //EulerToQuaternion(pitch*PI/180.0f, roll*PI/180.0f, yaw*PI/180.0f, q_output);
       
      if(SerialDebug) {
      //Serial.print("Yaw, Pitch, Roll: ");
//      Serial.print(yaw, 2);
//      Serial.print(" ");
//      Serial.print(pitch, 2);
//      Serial.print(" ");
//      Serial.println(roll, 2);
      
      //Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
      }  

    // With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and 
    // >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
    // The filter update rate is determined mostly by the mathematical steps in the respective algorithms, 
    // the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
    // an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
    // filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively. 
    // This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
    // This filter update rate should be fast enough to maintain accurate platform orientation for 
    // stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
    // produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
    // The 3.3 V 8 MHz Pro Mini is doing pretty well!

      count = millis(); 
      sumCount = 0;
      sum = 0;    
    }
  }
}

static void EulerToQuaternion(double _pitch, double _roll, double _yaw, float* q_output)
{
  double t0 = cos(_yaw * 0.5);
  double t1 = sin(_yaw * 0.5);
  double t2 = cos(_roll * 0.5);
  double t3 = sin(_roll * 0.5);
  double t4 = cos(_pitch * 0.5);
  double t5 = sin(_pitch * 0.5);

  q_output[0] = t0 * t2 * t4 + t1 * t3 * t5;
  q_output[1] = t0 * t3 * t4 - t1 * t2 * t5;
  q_output[2] = t0 * t2 * t5 + t1 * t3 * t4;
  q_output[3] = t1 * t2 * t4 - t0 * t3 * t5;
}
