///////////////////////////////////    IMU CONFIGURATION   /////////////////////////////

// I2Cdev and MPU6050 must be installed as libraries
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "limits.h"


// default I2C address is 0x68
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu; // <-- use for AD0 high

//raw motion variables
int16_t ax, ay, az, gx, gy, gz;

//Calibration parameters
int buffersize = 100;   //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone = 20;   //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 20;   //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

//Calibration variables
int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
int ready = 0;

#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_WORLDACCEL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <PololuMaestro.h>

/* On boards with a hardware serial port available for use, use
  that port to communicate with the Maestro. For other boards,
  create a SoftwareSerial object using pin 10 to receive (RX) and
  pin 11 to transmit (TX). */

#ifdef SERIAL_PORT_HARDWARE_OPEN
#define maestroSerial SERIAL_PORT_HARDWARE_OPEN
#else
#include <SoftwareSerial.h>
SoftwareSerial maestroSerial(10, 11);
#endif

/* Next, create a Maestro object using the serial port. */

MicroMaestro maestro(maestroSerial);



int incoming[12];

float a0 = 90;
float a1 = 90;
float a2 = 90;
float a3 = 90;
float a4 = 90;
float a5 = 90;

void setup() {
  // Set the serial baud rate.
  Serial.begin(9600);
  setup_imu();
  maestroSerial.begin(9600);
  delay(100);
}

void loop() {

//  makesureserialisopen();
  if (Serial.available() >= 0) {
    if (Serial.read() == 99) calibration();
  }


  if (Serial.available() >= 12) {
    for (int i = 0; i < 12; i++) {
      incoming[i] = Serial.read();
    }

    a0 = incoming[0] + incoming[1] / 10;
    a1 = incoming[2] + incoming[3] / 10;
    a2 = incoming[4] + incoming[5] / 10;
    a3 = incoming[6] + incoming[7] / 10;
    a4 = incoming[8] + incoming[9] / 10;
    a5 = incoming[10] + incoming[11] / 10;

    servo_0(a0);
    servo_1(a1);
    servo_2(a2);
    servo_3(a3);
    servo_4(a4);
    servo_5(a5);
    gibdata();
  }

}

void servo_0(float angle) {
  int MicroSeconds = mapf(angle, 0.0, 180.0, 576.0, 2128.0);
  Serial.println(MicroSeconds);
  maestro.setTarget(0, MicroSeconds * 4);
}

void servo_1(float angle) {
  int MicroSeconds = mapf(angle, 0, 180, 592, 2288);
  maestro.setTarget(1, MicroSeconds * 4);
}

void servo_2(float angle) {
  int MicroSeconds = mapf(angle, 0, 180, 592, 2160);
  maestro.setTarget(2, MicroSeconds * 4);
}

void servo_3(float angle) {
  int MicroSeconds = mapf(angle, 0, 180, 560, 2128);
  maestro.setTarget(3, MicroSeconds * 4);
}

void servo_4(float angle) {
  int MicroSeconds = mapf(angle, 0, 180, 560, 2016);
  maestro.setTarget(4, MicroSeconds * 4);
}

void servo_5(float angle) {
  int MicroSeconds = mapf(angle, 0, 180, 560, 2192);
  maestro.setTarget(5, MicroSeconds * 4);
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



/////////////////////////////////////////////////////////////////////////////////////////

void setup_imu() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  // COMMENT NEXT LINE IF YOU ARE USING ARDUINO DUE
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Leonardo measured 250kHz.

  // initialize device
  mpu.initialize();

  // start message
  delay(1000);
  // verify connection
  if (!mpu.testConnection()) {
    Serial.println("IMU connection could not be established");
  }

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.println("DMP can not be initialized");
  }

}


int sabs(int i) {
  int res;

  if (INT_MIN == i)
  {
    res = INT_MAX;
  }
  else
  {
    res = i < 0 ? -i : i;
  }

  return res;
}

void meansensors() {

  i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

  while ( i < buffersize + 11 ) {
    // read raw accel/gyro measurements from device
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (i > 10 && i <= (buffersize + 10)) { //First 10 measures are discarded
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if (i == (buffersize + 10)) {
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

void calibration() {
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;

  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;
  ready = 0;
  while (1) {
    Serial.println(ready);
    ready = 0;
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);

    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);

    meansensors();

    if (sabs(mean_ax) <= acel_deadzone) {
      ready++;
    }
    else {
      ax_offset = ax_offset - mean_ax / 10;
    }
    if (sabs(mean_ay) <= acel_deadzone) {
      ready++;
    }
    else {
      ay_offset = ay_offset - mean_ay / 10;
    }
    if (sabs(16384 - mean_az) <= acel_deadzone) {
      ready++;
    }
    else {
      az_offset = az_offset + (16384 - mean_az) / 10;
    }
    if (sabs(mean_gx) <= giro_deadzone) {
      ready++;
    }
    else {
      gx_offset = gx_offset - mean_gx / 5;
    }
    if (sabs(mean_gy) <= giro_deadzone) {
      ready++;
    }
    else {
      gy_offset = gy_offset - mean_gy / 5;
    }
    if (sabs(mean_gz) <= giro_deadzone) {
      ready++;
    }
    else {
      gz_offset = gz_offset - mean_gz / 5;
    }
    if (ready == 6) break;
  }

  Serial.println("b");
}

void gibdata() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  mpu.resetFIFO();
  fifoCount = 0 ;

  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);


#ifdef OUTPUT_READABLE_WORLDACCEL
  // display initial world-frame acceleration, adjusted to remove gravity
  // and rotated based on known orientation from quaternion
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  Serial.print(aaWorld.x);
  Serial.print(",");
  Serial.print(aaWorld.y);
  Serial.print(",");
  Serial.print(aaWorld.z);
  Serial.print(",");
  Serial.flush();
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  Serial.print(ypr[0] * 180 / M_PI);
  Serial.print(",");
  Serial.print(ypr[1] * 180 / M_PI);
  Serial.print(",");
  Serial.println(ypr[2] * 180 / M_PI);
  Serial.flush();
#endif
}


void makesureserialisopen() {
  if (!Serial) { //check if Serial is available... if not,
    Serial.end();      // close serial port
    delay(100);        //wait 100 millis
    Serial.begin(9600); // reenable serial again
  }
}


