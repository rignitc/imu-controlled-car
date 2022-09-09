#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN D8

bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q; // [w, x, y, z]         quaternion container
VectorInt16 aa; // [x, y, z]            accel sensor measurements
VectorInt16 aaReal; // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3]; // [psi, theta, phi]    Euler angle container
float ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;
void ICACHE_RAM_ATTR dmpDataReady() {
  mpuInterrupt = true;
}
int motorSpeedA = 0;
int motorSpeedB = 0;
int xAxis = 0;
int yAxis = 0;
int flag = 1;

/////////////////////////////////////////

const unsigned int SERVER_PORT  = 9999;
const char *SERVER_NAME         = "NODEMCU";
const char *SERVER_PASWD        = "";
const int SERVER_CHANNEL        = 4;

IPAddress AP_SERVER(192, 168, 4, 1);

typedef struct _udp_packet {
  int pwm_l;
  int pwm_r;
} UDP_PACKET;

WiFiUDP Udp;
//UDP_PACKET packet = {1, "room", 0, true};
//UDP_PACKET packet = {2, "kitchen", 0, false};
UDP_PACKET packet = {1,1};

/////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);

  
  ///////////////////////////////////////
  WiFi.mode(WIFI_STA);
  Serial.printf("Connecting to %s ", SERVER_NAME);
  WiFi.begin(SERVER_NAME, SERVER_PASWD);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println(" connected");
  Udp.begin(SERVER_PORT);
  ////////////////////////////////////

  
  //Wire.begin();    
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  
}

void loop() {

  //////////////////////////
  
  //////////////////////////
  flag--;
  if (flag < 0) {
    yAxis = int(710.47*(q.x - 0.72) + 1023);
    xAxis = int(710.47*(q.y - 0.72) + 1023);
    if (yAxis < 470) {
      //              digitalWrite(in1, HIGH);
      //              digitalWrite(in2, LOW);
      //              digitalWrite(in3, HIGH);
      //              digitalWrite(in4, LOW);
      motorSpeedA = map(yAxis, 470, 0, 0, 255);
      motorSpeedB = map(yAxis, 470, 0, 0, 255);
    } else if (yAxis > 550) {
      //              digitalWrite(in1, LOW);
      //              digitalWrite(in2, HIGH);
      //              digitalWrite(in3, LOW);
      //              digitalWrite(in4, HIGH);
      motorSpeedA = map(yAxis, 550, 1023, 0, 255);
      motorSpeedB = map(yAxis, 550, 1023, 0, 255);
    } else {
      motorSpeedA = 0;
      motorSpeedB = 0;
    }

    if (xAxis < 470) {

      int xMapped = map(xAxis, 470, 0, 0, 255);
      motorSpeedA = motorSpeedA - xMapped;
      motorSpeedB = motorSpeedB + xMapped;

      if (motorSpeedA < 0) {
        motorSpeedA = 0;
      }
      if (motorSpeedB > 255) {
        motorSpeedB = 255;
      }
    }
    if (xAxis > 550) {
      int xMapped = map(xAxis, 550, 1023, 0, 255);
      motorSpeedA = motorSpeedA + xMapped;
      motorSpeedB = motorSpeedB - xMapped;
      if (motorSpeedA > 255) {
        motorSpeedA = 255;
      }
      if (motorSpeedB < 0) {
        motorSpeedB = 0;
      }
    }

    if (motorSpeedA < 70) {
      motorSpeedA = 0;
    }
    if (motorSpeedB < 70) {
      motorSpeedB = 0;
    }
    Serial.println("");
    Serial.print("MotorSpeedA:");
    Serial.print(motorSpeedA);
    Serial.print("MotorSpeedB:");
    Serial.print(motorSpeedB);
    Serial.println("");
    packet.pwm_l = motorSpeedA;
    packet.pwm_r = motorSpeedB;
    Serial.println("PWM L is ");
    Serial.println(packet.pwm_l);
    Serial.println("PWM R is ");
    Serial.println(packet.pwm_r);
    Udp.beginPacket(AP_SERVER, SERVER_PORT);
    Udp.write((byte *)&packet, sizeof(UDP_PACKET));
    Udp.endPacket();
  }

  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize) {}

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
  } else if (mpuIntStatus & 0x01) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);

    fifoCount -= packetSize;

    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion( & q, fifoBuffer);
    mpu.dmpGetGravity( & gravity, & q);
    mpu.dmpGetYawPitchRoll(ypr, & q, & gravity);
    mpu.dmpGetAccel( & aa, fifoBuffer);
    mpu.dmpGetGravity( & gravity, & q);
    mpu.dmpGetLinearAccel( & aaReal, & aa, & gravity);
    mpu.dmpGetLinearAccelInWorld( & aaWorld, & aaReal, & q);

    Serial.print('/n');
    Serial.print("Orientation X: ");
    Serial.print(q.x);
    Serial.print(", Y: ");
    Serial.print(q.y);
    Serial.print(", Z: ");
    Serial.print(q.z);
    Serial.print(", W: ");
    Serial.print(q.w);
    Serial.println("");

    Serial.print("Acceleration X: ");
    Serial.print(aaReal.x * 1 / 16384. * 9.80665);
    Serial.print(", Y: ");
    Serial.print(aaReal.y * 1 / 16384. * 9.80665);
    Serial.print(", Z: ");
    Serial.print(aaReal.z * 1 / 16384. * 9.80665);
    Serial.println("");

    Serial.print("Rotation X: ");
    Serial.print(ypr[0]);
    Serial.print(", Y: ");
    Serial.print(ypr[1]);
    Serial.print(", Z: ");
    Serial.print(ypr[2]);
    Serial.println("");

    delay(200);
  }

}
void send_pwm(int pwm_l, int pwm_r){
  packet.pwm_l = pwm_l;
  packet.pwm_r = pwm_r;
  Udp.beginPacket(AP_SERVER, SERVER_PORT);
  Udp.write((byte *)&packet, sizeof(UDP_PACKET));
  Udp.endPacket();
}
