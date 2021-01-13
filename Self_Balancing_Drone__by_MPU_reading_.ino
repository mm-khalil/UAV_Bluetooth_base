//in order to fly drone autonomously then merge MPU code with Motors code both are here in this code but not merged so merge it and use it

//drone motors code
#include <Servo.h>
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

int escPin1 = 3;
int escPin2 = 5;
int escPin3 = 6;
int escPin4 = 9;

int minPulseRate = 1000;
int maxPulseRate = 2000;
int throttleChangeDelay = 200;

int change_value   = 110;
int constant_value = 50;

void setup() 
{  Serial.begin(9600);
   jerk_motors();
   start_motors();  
}

void loop() 
{   
        
        
       
       
        
}

void jerk_motors ()
{    Serial.setTimeout(500);
  
     // Attach the the servo to the correct pin and set the pulse range
     esc1.attach(escPin1, minPulseRate, maxPulseRate); 
     esc2.attach(escPin2, minPulseRate, maxPulseRate); 
     esc3.attach(escPin3, minPulseRate, maxPulseRate); 
     esc4.attach(escPin4, minPulseRate, maxPulseRate); 
  
  // Write a minimum value (most ESCs require this correct startup)
     esc1.write(0);
     esc2.write(0);
     esc3.write(0);
     esc4.write(0);
       delay(2000);
    
     esc1.write(35);
     esc2.write(35);
     esc3.write(35);
     esc4.write(35);
        delay(1000);
      
     esc1.write(40);
     esc2.write(40);
     esc3.write(40);
     esc4.write(40);
        delay(1000);  
}


    
void start_motors()
{
   esc1.write(35);
   esc2.write(35);
   esc3.write(35);
   esc4.write(35);   //min threshold is 30;
      delay(6000);
    
   esc1.write(0);
   esc2.write(0);
   esc3.write(0);
   esc4.write(0); 
     delay(1000);
    
   esc1.write(40);
   esc2.write(40);
   esc3.write(40);
   esc4.write(40); 
      delay(6000);  
}

void stop_motors()
{           esc1.write(0);
            esc2.write(0);
            esc3.write(0);
            esc4.write(0);       }

void forward () 
{           esc1.write(change_value);
            esc2.write(change_value);
            esc3.write(constant_value);
            esc4.write(constant_value);
            //   Serial.println("forward");
            }

void reverse () 
{           esc1.write(constant_value);
            esc2.write(constant_value);
            esc3.write(change_value);
            esc4.write(change_value);
            //Serial.println("reverse");}

void move_left () 
{           esc1.write(change_value);
            esc2.write(constant_value);
            esc3.write(change_value);
            esc4.write(constant_value);
            //Serial.println("moving left");
            }

void move_right () 
{           esc1.write(constant_value);
            esc2.write(change_value);
            esc3.write(constant_value);
            esc4.write(change_value);
            //Serial.println("moving right ");
            }
                         











//MPU code

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

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

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
   
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
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

       packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
     
    }

    
    pinMode(LED_PIN, OUTPUT);
}



void loop() {
    if (!dmpReady) return;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_QUATERNION
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        // blink LED to indicate activity 
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}




  
