// #line 1 "/Users/yewon/Library/CloudStorage/OneDrive-연세대학교(YonseiUniversity)/예원/YONSEI/동아리/RoboIn/22-1 학기 중 프로젝트/3xGimbal/3axis_gymbol/mpu6050_wrapper.cpp"
//2022-1 Semester Yonsei-RoboIN 3 Axis Gymbal Project. 
//@author: ga06033@yonsei.ac.kr
//@date: 2022-06-23

#include "mpu6050_wrapper.h"

bool blinkState = false;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
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

MPU6050 mpu;

void init_mpu(){
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
        //  Following line will prevent mpu6050 stopping
        // Wire.setWireTimeout(2500 /* us */, true /* reset_on_timeout */);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(baud_rate);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize(9);

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(X_ACC_OFFSET);
    mpu.setYAccelOffset(Y_ACC_OFFSET);
    mpu.setZAccelOffset(Z_ACC_OFFSET);

    mpu.setXGyroOffset(X_GYRO_OFFSET);
    mpu.setYGyroOffset(Y_GYRO_OFFSET);
    mpu.setZGyroOffset(Z_GYRO_OFFSET);


    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // // // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

void dmpDataReady() {mpuInterrupt = true;}

void get_ypr(float* ypr){
    // if programming failed, don't try to do anything
    if (!dmpReady)  return;
    unsigned long st_time = micros();
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        if (st_time-micros() > MPU_TIMEOUT){
            Serial.println("I died! =============================");
            init_mpu();
            break;
        }
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        //Serial.print("ypr\t");
        ypr[0] = ypr[0] * 180/M_PI;
        ypr[1] = ypr[1] * 180/M_PI;
        ypr[2] = ypr[2] * 180/M_PI;
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
    
}