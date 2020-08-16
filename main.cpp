#include "mbed.h"
#include "MPU6050.h"
//#include "QMC5883L.h"
#include "controller.h"
#include "PID.h"

#define Kc 1.55
#define Ti 0.03
#define Td 0.05
#define RATE 0.01

#define PITCH_IN_MIN -360.0
#define PITCH_IN_MAX 360.0
#define PITCH_OUT_MIN 1000
#define PITCH_OUT_MAX 2000
 
#define ROLL_IN_MIN -360.0
#define ROLL_IN_MAX 360.0
#define ROLL_OUT_MIN 1000
#define ROLL_OUT_MAX 2000

#define MPU_OFFSET_SAMPLES 50

// 0 AXIS1 - 1 AXIS2 - 2 AXIS3 (need sync with card position)
#define MPU_X_AXIS 0 //robotu oldugu yerde saga sola cevirme
#define MPU_Y_AXIS 1 //kafayi oldugu yerde yukari asagi yapma

static BufferedSerial serial_port(PA_9, PA_10, 9600);
FileHandle *mbed::mbed_override_console(int fd)
{
    return &serial_port;
}

MPU6050 mpu(PB_7,PB_6);
//QMC5883L qmc(PB_7,PB_6);

Timer timer;

DigitalOut led1(PB_0);
DigitalOut led2(PB_2);

PwmOut mOnSag(PB_15);
PwmOut mOnSol(PB_14);
PwmOut mArkaSag(PC_7);
PwmOut mArkaSol(PC_6);
PwmOut mUstArka(PA_4);
PwmOut mUstSag(PC_9);
PwmOut mUstSol(PC_8);

PID pitchPID (Kc, Ti, Td, RATE);
PID rollPID (Kc, Ti, Td, RATE);

CAN can1(PB_8, PB_9, 500000);
Controller controller;

bool isRobotActive = false;
void checkIsRobotActive() {
    if(controller.buttonA) {
        isRobotActive = true;
    }
    if(controller.buttonB) {
        isRobotActive = false;
    }
}

void resetRobot() {
    controller.Reset();
}


bool autonomousMod = false;
void checkAutonomous(bool robotActive) {

    if(!robotActive) {
        return;
    }

    if(controller.buttonX) {
        autonomousMod = false;
    }
    if(controller.buttonY) {
        autonomousMod = true;
    }

    if(autonomousMod) {
        led1 = true;
        led2 = false;
    }else {
        led2 = true;
        led1 = false;
    }
}

void AutonomousDrive(float pitchDiff, float rollDiff) {
    //not implemented
}

int byteToMotorValue(int val) {
    return val*4+(255/1000)+1000;
}

int fitMotorValue(int val, bool reverse = false){
    int result = val;
    
    if(result < 1530 && result > 1470) {
        return 1500;
    }

    if(val < 1000){
        result = 1000;
    }
    if(val > 2000){
        result = 2000;
    }
    if(reverse){
        result = 3000 - result;
    }
    return result;
}

void ManuelDrive(float pitchDiff, float rollDiff) {
    int mOnsag = 1500 - (byteToMotorValue(controller.rightY) - 1500) + (byteToMotorValue(controller.rightX) - 1500)  + (byteToMotorValue(controller.leftX) - 1500);
    int mOnsol = 1500 - (byteToMotorValue(controller.rightY) - 1500) - (byteToMotorValue(controller.rightX) - 1500)  - (byteToMotorValue(controller.leftX) - 1500);
    int mArkasag = 1500 + (byteToMotorValue(controller.rightY) - 1500) - (byteToMotorValue(controller.rightX) - 1500)  + (byteToMotorValue(controller.leftX) - 1500);
    int mArkasol = 1500 - (byteToMotorValue(controller.rightY) - 1500) - (byteToMotorValue(controller.rightX) - 1500)  + (byteToMotorValue(controller.leftX) - 1500);

    mOnSag.pulsewidth_us(fitMotorValue(mOnsag));
    mOnSol.pulsewidth_us(fitMotorValue(mOnsol));
    mArkaSag.pulsewidth_us(fitMotorValue(mArkasag,true));
    mArkaSol.pulsewidth_us(fitMotorValue(mArkasol));
    
    mUstArka.pulsewidth_us(fitMotorValue(byteToMotorValue(controller.leftY)));
    mUstSag.pulsewidth_us(fitMotorValue(byteToMotorValue(controller.leftY), true));
    mUstSol.pulsewidth_us(fitMotorValue(byteToMotorValue(controller.leftY)));
    /* 
    mOnSag.pulsewidth_us(fitMotorValue(byteToMotorValue(controller.leftY),true));
    mOnSol.pulsewidth_us(fitMotorValue(byteToMotorValue(controller.leftY),true));
    mArkaSag.pulsewidth_us(fitMotorValue(byteToMotorValue(controller.leftY),true));
    mArkaSol.pulsewidth_us(fitMotorValue(byteToMotorValue(controller.leftY),true));
    */
}

void initMotors() {
    mOnSag.period_ms(20);
    mOnSol.period_ms(20);
    mArkaSag.period_ms(20);
    mArkaSol.period_ms(20);
    mUstArka.period_ms(20);
    mUstSag.period_ms(20);
    mUstSol.period_ms(20);
    mUstSol.pulsewidth_us(1500);
    mOnSol.pulsewidth_us(1500);
    mArkaSag.pulsewidth_us(1500);
    mArkaSol.pulsewidth_us(1500);
    mUstArka.pulsewidth_us(1500);
    mUstSag.pulsewidth_us(1500);
    mUstSol.pulsewidth_us(1500);
    wait_us(3000000);
}

void initPID() {
    pitchPID.setInputLimits (PITCH_IN_MIN, PITCH_IN_MAX);  
    pitchPID.setOutputLimits (PITCH_OUT_MIN, PITCH_OUT_MAX);
 
    rollPID.setInputLimits (ROLL_IN_MIN, ROLL_IN_MAX);
    rollPID.setOutputLimits (ROLL_OUT_MIN, ROLL_OUT_MAX);
    
    rollPID.setBias(1500);
    pitchPID.setBias(1500);

    pitchPID.setMode(AUTO_MODE);  
    rollPID.setMode(AUTO_MODE);

    pitchPID.setSetPoint(0);
    rollPID.setSetPoint(0); 
}

void initGyro(float *accOffset, float *gyroOffset) {
    mpu.setAlpha(0.97);
    mpu.getOffset(accOffset, gyroOffset, MPU_OFFSET_SAMPLES);
    wait_us(1000);
}

int main()
{
    initMotors();
    CANMessage msg;
    initPID();

    float pitchDiff;
    float rollDiff;

    float currTime;
    float prevTime;
    float timeDiff;

    float accOffset[3];
    float gyroOffset[3];
    float angle[3]; 
    initGyro(accOffset, gyroOffset);

    timer.start();
    prevTime = chrono::duration<float>(timer.elapsed_time()).count();

    while (true) {
        currTime = chrono::duration<float>(timer.elapsed_time()).count();
        timeDiff = currTime - prevTime;
        mpu.computeAngle (angle, accOffset, gyroOffset, timeDiff);

        rollPID.setInterval(timeDiff);
        pitchPID.setInterval(timeDiff);

        prevTime = chrono::duration<float>(timer.elapsed_time()).count();

        rollPID.setProcessValue (angle[MPU_X_AXIS]); 
        pitchPID.setProcessValue (angle[MPU_Y_AXIS]);

        pitchDiff = pitchPID.compute();
        rollDiff = rollPID.compute();

        //mpu aci kalibrasyonu icin
        printf("timeDiff:%d\n", (int)timeDiff);
        printf("axis1:%d axis2:%d axis3:%d\n", (int)angle[0], (int)angle[1], (int)angle[2]);
        printf("pitch:%d roll:%d\n", (int)pitchDiff, (int)rollDiff);

        if (can1.read(msg)) {
            controller.SetKeyValues(msg.id, msg.data);
            checkIsRobotActive();
            checkAutonomous(isRobotActive);
        }
        if(!isRobotActive) {
            resetRobot();
            bool ledVal = !led1;
            led1 = ledVal;
            led2 = ledVal;
            wait_us(50000);
        }
        if(autonomousMod) {
            AutonomousDrive(pitchDiff, rollDiff);
        }
        else {
            ManuelDrive(pitchDiff, rollDiff);
        }
    }
}