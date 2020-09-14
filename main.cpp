#include "mbed.h"
#include "MPU6050.h"
//#include "QMC5883L.h"
#include "controller.h"
#include "PID.h"
//#include "MS5837.h"

#define PID_ROLL_Kc 2.16
#define PID_ROLL_Ti 1.75
#define PID_ROLL_Td 0.01

#define PID_PITCH_Kc 2.16
#define PID_PITCH_Ti 1.75
#define PID_PITCH_Td 0.01

#define PID_YAW_Kc 2.61
#define PID_YAW_Ti 0.75
#define PID_YAW_Td 0.2

#define PID_ROLL_PITCH_IN 90

#define PID_YAW_IN 360

#define PID_OUT_MIN 1000
#define PID_OUT_MAX 2000
#define PID_BIAS 1500

#define MPU_OFFSET_SAMPLES 50

#define MPU_X_AXIS 2 //ROBOTUN OLDUGU YERDE SAGA SOLA DONMESI
#define MPU_Y_AXIS 0 //ROBOTUN KAFASINI YUKARI ASAGI YAPMASI
#define MPU_Z_AXIS 1 //ROBOTUN SAGA SOLA YATMASI

static BufferedSerial serial_port(PA_9, PA_10, 9600);
FileHandle *mbed::mbed_override_console(int fd)
{
    return &serial_port;
}


MPU6050 mpu(PB_7,PB_6);
//QMC5883L qmc(PB_7,PB_6);
//MS5837 ms(PB_7,PB_6);

Timer timer;

DigitalOut led1(PB_0);
DigitalOut led2(PB_2);
AnalogIn a0(PA_0);

PwmOut mOnSag(PB_15);
PwmOut mOnSol(PB_14);
PwmOut mArkaSag(PC_7);
PwmOut mArkaSol(PC_6);
PwmOut mUstArka(PA_4);
PwmOut mUstSag(PC_9);
PwmOut mUstSol(PC_8);

PID pitchPID (PID_PITCH_Kc, PID_PITCH_Ti, PID_PITCH_Td, 0.1);
PID rollPID (PID_ROLL_Kc, PID_ROLL_Ti, PID_ROLL_Td, 0.1);
PID yawPID (PID_YAW_Kc, PID_YAW_Ti, PID_YAW_Td, 0.1);

CAN can1(PB_8, PB_9, 500000);
Controller controller;

float pitchDiff;
float yawDiff;
float rollDiff;

float currTime;
float prevTime;
float timeDiff;

float accOffset[3];
float gyroOffset[3];
float angle[3];

bool isRobotActive = false;
void checkIsRobotActive() {
    if(controller.buttonA) {
        isRobotActive = true;
    }
    if(controller.buttonB) {
        isRobotActive = false;
    }
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
        led1 = false;
    }else {
        led1 = true;
    }
}

void AutonomousDrive(int pitchDiff, int yawDiff, int rollDiff) {
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

bool isAnalogStickMoved(int stick){
    return fitMotorValue(byteToMotorValue(stick)) != 1500;
}
 
void JoystickCheck(int yawAngle, int pitchAngle, int rollAngle){
    if(isAnalogStickMoved(controller.leftX)){
        yawPID.setSetPoint(yawAngle);
    }  
    // int trigger = (controller.leftTrigger - controller.rightTrigger)/6;
    // pitchPID.setSetPoint(trigger);
}

void ManuelDrive(int pitchDiff, int yawDiff, int rollDiff) {
    int mOnsag = (1500 - (yawDiff - 1500) - (byteToMotorValue(controller.rightY) - 1500) + (byteToMotorValue(controller.rightX) - 1500)  + (byteToMotorValue(controller.leftX) - 1500));
    int mOnsol = (1500 - (yawDiff - 1500) - (byteToMotorValue(controller.rightY) - 1500) - (byteToMotorValue(controller.rightX) - 1500)  - (byteToMotorValue(controller.leftX) - 1500));
    int mArkasag = (1500 - (yawDiff - 1500) + (byteToMotorValue(controller.rightY) - 1500) + (byteToMotorValue(controller.rightX) - 1500)  - (byteToMotorValue(controller.leftX) - 1500));
    int mArkasol = (1500 - (yawDiff - 1500) - (byteToMotorValue(controller.rightY) - 1500) + (byteToMotorValue(controller.rightX) - 1500)  - (byteToMotorValue(controller.leftX) - 1500));

    int mUstarka = (1500 +  (byteToMotorValue(controller.leftY) - 1500));
    int mUstsag = (1500 + (byteToMotorValue(controller.leftY) - 1500));
    int mUstsol = (1500 - (byteToMotorValue(controller.leftY) - 1500));

    mOnSag.pulsewidth_us(fitMotorValue(mOnsag));
    mOnSol.pulsewidth_us(fitMotorValue(mOnsol));
    mArkaSag.pulsewidth_us(fitMotorValue(mArkasag));
    mArkaSol.pulsewidth_us(fitMotorValue(mArkasol));

    mUstArka.pulsewidth_us(fitMotorValue(mUstarka));
    mUstSag.pulsewidth_us(fitMotorValue(mUstsag));
    mUstSol.pulsewidth_us(fitMotorValue(mUstsol));

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
    pitchPID.setInputLimits (-1 * PID_ROLL_PITCH_IN, PID_ROLL_PITCH_IN);  
    pitchPID.setOutputLimits (PID_OUT_MIN, PID_OUT_MAX);
 
    yawPID.setInputLimits (-1 * PID_YAW_IN, PID_YAW_IN);
    yawPID.setOutputLimits (PID_OUT_MIN, PID_OUT_MAX);

    rollPID.setInputLimits (-1 * PID_ROLL_PITCH_IN, PID_ROLL_PITCH_IN);
    rollPID.setOutputLimits (PID_OUT_MIN, PID_OUT_MAX);
    
    pitchPID.setBias(PID_BIAS);
    yawPID.setBias(PID_BIAS);
    rollPID.setBias(PID_BIAS);

    pitchPID.setMode(AUTO_MODE);  
    yawPID.setMode(AUTO_MODE); 
    rollPID.setMode(AUTO_MODE);
}
void initGyro(float *accOffset, float *gyroOffset) {
    mpu.setAlpha(0.97);
    mpu.getOffset(accOffset, gyroOffset, MPU_OFFSET_SAMPLES);
    wait_us(10000);
}
void resetRobot() {
    controller.Reset();
    pitchPID.reset();
    yawPID.reset();
    rollPID.reset();
    mOnSag.pulsewidth_us(1500);
    mOnSol.pulsewidth_us(1500);
    mArkaSag.pulsewidth_us(1500);
    mArkaSol.pulsewidth_us(1500);
    mUstArka.pulsewidth_us(1500);
    mUstSag.pulsewidth_us(1500);
    mUstSol.pulsewidth_us(1500);
}

int main()
{
    initMotors();
    CANMessage msg;

    initPID();
    initGyro(accOffset, gyroOffset);
    pitchPID.setSetPoint(0);
    yawPID.setSetPoint(0);
    rollPID.setSetPoint(0);

    timer.start();
    prevTime = chrono::duration<float>(timer.elapsed_time()).count();

    while (true) {
        int batteryVoltage = (10000+1000) / 1000 * (3.3 / 65535) * a0.read_u16() *10;
        led2 = batteryVoltage > 180;
         printf("analog: %d\n",batteryVoltage); 
        
        currTime = chrono::duration<float>(timer.elapsed_time()).count();
        timeDiff = currTime - prevTime;
        prevTime = currTime;

        if (can1.read(msg)) {
            controller.SetKeyValues(msg.id, msg.data);
            checkIsRobotActive();
            checkAutonomous(isRobotActive);
        }

        if(!isRobotActive) {
            resetRobot();
            led1 = !led1;
            continue;
        }

        mpu.computeAngle (angle, accOffset, gyroOffset, timeDiff);

        yawPID.setInterval(timeDiff);
        pitchPID.setInterval(timeDiff);
        rollPID.setInterval(timeDiff);

        int yawAngle = (int)angle[MPU_Z_AXIS] % PID_YAW_IN;
        int pitchAngle = (int)angle[MPU_Y_AXIS] > PID_ROLL_PITCH_IN ? PID_ROLL_PITCH_IN : angle[MPU_Y_AXIS];
        int rollAngle = (int)angle[MPU_X_AXIS] > PID_ROLL_PITCH_IN ? PID_ROLL_PITCH_IN : angle[MPU_X_AXIS];
        pitchAngle = (int)angle[MPU_Y_AXIS] < -1*PID_ROLL_PITCH_IN ? -1*PID_ROLL_PITCH_IN : angle[MPU_Y_AXIS];
        rollAngle = (int)angle[MPU_X_AXIS] < -1*PID_ROLL_PITCH_IN ? -1*PID_ROLL_PITCH_IN : angle[MPU_X_AXIS];

        yawPID.setProcessValue (yawAngle); 
        pitchPID.setProcessValue (pitchAngle);
        rollPID.setProcessValue (rollAngle);

        pitchDiff = pitchPID.compute();
        yawDiff = yawPID.compute();
        rollDiff = rollPID.compute();

        //printf("%d - %d\n",yawAngle, (int)yawDiff); 

        if(autonomousMod) {
            AutonomousDrive((int)pitchDiff, (int)yawDiff, (int)rollDiff);
        }
        else {
            ManuelDrive((int)pitchDiff, (int)yawDiff, (int)rollDiff);
            JoystickCheck(yawAngle, pitchAngle, rollAngle);
        }
        wait_us(100);
    }
}