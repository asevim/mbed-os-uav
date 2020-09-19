#include "mbed.h"
#include "MPU6050.h"
#include "controller.h"
#include "PID.h"
// #include "MS5837.h"

#define PID_ROLL_Kc 3//0.48
#define PID_ROLL_Ti 0//0.5
#define PID_ROLL_Td 0//0.125

#define PID_PITCH_Kc 3//0.48
#define PID_PITCH_Ti 0//0.5
#define PID_PITCH_Td 0//0.125s
#define PID_ROLL_PITCH_IN 360.0

#define PID_YAW_Kc 3//0.48
#define PID_YAW_Ti 0//0.25
#define PID_YAW_Td 0//0.125
#define PID_YAW_IN 360.0

#define PID_OUT_MIN 1000.0
#define PID_OUT_MAX 2000.0
#define PID_BIAS 1500.0

#define MPU_OFFSET_SAMPLES 50

#define MPU_X_AXIS 2
#define MPU_Y_AXIS 0
#define MPU_Z_AXIS 1

static BufferedSerial serial_port(PA_9, PA_10);
FileHandle *mbed::mbed_override_console(int fd)
{
    return &serial_port;
}

MPU6050 mpu(PB_7,PB_6);

Timer timer;

DigitalOut led1(PB_0);
DigitalOut led2(PB_2);
DigitalOut Gripp(PA_1);
AnalogIn voltageAnalogPin(PA_0);

PwmOut mOnSag(PB_15);
PwmOut mOnSol(PB_14);
PwmOut mArkaSag(PC_7);
PwmOut mArkaSol(PC_6);
PwmOut mUstArka(PA_4);
PwmOut mUstSag(PC_8);
PwmOut mUstSol(PC_9);

PID pitchPID (PID_PITCH_Kc, PID_PITCH_Ti, PID_PITCH_Td, 0.1);
PID rollPID (PID_ROLL_Kc, PID_ROLL_Ti, PID_ROLL_Td, 0.1);
PID yawPID (PID_YAW_Kc, PID_YAW_Ti, PID_YAW_Td, 0.1);

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

float yawAngleOffset;
float lastMovedTime = 0;

void JoystickCheck(float yawAngle, float pitchAngle, float currentTime){
    if(isAnalogStickMoved(controller.leftX)){
        lastMovedTime = currentTime;
    }
    if(currentTime - lastMovedTime < 0.5) {
        yawAngleOffset = (-1) * yawAngle;
    }

}
void Drive(int RightY, int RightX, int LeftY, int LeftX, int pitchDiff, int yawDiff, int rollDiff){
    int ry = byteToMotorValue(RightY);
    int rx = byteToMotorValue(RightX);
    int ly = byteToMotorValue(LeftY);
    int lx = byteToMotorValue(LeftX);
    int leftTrigger = byteToMotorValue(controller.leftTrigger);
    int rightTrigger = byteToMotorValue(controller.rightTrigger);

    int mOnsag = (1500 + (yawDiff - 1500) - (ry - 1500) + (rx - 1500)  + (lx - 1500));
    int mOnsol = (1500 - (yawDiff - 1500) - (ry - 1500) - (rx - 1500)  - (lx - 1500));
    int mArkasag = (1500 - (yawDiff - 1500) + (ry - 1500) + (rx - 1500)  - (lx - 1500));
    int mArkasol = (1500 - (yawDiff - 1500) - (ry - 1500) + (rx - 1500)  - (lx - 1500));

    int ustPitch = pitchDiff;
    int ustRoll = 1500;

    if(controller.leftTrigger != 0 || controller.rightTrigger != 0){
        ustPitch = 1500;
        ustRoll = 1500;
    }

    int mUstarka = (1500 + (ustPitch - 1500) + (ly - 1500) + (leftTrigger/4 - rightTrigger/4));
    int mUstsag = (1500  - (ustPitch-1500) + (ly - 1500) + (ustRoll - 1500) - (leftTrigger/4 - rightTrigger/4));
    int mUstsol = (1500 + (ustPitch - 1500) - (ly - 1500) - (ustRoll - 1500) + (leftTrigger/4 - rightTrigger/4));

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

    pitchPID.setSetPoint(0);
    yawPID.setSetPoint(0);
    rollPID.setSetPoint(0);
}
void initGyro(float *accOffset, float *gyroOffset) {
    mpu.setAlpha(0.97);
    mpu.getOffset(accOffset, gyroOffset, MPU_OFFSET_SAMPLES);
    wait_us(1000000);
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

void FlushSerial() {
    char *temp;
    while(serial_port.readable()) {
        serial_port.read(temp, 1);
    }
}

int main()
{
    yawAngleOffset = 0.0;

    int pitchDiff = 0;
    int yawDiff = 0;
    int rollDiff = 0;

    float currTime = 0.0;
    float prevTime = 0.0;
    float timeDiff = 0.0;

    float accOffset[3];
    float gyroOffset[3];
    float angle[3];

    float yawAngle = 0.0;
    float pitchAngle = 0.0;
    float rollAngle = 0.0;

    Gripp = false;

    initGyro(accOffset, gyroOffset);
    // ms.MS5837Init();
    initMotors();
    initPID();
    CANMessage msg;

    serial_port.set_baud(460800);

    timer.start();
    prevTime = chrono::duration<float>(timer.elapsed_time()).count();
    // int BarometerPrevTime = prevTime;

    while (true) {
        int batteryVoltage = (363.0 / 65535.0) * voltageAnalogPin.read_u16();
        led2 = batteryVoltage > 185;
        //printf("batteryVoltage: %d\n",batteryVoltage);
        currTime = chrono::duration<float>(timer.elapsed_time()).count();
        timeDiff = currTime - prevTime;
        prevTime = currTime;
        mpu.computeAngle (angle, accOffset, gyroOffset, timeDiff);

        yawAngle = angle[MPU_Z_AXIS];
        pitchAngle = angle[MPU_Y_AXIS];
        rollAngle = angle[MPU_X_AXIS];

        FlushSerial();

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

        yawPID.setInterval(timeDiff);
        pitchPID.setInterval(timeDiff);
        rollPID.setInterval(timeDiff);
        yawPID.setProcessValue (yawAngle + yawAngleOffset);
        pitchPID.setProcessValue (pitchAngle);
        rollPID.setProcessValue (rollAngle);

        pitchDiff = (int)pitchPID.compute();
        yawDiff = (int)yawPID.compute();
        rollDiff = (int)rollPID.compute();

        // BarometerTimeDiff = currTime - BarometerPrevTime;

        // if(BarometerTimeDiff > 1000000){
        //     //ms.Barometer_MS5837();
        //     BarometerPrevTime = currTime;
        // }
        // int pressure = (int)ms.MS5837_Pressure();

        //printf("yaw = %d, pitch = %d, roll = %d\n", (int)yawAngle, (int)pitchAngle, (int)rollAngle);

        if(autonomousMod) {
            Gripp = false;

            char c[3];

            while(serial_port.readable()) {
                serial_port.read(c, 1);
                if(c[0] != 60) {
                    continue;
                }
                serial_port.read(c, 1);
                if(c[0] != 61) {
                    continue;
                }
                serial_port.read(c, 3);
                controller.SetAutonomousValues(int(c[0]), int(c[1]), int(c[2]));
            }
            Drive(controller.autonomous_rightY,controller.autonomous_rightX,controller.autonomous_leftY,127,(int)pitchDiff, (int)yawDiff, (int)rollDiff);
        }
        else {
            //manual
            if(controller.rightBumper) Gripp = true;
            if(controller.leftBumper) Gripp = false;

            Drive(controller.rightY,controller.rightX,controller.leftY,controller.leftX,(int)pitchDiff, (int)yawDiff, (int)rollDiff);
            JoystickCheck(yawAngle, pitchAngle, currTime);
        }
    }
}
