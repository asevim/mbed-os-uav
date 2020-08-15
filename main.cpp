#include "mbed.h"
#include "MPU6050.h"
#include "QMC5883L.h"
#include "controller.h"

#define TARGET_TX_PIN      PA_9
#define TARGET_RX_PIN      PA_10

static BufferedSerial serial_port(TARGET_TX_PIN, TARGET_RX_PIN, 9600);
FileHandle *mbed::mbed_override_console(int fd)
{
    return &serial_port;
}

MPU6050 mpu(PB_7,PB_6);
QMC5883L qmc(PB_7,PB_6);

DigitalOut led1(PB_0);
DigitalOut led2(PB_2);

PwmOut mOnSag(PB_15);
PwmOut mOnSol(PB_14);
PwmOut mArkaSag(PC_6);
PwmOut mArkaSol(PC_7);
PwmOut mUstArka(PA_4);
PwmOut mUstSag(PC_9);
PwmOut mUstSol(PC_8);

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

void AutonomousDrive() {
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

void ManuelDrive() {
    int mOnsag = 1500 - (byteToMotorValue(controller.rightY) - 1500) + (byteToMotorValue(controller.rightX) - 1500)  + (byteToMotorValue(controller.leftX) - 1500);
    int mOnsol = 1500 - (byteToMotorValue(controller.rightY) - 1500) - (byteToMotorValue(controller.rightX) - 1500)  - (byteToMotorValue(controller.leftX) - 1500);
    int mArkasag = 1500 + (byteToMotorValue(controller.rightY) - 1500) + (byteToMotorValue(controller.rightX) - 1500)  + (byteToMotorValue(controller.leftX) - 1500);
    int mArkasol = 1500 - (byteToMotorValue(controller.rightY) - 1500) + (byteToMotorValue(controller.rightX) - 1500)  + (byteToMotorValue(controller.leftX) - 1500);

    mOnSag.pulsewidth_us(fitMotorValue(mOnsag));
    mOnSol.pulsewidth_us(fitMotorValue(mOnsol));
    mArkaSag.pulsewidth_us(fitMotorValue(mArkasag,true));
    mArkaSol.pulsewidth_us(fitMotorValue(mArkasol));
    
    mUstArka.pulsewidth_us(fitMotorValue(byteToMotorValue(controller.leftY)));
    mUstSag.pulsewidth_us(fitMotorValue(byteToMotorValue(controller.leftY), true));
    mUstSol.pulsewidth_us(fitMotorValue(byteToMotorValue(controller.leftY)));
/* 
    printf("RX: %d", controller.rightX);
    printf("RY: %d", controller.rightY);
    printf("LX: %d", controller.leftX);    
    printf("on sag: %d", mOnsag);
    
    printf("on sol: %d", mOnsol);
    
    printf("arka sag: %d",mArkasag);
    
    printf("arka sol: %d\n", mArkasol);
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

int main()
{
    initMotors();
    CANMessage msg;
    while (true) {
        if (can1.read(msg)) {
            controller.SetKeyValues(msg.id, msg.data);
            checkIsRobotActive();
            checkAutonomous(isRobotActive);

            //printf("%d\n", msg.data[0]);
        }
        if(!isRobotActive) {
            resetRobot();
            bool ledVal = !led1;
            led1 = ledVal;
            led2 = ledVal;
            wait_us(50000);
            
        }
        if(autonomousMod) {
            AutonomousDrive();
        }
        else {
            ManuelDrive();
        }
    }
}