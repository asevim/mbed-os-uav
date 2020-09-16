#include "controller.h"

Controller::Controller() {
    this->Reset();
}

void Controller::SetKeyValues(unsigned int id, unsigned char *data) {
    if(id == 61) {
        leftX = data[0];
        leftY = data[1];
        rightX = data[2];
        rightY = data[3];
        buttonA = (data[4] == 255)?true: false;
        buttonB = (data[5] == 255)?true: false;
        buttonX = (data[6] == 255)?true: false;
        buttonY = (data[7] == 255)?true: false;
    }
    else if(id == 62) {
        dpadUp = (data[0] == 255)?true: false;
        dpadDown = (data[1] == 255)?true: false;
        dpadLeft = (data[2] == 255)?true: false;
        dpadRight = (data[3] == 255)?true: false;
        leftTrigger = data[4];
        rightTrigger = data[5];
        leftBumper = (data[6] == 255)?true: false;
        rightBumper = (data[7] == 255)?true: false;
    }
}

void Controller::SetAutonomousValues(int autonomous_leftY, int autonomous_rightY, int autonomous_rightX) {
    if(autonomous_leftY > 255 || autonomous_leftY < 0) {
        autonomous_leftY = this->autonomous_leftY;
    }
    if(autonomous_rightY > 255 || autonomous_rightY < 0) {
        autonomous_rightY = this->autonomous_rightY;
    }
    if(autonomous_rightX > 255 || autonomous_rightX < 0) {
        autonomous_rightX = this->autonomous_rightX;
    }
    this->autonomous_leftY = autonomous_leftY;
    this->autonomous_rightY = autonomous_rightY;;
    this->autonomous_rightX = autonomous_rightX;;
}

void Controller::Reset() {
    leftX = 127;
    leftY = 127;
    rightX = 127;
    rightY = 127;
    buttonA = false;
    buttonB = false;
    buttonX = false;
    buttonY = false;
    dpadUp = false;
    dpadDown = false;
    dpadLeft = false;
    dpadRight = false;
    leftTrigger = 0;
    rightTrigger = 0;
    leftBumper = false;
    rightBumper = false;
    autonomous_leftY = 127;
    autonomous_rightY = 127;
    autonomous_rightX = 127;
}