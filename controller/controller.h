#ifndef CONTROLLER_H
#define CONTROLLER_H

class Controller 
{
public:
    Controller();
    void SetKeyValues(unsigned int id, unsigned char *data);
    void Reset();

public:
    int leftX;
    int leftY;
    int rightX;
    int rightY;
    int leftTrigger;
    int rightTrigger;
    bool buttonA;
    bool buttonB;
    bool buttonX;
    bool buttonY;
    bool dpadUp;
    bool dpadDown;
    bool dpadLeft;
    bool dpadRight;
    bool leftBumper;
    bool rightBumper;
};

#endif // CONTROLLER_H