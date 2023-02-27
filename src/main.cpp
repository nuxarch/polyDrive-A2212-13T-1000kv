#include <Arduino.h>
#include <SimpleFOC.h>
#include "DRV8301.h"
#include "polyDrive.h"

Encoder sensor = Encoder(34, 32, 2048);
void doA(void) { sensor.handleA(); }
void doB(void) { sensor.handleB(); }

xTaskHandle taskThrottleHandle;
xTaskHandle taskTestSetPointHandle;
xTaskHandle taskProtectionHandle;

polyDrive myboard;
void onMotor(char *cmd) { myboard.command.motor(&myboard.motor, cmd); }
void doTarget(char *cmd) { myboard.command.scalar(&myboard.target_velocity, cmd); }

void taskTestSetPoint(void *parameter)
{
    myboard.taskSet();
}

void setControl(char *cmd)
{
    xTaskCreate(taskTestSetPoint, "task manual", 4000, NULL, 1, &taskTestSetPointHandle);
}
void setControlOff(char *cmd)
{
    vTaskDelete(taskTestSetPointHandle);
}

void setup()
{
    Serial.begin(115200);
    myboard.initBoard(36);
    sensor.quadrature = Quadrature::OFF;
    sensor.pullup = Pullup::USE_EXTERN;
    sensor.enableInterrupts(doA, doB);

    myboard.limits(3, 10, 1, 0.5);
    myboard.motor.linkSensor(&sensor);
    myboard.vel_PID(0.1, 0.3, 0.001, 25, 1, 0.5);
    myboard.motor.useMonitoring(Serial);
    
    // myboard.motor.controller = MotionControlType::velocity_openloop;
    // myboard.motor.controller = MotionControlType::angle_openloop;
    // myboard.motor.controller = MotionControlType::torque;
    myboard.motor.controller = MotionControlType::velocity;
    // myboard.motor.controller = MotionControlType::angle;
    myboard.motor.init();
    myboard.motor.initFOC();
    myboard.command.add('T', doTarget, "target velocity");
    myboard.command.add('M', onMotor, "on motor command");
    myboard.command.add('S', setControl, "manual test point");
    myboard.command.add('s', setControlOff, "manual test point off");
}

void loop()
{
    myboard.run();
}