#include <SimpleFOC.h>
#include "DRV8301.h"
#include "polyDrive.h"

// MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, 5);

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
void taskReadTemp(void *parameter)
{
    myboard.taskProtection();
}
void readTemp(char *cmd)
{
    // myboard.taskProtection();
    xTaskCreate(taskReadTemp, "task read temp", 4000, NULL, 1, &taskProtectionHandle);
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
    
    myboard.initBoard(24);
    myboard.initDRV8313();
    // sensor.quadrature = Quadrature::OFF;
    // sensor.pullup = Pullup::USE_EXTERN;
    // sensor.enableInterrupts(doA, doB);

    // configure i2C
    Wire.setClock(400000);
    // initialise magnetic sensor hardware
    sensor.init();

    myboard.limits(2, 5, 12, 2);
    myboard.motor.linkSensor(&sensor);
    // for A2212
    myboard.vel_PID(0.1, 0.01, 0.001, 100, 20, 1);

    myboard.angle_PID(15,0,0,50,5,0.01);
    myboard.motor.useMonitoring(Serial);

    // myboard.motor.controller = MotionControlType::velocity_openloop;
    // myboard.motor.controller = MotionControlType::angle_openloop;
    // myboard.motor.controller = MotionControlType::torque;
    // myboard.motor.controller = MotionControlType::velocity;
    myboard.motor.controller = MotionControlType::angle;
    myboard.motor.init();
    int i = 0;
    // while (myboard.motor.initFOC(5.66,CCW) != 1){
    while (myboard.motor.initFOC() != 1){
        delay(1000);
        Serial.println("trying "+String(++i));
        digitalWrite(RESET_PIN, LOW);delay(500);
        digitalWrite(RESET_PIN, HIGH);
        myboard.motor.init();
        if (i >=3)
        {
            Serial.println("please restart");
            while(1);
        }
        
    }
    myboard.command.add('T', doTarget, "target velocity");
    myboard.command.add('M', onMotor, "on motor command");
    myboard.command.add('S', setControl, "manual test point");
    myboard.command.add('s', setControlOff, "manual test point off");
    myboard.command.add('r', readTemp, "monitor temperature safety");
    xTaskCreate(taskReadTemp, "task read temp", 4000, NULL, 1, &taskProtectionHandle);
    
}

void loop()
{
    myboard.run();
}