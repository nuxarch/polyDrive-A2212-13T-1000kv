#ifndef __POLYDRIVE_H__
#define __POLYDRIVE_H__

#include <SimpleFOC.h>
#include "DRV8301.h"
#define THROTTLE_PIN 33
#define TEMPERATURE_PIN 39

// DRV8313 definition
#define FAULT_PIN   4
#define RESET_PIN   14
#define SLEEP_PIN   15

class polyDrive
{
public:
    /**
     *
     * polyDrive class constructor
     * @param MotionControlType
     * @param zero_electric_offset
     * @param Direction DRV8301 SPI clock pin
     */
    polyDrive();
    ~polyDrive();

    
    // BLDCMotor motor = BLDCMotor(7);
    // BLDCDriver3PWM driver = BLDCDriver3PWM(33, 25, 26);
    // DRV8301 gate_driver = DRV8301(23, 19, 18, 5, 27, 14);


    // BLDC motor & driver instance
    BLDCMotor motor = BLDCMotor(7,0.1);
    // BLDCDriver3PWM driver = BLDCDriver3PWM(26, 33, 32,25);
    BLDCDriver3PWM driver = BLDCDriver3PWM(25, 26, 27,13);

    Commander command = Commander(Serial);

    int angle_max = 3.14 * 20;
    float board_temp;
    String INFO;
    float target_velocity;
    int delay_test = 3000;

    // void doA(); // { sensor.handleA(); }
    // void doB(); // { sensor.handleB(); }
    // void doC(); // { sensor.handleC(); }

    void init_hall();
    void initBoard(float supply_power);
    void initDRV8313();
    void vel_PID(float _kP, float _kI, float _kD, int _output_ramp, float _limit, float _lpf_velocity);
    void angle_PID(float _kP, float _kI, float _kD, int _output_ramp, float _limit, float _lpf_angle);
    void limits(float _phase_resistance, float _vel_limit, float _voltage_limit, float _current_limit);
    void clear_error(char *cmd);
    void vel_mode(char *cmd);
    void pos_mode(char *cmd);
    void taskSet();
    void taskProtection();
    void run();
};


#endif