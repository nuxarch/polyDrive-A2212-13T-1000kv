#include "polyDrive.h"

// void polyDrive::doA() { sensor.handleA(); }
// void polyDrive::doB() { sensor.handleB(); }
// void polyDrive::doC() { sensor.handleC(); }

// void doA(){ sensor.handleA(); }
// void doB(){ sensor.handleB(); }
// void doC(){ sensor.handleC(); }

void polyDrive::init_hall()
{
    // _delay(100);
    // sensor.pullup = Pullup::USE_EXTERN;
    // sensor.init();
    // sensor.enableInterrupts(doA, doB, doC);
    // Serial.println("Sensor readdoAy");
    // _delay(500);
}

void polyDrive::vel_PID(float _kP, float _kI, float _kD, int _output_ramp, float _limit, float _lpf_velocity)
{
    // 0.1, 0.0, 0.0, 10000, 5, 0.1
    motor.PID_velocity.P = _kP;
    motor.PID_velocity.I = _kI;
    motor.PID_velocity.D = _kD;
    motor.PID_velocity.output_ramp = _output_ramp;
    motor.PID_velocity.limit = _limit;
    motor.LPF_velocity = _lpf_velocity;
}
void polyDrive::angle_PID(float _kP, float _kI, float _kD, int _output_ramp, float _limit, float _lpf_angle)
{
    // 2.5, 0.1, 0, 1000, 5, 5
    motor.P_angle.P = _kP;
    motor.P_angle.I = _kI;
    motor.P_angle.D = _kD;
    motor.P_angle.output_ramp = _output_ramp;
    motor.P_angle.limit = _limit;
    motor.LPF_angle = _lpf_angle;
}

void polyDrive::initBoard(float supply_power)
{
    Serial.print("initiate controller board");
    driver.voltage_power_supply = supply_power;
    driver.pwm_frequency = 15000;
    driver.init();
    // gate_driver.begin(PWM_INPUT_MODE_3PWM);
    motor.linkDriver(&driver);
    // motor.linkSensor(&sensor);
}
void polyDrive::initDRV8313(){
    pinMode(FAULT_PIN, INPUT_PULLUP);
    pinMode(RESET_PIN,OUTPUT);
    pinMode(SLEEP_PIN, OUTPUT);
    digitalWrite(RESET_PIN,HIGH);
    digitalWrite(SLEEP_PIN,HIGH);
}

void polyDrive::limits(float _phase_resistance, float _vel_limit, float _voltage_limit, float _current_limit)
{
    motor.phase_resistance = _phase_resistance; // 0.63
    motor.velocity_limit = _vel_limit;          // 100.0;
    motor.voltage_limit = _voltage_limit;
    motor.current_limit = _current_limit; // 5;
    motor.voltage_sensor_align = 2;
    motor.velocity_index_search = 3;
}

void polyDrive::clear_error(char *cmd)
{
    // gate_driver.begin(PWM_INPUT_MODE_3PWM);
    delay(1000);
    // Serial.println("Is error " + String(gate_driver.get_PWM_Mode()));
    // Serial.println("error :" + String(gate_driver.read_fault()));
    Serial.println("resetting eeror");
    delay(1000);
    // gate_driver.reset();
    delay(1000);
    // Serial.println("Is error " + String(gate_driver.get_PWM_Mode()));
    // Serial.println("error :" + String(gate_driver.read_fault()));
    delay(1000);
}
void polyDrive::vel_mode(char *cmd)
{
    motor.controller = MotionControlType::velocity;
}
void polyDrive::pos_mode(char *cmd)
{
    motor.controller = MotionControlType::angle;
}

void polyDrive::taskProtection()
{
    vTaskDelay(100 / portTICK_PERIOD_MS);
    Serial.println("protection task started");
    String  temp;
    for (;;)
    {
        board_temp = map(analogRead(TEMPERATURE_PIN), 0, 4035, 0, 100);
        if ((board_temp > 26) || (digitalRead(FAULT_PIN) == LOW)){
            driver.disable();
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            temp= "nFault / OverTemp : [" + String(board_temp) + "]\n\r";
            Serial.print(temp);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
void polyDrive::taskSet()
{
    while (1)
    {
        if (motor.controller == MotionControlType::angle)
        {
            vTaskDelay(delay_test / portTICK_PERIOD_MS);
            target_velocity = 1.571;
            vTaskDelay(delay_test / portTICK_PERIOD_MS);
            target_velocity = 3.142;
            vTaskDelay(delay_test / portTICK_PERIOD_MS);
            target_velocity = 4.712;
            vTaskDelay(delay_test / portTICK_PERIOD_MS);
            target_velocity = 6.283;
            vTaskDelay(delay_test / portTICK_PERIOD_MS);
            target_velocity = 0;
        }
        if (motor.controller == MotionControlType::velocity)
        {
            vTaskDelay(delay_test / portTICK_PERIOD_MS);
            target_velocity = 10;
            vTaskDelay(delay_test / portTICK_PERIOD_MS);
            target_velocity = 20;
            vTaskDelay(delay_test / portTICK_PERIOD_MS);
            target_velocity = 30;
            vTaskDelay(delay_test / portTICK_PERIOD_MS);
            target_velocity = 40;
            vTaskDelay(delay_test / portTICK_PERIOD_MS);
            target_velocity = 30;
            vTaskDelay(delay_test / portTICK_PERIOD_MS);
            target_velocity = 20;
            vTaskDelay(delay_test / portTICK_PERIOD_MS);
            target_velocity = 10;
            vTaskDelay(delay_test / portTICK_PERIOD_MS);
            target_velocity = 0;

        }
    }
}

void polyDrive::run()
{
    motor.move(target_velocity);
    motor.monitor();
    motor.loopFOC();
    command.run();
}
polyDrive::polyDrive()
{
    // Serial.begin(115200);
    // initBoard(36);
    // vel_PID(0.1, 0.0, 0.0, 10000, 5, 0.1);
    // angle_PID(2.5, 0.1, 0, 1000, 5, 5);
    // limits(0.63, 100, 5);
    // motor.useMonitoring(Serial);
    // motor.init();
    // motor.PID_velocity.limit = 5;
    // // motor.initFOC();
    // Serial.println("initiate FOC");
    // motor.initFOC(zero_electric_offset, _sensor_direction);
    // motor.controller = motion;
    // // sensor.enableInterrupts(doA, doB, doC);
}

polyDrive::~polyDrive()
{
}
