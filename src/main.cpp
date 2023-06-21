// BLDC driver standalone example
#include <SimpleFOC.h>


// BLDC driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(25, 26, 27,13);

void setup() {
  
  pinMode(4, INPUT_PULLUP);
  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);
  digitalWrite(14, HIGH);
  digitalWrite(15, HIGH);
  // pwm frequency to be used [Hz]
  // for atmega328 fixed to 32kHz
  // esp32/stm32/teensy configurable
  driver.pwm_frequency = 50000;
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 12;

  // driver init
  driver.init();

  // enable driver
  driver.enable();

  _delay(1000);
  
  Serial.begin(115200);;
  Serial.println("test");
}

void loop() {
    // setting pwm
    // phase A: 3V
    // phase B: 6V
    // phase C: 5V
    driver.setPwm(3,6,12);
}