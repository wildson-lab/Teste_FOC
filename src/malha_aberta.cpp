

// MKS DUAL FOC open-loop speed control routine.Test library: SimpleFOC 2.1.1 Test hardware: MKS DUAL FOC V3.1
// Enter "T+number" in the serial port to set the speed of the two motors. For example, set the motor to rotate at 10rad/s, input "T10", and the motor will rotate at 5rad/s by default when it is powered on
// When using your own motor, please remember to modify the default number of pole pairs, that is, the value in BLDCMotor(7), and set it to your own number of pole pairs
// The default power supply voltage set by the program is 12V, please remember to modify the values in voltage_power_supply and voltage_limit variables if you use other voltages for power supply

#include <Arduino.h>
#include <SimpleFOC.h>

// STM32:
#ifdef STM32_BLUEPILL

  #define M1_A PB8
  #define M1_B PB9
  #define M1_C PA0

  #define M2_A PA8
  #define M2_B PA9
  #define M2_C PA10

  #define M3_A PA1
  #define M3_B PA2
  #define M3_C PA3

  #define M4_A PA6
  #define M4_B PA7
  #define M4_C PB0
#endif

// Arduino Mega:
#ifdef ARDUINO_MEGA
  #define M1_A 2
  #define M1_B 3
  #define M1_C 4

  #define M2_A 5
  #define M2_B 6
  #define M2_C 7

  #define M3_A 8
  #define M3_B 9
  #define M3_C 10

  #define M4_A 11
  #define M4_B 12
  #define M4_C 13
#endif


BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1  = BLDCDriver3PWM(M1_A, M1_B, M1_C);
  
// BLDC motor & driver instance
BLDCMotor motor2 = BLDCMotor(7);
BLDCDriver3PWM driver2  = BLDCDriver3PWM(M2_A, M2_B, M2_C);

BLDCMotor motor3 = BLDCMotor(7);
BLDCDriver3PWM driver3  = BLDCDriver3PWM(M3_A, M3_B, M3_C);

BLDCMotor motor4 = BLDCMotor(7);
BLDCDriver3PWM driver4  = BLDCDriver3PWM(M4_A, M4_B, M4_C);

//Target variable
float target_velocity = 5;
float r_shunt = 0.1;
float current_gain = 50.0;

//Serial command setting
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }


void setup() {
  driver1.voltage_power_supply = 12;
  driver1.init();
  motor1.linkDriver(&driver1);
  motor1.voltage_limit = 3;   // [V]
  motor1.velocity_limit = 40; // [rad/s]
  
  driver2.voltage_power_supply = 12;
  driver2.init();
  motor2.linkDriver(&driver2);
  motor2.voltage_limit = 3;   // [V]
  motor2.velocity_limit = 40; // [rad/s]
  
  driver3.voltage_power_supply = 12;
  driver3.init();
  motor3.linkDriver(&driver3);
  motor3.voltage_limit = 3;   // [V]
  motor3.velocity_limit = 40; // [rad/s]
  
  driver4.voltage_power_supply = 12;
  driver4.init();
  motor4.linkDriver(&driver4);
  motor4.voltage_limit = 3;   // [V]
  motor4.velocity_limit = 40; // [rad/s]


  //Open loop control mode setting
  motor1.controller = MotionControlType::velocity_openloop;
  motor2.controller = MotionControlType::velocity_openloop;
  motor3.controller = MotionControlType::velocity_openloop;
  motor4.controller = MotionControlType::velocity_openloop;

  //Initialize the hardware
  motor1.init();
  motor2.init();
  motor3.init();
  motor4.init();


  //Add T command
  command.add('T', doTarget, "target velocity");

  Serial.begin(9600);
  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}

void loop() {
  float vel = 30.0;

  motor1.move(vel);
  motor2.move(vel);
  motor3.move(vel);
  motor4.move(vel);

  //User newsletter
  command.run();
}