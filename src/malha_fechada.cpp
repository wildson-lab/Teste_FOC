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


MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1  = BLDCDriver3PWM(M1_A, M1_B, M1_C);
  
// // BLDC motor & driver instance
// BLDCMotor motor2 = BLDCMotor(7);
// BLDCDriver3PWM driver2  = BLDCDriver3PWM(M2_A, M2_B, M2_C);

// BLDCMotor motor3 = BLDCMotor(7);
// BLDCDriver3PWM driver3  = BLDCDriver3PWM(M3_A, M3_B, M3_C);

// BLDCMotor motor4 = BLDCMotor(7);
// BLDCDriver3PWM driver4  = BLDCDriver3PWM(M4_A, M4_B, M4_C);


float target_velocity = 0.0;

Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

void setup() {
  sensor.init();
  motor1.linkSensor(&sensor);
  driver1.voltage_power_supply = 9;
  driver1.init();
  motor1.linkDriver(&driver1);

  // set motion control loop to be used
  motor1.controller = MotionControlType::velocity;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor1.PID_velocity.P = 0.2f;
  motor1.PID_velocity.I = 20;
  motor1.PID_velocity.D = 0;

  // default voltage_power_supply
  motor1.voltage_limit = 6;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor1.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best.
  // the lower the less filtered
  motor1.LPF_velocity.Tf = 0.01f;

  // use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  motor1.useMonitoring(Serial);

  // initialize motor
  motor1.init();
  // align sensor and start FOC
  motor1.initFOC();

//   add target command T
  command.add('T', doTarget, "target velocity");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
  _delay(1000);
}

void loop() {
  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor1.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor1.move(target_velocity);
//   motor2.move(0);
//   motor3.move(0);
//   motor4.move(0);

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor1.monitor();

  // user communication
  command.run();
}
