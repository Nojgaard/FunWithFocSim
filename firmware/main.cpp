#include <Arduino.h>
#include <SimpleFOC.h>

MagneticSensorI2C sensor(MT6701_I2C);

// BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(11, 10, 9, 8); // PWM pins for phases A, B, C and enable pin

void setup() {
  // monitoring port
  Serial.begin(115200);

  // configure i2C
  Wire.setClock(400000);
  // initialise magnetic sensor hardware
  sensor.init();

  Serial.println("Sensor ready");
  _delay(1000);
}

void loop() {
  // iterative function updating the sensor internal variables
  // it is usually called in motor.loopFOC()
  // this function reads the sensor hardware and 
  // has to be called before getAngle nad getVelocity
  sensor.update();
  
  // display the angle and the angular velocity to the terminal
  Serial.print(sensor.getAngle());
  Serial.print("\t");
  Serial.println(sensor.getVelocity());
}

/*


void setup() { 
  
  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);
  
  Serial.println("init sensor!");
  // initialize encoder sensor hardware
  sensor.init();

  Serial.println("link motor!");
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.voltage_limit = 6;
  // driver init
  if(!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }
  // link driver
  Serial.println("link driver!");
  motor.linkDriver(&driver);

  // aligning voltage
  //motor.voltage_sensor_align = 3;

  // set motion control loop to be used
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;

  // comment out if not needed
  Serial.println("monitor!");
  motor.useMonitoring(Serial);

  Serial.println("init motor!");
  // initialize motor
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }
  // align sensor and start FOC
  if(!motor.initFOC()){
    Serial.println("FOC init failed!");
    return;
  }

  // set the initial motor target
  motor.target = 2; // Volts 

  // add target command M
  // command.add('M', doMotor, "Motor");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target using serial terminal and command M:"));
  _delay(1000);
}

void loop() {
  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move();

  // user communication
  // command.run();
}*/