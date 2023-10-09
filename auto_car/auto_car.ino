#include <Wire.h>
#include <VL6180X.h>
#include "Adafruit_VL53L0X.h"

VL6180X sensor;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
int motor_1 = 12;
int pwm_1 = 3;
int brake_1 = 9;
int motor_2 = 13;
int pwm_2 = 11;
int brake_2 = 8;
void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(motor_1, OUTPUT);
  pinMode(pwm_1, OUTPUT);
  pinMode(brake_1, OUTPUT);
  pinMode(motor_2, OUTPUT);
  pinMode(pwm_2, OUTPUT);
  pinMode(brake_2, OUTPUT);
  Serial.println("Adafruit VL53L0X test");
  sensor.init();
  sensor.configureDefault();
  sensor.setTimeout(500);

  // put your setup code here, to run once:

}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  Serial.print(sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  
    
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }
    
  delay(100);
  Serial.println();
  // put your main code here, to run repeatedly:

}
