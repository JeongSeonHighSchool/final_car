#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <Adafruit_VL6180X.h>

#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

#define SHT_LOX1 7
#define SHT_LOX2 6

#define GPIO_LOX1 4
#define GPIO_LOX2 3

#define TIMING_PIN 13
Adafruit_VL6180X lox1 = Adafruit_VL6180X();
Adafruit_VL6180X lox2 = Adafruit_VL6180X();
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

typedef enum {RUN_MODE_DEFAULT, RUN_MODE_TIMED, RUN_MODE_ASYNC, RUN_MODE_GPIO, RUN_MODE_CONT} runmode_t;

runmode_t run_mode = RUN_MODE_DEFAULT;
uint8_t show_command_list = 1;

Adafruit_VL6180X *sensors[] = {&lox1, &lox2};
const uint8_t COUNT_SENSORS = sizeof(sensors) / sizeof(sensors[0]);
const int sensor_gpios[COUNT_SENSORS] = {GPIO_LOX1, GPIO_LOX2};

uint8_t         sensor_ranges[COUNT_SENSORS];
uint8_t         sensor_status[COUNT_SENSORS];

const uint16_t  ALL_SENSORS_PENDING = ((1 << COUNT_SENSORS) - 1);
uint16_t        sensors_pending = ALL_SENSORS_PENDING;
uint32_t        sensor_last_cycle_time;

int motor_1 = 12;
int pwm_1 = 3;
int brake_1 = 9;
int motor_2 = 13;
int pwm_2 = 11;
int brake_2 = 8;

void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);

  delay(10);

  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);

  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);


  // initing LOX1
  if (!lox1.begin()) {
    Serial.println(F("Failed to boot first VL6180X"));
    while (1);
  }
  lox1.setAddress(LOX1_ADDRESS);
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if (!lox2.begin()) {
    Serial.println(F("Failed to boot second VL6180X"));
    while (1);
  }
  lox2.setAddress(LOX2_ADDRESS);


}
void readSensor(Adafruit_VL6180X &vl) {
  Serial.print(" Addr: ");
  Serial.print(vl.getAddress(), HEX);

  float lux = vl.readLux(VL6180X_ALS_GAIN_5);
  Serial.print(" Lux: "); Serial.print(lux);
  uint8_t range = vl.readRange();
  uint8_t status = vl.readRangeStatus();

  if (status == VL6180X_ERROR_NONE) {
    Serial.print(" Range: "); Serial.print(range);
  }

  // Some error occurred, print it out!

  if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
    Serial.print("(System error)");
  }
  else if (status == VL6180X_ERROR_ECEFAIL) {
    Serial.print("(ECE failure)");
  }
  else if (status == VL6180X_ERROR_NOCONVERGE) {
    Serial.print("(No convergence)");
  }
  else if (status == VL6180X_ERROR_RANGEIGNORE) {
    Serial.print("(Ignoring range)");
  }
  else if (status == VL6180X_ERROR_SNR) {
    Serial.print("Signal/Noise error");
  }
  else if (status == VL6180X_ERROR_RAWUFLOW) {
    Serial.print("Raw reading underflow");
  }
  else if (status == VL6180X_ERROR_RAWOFLOW) {
    Serial.print("Raw reading overflow");
  }
  else if (status == VL6180X_ERROR_RANGEUFLOW) {
    Serial.print("Range reading underflow");
  }
  else if (status == VL6180X_ERROR_RANGEOFLOW) {
    Serial.print("Range reading overflow");
  }
}
void read_sensors() {
  readSensor(lox1);
  readSensor(lox2);
  Serial.println();
}
void timed_read_sensors() {
  digitalWrite(TIMING_PIN, HIGH);
  uint32_t start_time = millis();
  uint8_t range_lox1 = lox1.readRange();
  uint8_t status_lox1 = lox1.readRangeStatus();
  uint8_t range_lox2 = lox2.readRange();
  uint8_t status_lox2 = lox2.readRangeStatus();
  uint32_t delta_time = millis() - start_time;
  digitalWrite(TIMING_PIN, LOW);
  Serial.print(delta_time, DEC);
  Serial.print(" : ");
  if (status_lox1 == VL6180X_ERROR_NONE) Serial.print(range_lox1, DEC);
  else Serial.print("###");
  Serial.print(" : ");
  if (status_lox2 == VL6180X_ERROR_NONE) Serial.print(range_lox2, DEC);
  else Serial.print("###");
  Serial.println();
}
void timed_async_read_gpio() {
#ifdef GPIO_LOX1
  uint8_t range_lox1 = 0;
  uint8_t status_lox1 = VL6180X_ERROR_NONE;
  uint8_t range_lox2 = 0;
  uint8_t status_lox2 = VL6180X_ERROR_NONE;


  digitalWrite(TIMING_PIN, HIGH);
  uint8_t pending_sensors = 0x7;
  uint32_t start_time = millis();

  // lets start up all three
  lox1.startRange();
  lox2.startRange();

  while (pending_sensors && ((millis() - start_time) < 1000)) {
    if ((pending_sensors & 0x1) && !digitalRead(GPIO_LOX1)) {
      range_lox1 = lox1.readRangeResult();
      status_lox1 = lox1.readRangeStatus();
      pending_sensors ^= 0x1;
    }
    if ((pending_sensors & 0x2) && !digitalRead(GPIO_LOX2)) {
      range_lox2 = lox2.readRangeResult();
      status_lox2 = lox2.readRangeStatus();
      pending_sensors ^= 0x2;
    }
  }
  uint32_t delta_time = millis() - start_time;
  digitalWrite(TIMING_PIN, LOW);
  Serial.print(delta_time, DEC);
  Serial.print("(");
  Serial.print(pending_sensors, DEC);
  Serial.print(") : ");
  if (status_lox1 == VL6180X_ERROR_NONE) Serial.print(range_lox1, DEC);
  else Serial.print("###");
  Serial.print(" : ");
  if (status_lox2 == VL6180X_ERROR_NONE) Serial.print(range_lox2, DEC);
  else Serial.print("###");

  Serial.println();
#endif
}
void start_continuous_range() {
  Serial.println("start Continuous range mode");
  for (uint8_t i = 0; i < COUNT_SENSORS; i++) {
    sensors[i]->startRangeContinuous();
  }
  sensors_pending = ALL_SENSORS_PENDING;
  sensor_last_cycle_time = millis();
}

void stop_continuous_range() {
  Serial.println("Stop Continuous range mode");
  for (uint8_t i = 0; i < COUNT_SENSORS; i++) {
    sensors[i]->stopRangeContinuous();
  }
  delay(100); // give time for it to complete.
}
void Process_continuous_range() {

  uint16_t mask = 1;
  for (uint8_t i = 0; i < COUNT_SENSORS; i++) {
    bool range_complete = false;
    if (sensors_pending & mask) {
      if (sensor_gpios[i] >= 0)
        range_complete = !digitalRead(sensor_gpios[i]);
      else
        range_complete = sensors[i]->isRangeComplete();
      if (range_complete) {
        sensor_ranges[i] = sensors[i]->readRangeResult();
        sensor_status[i] = sensors[i]->readRangeStatus();
        sensors_pending ^= mask;
      }
    }
    mask <<= 1; // setup to test next one
  }
  // See if we have all of our sensors read OK
  uint32_t delta_time = millis() - sensor_last_cycle_time;
  if (!sensors_pending || (delta_time > 1000)) {
    Serial.print(delta_time, DEC);
    Serial.print("(");
    Serial.print(sensors_pending, HEX);
    Serial.print(")");
    mask = 1;
    for (uint8_t i = 0; i < COUNT_SENSORS; i++) {
      Serial.print(" : ");
      if (sensors_pending & mask) Serial.print("TTT");  // show timeout in this one
      else if (sensor_status[i] == VL6180X_ERROR_NONE) Serial.print(sensor_ranges[i], DEC);
      else {
        Serial.print("#");
        Serial.print(sensor_status[i], DEC);
      }
    }
    // setup for next pass
    Serial.println();
    sensor_last_cycle_time = millis();
    sensors_pending = ALL_SENSORS_PENDING;
  }
}
void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(TIMING_PIN, OUTPUT);
#ifdef GPIO_LOX1
  // If we defined GPIO pins, enable them as PULL UP
  pinMode(GPIO_LOX1, INPUT_PULLUP);
  pinMode(GPIO_LOX2, INPUT_PULLUP);
#endif

  Serial.println("Shutdown pins inited...");

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(TIMING_PIN, LOW);
  Serial.println("All in reset mode...(pins are low)");


  Serial.println("Starting...");
  setID();

  pinMode(motor_1, OUTPUT);
  pinMode(pwm_1, OUTPUT);
  pinMode(brake_1, OUTPUT);
  pinMode(motor_2, OUTPUT);
  pinMode(pwm_2, OUTPUT);
  pinMode(brake_2, OUTPUT);
  Serial.println("Adafruit VL53L0X test");


  // put your setup code here, to run once:

}

void loop() {
  read_sensors();
    for(int i=0; i< COUNT_SENSORS; i++){
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(" :");
      Serial.print(sensor_ranges[i]);
      Serial.print("mm");
      Serial.println();
      delay(500);
    }
    // if(sensor_ranges[0] > 75){ 1
    //   //do something
    // }
  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }
    
  delay(100);
  // put your main code here, to run repeatedly:

}
