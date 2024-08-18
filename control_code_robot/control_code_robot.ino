#include <CANSAME5x.h>

// Configure CAN
CANSAME5x CAN;
uint8_t angle = 90;
uint8_t speedL = 0;
uint8_t speedR = 0;

// Configure stepper
const uint8_t stepper_dir_pin = 13;
const uint8_t stepper_step_pin = A1;
const uint16_t stepper_step_delay = 2000;
const int16_t stepper_max_steps = 85;
int stepper_current_step = 0;
int stepper_target_step = 0;

// Configure motors
const uint8_t motorSTBY = 4;
const uint8_t motorL_PWM = A3;
const uint8_t motorL_IN1 = 24;
const uint8_t motorL_IN2 = 23;
const uint8_t motorR_PWM = A4;
const uint8_t motorR_IN1 = A5;
const uint8_t motorR_IN2 = 25;

// Configure encoders
volatile long count_motorL = 0;
volatile long count_motorR = 0;
const uint8_t motorL_encoderA = 10;
const uint8_t motorL_encoderB = 11;
const uint8_t motorR_encoderA = 6;

// Configure speed control
long last_count_motorL = 0;
long last_count_motorR = 0;
const uint8_t number_of_speed_measurements = 1;
long prev_motor_speeds[number_of_speed_measurements];
long avg_speed = 0;
long target_speed = 0;
long last_speed_measurement = 0;
uint8_t current_index = 0;
const uint8_t motorR_encoderB = 5;

void motorLEncoderAInterrupt() {
  if (digitalRead(motorL_encoderB)) {
    count_motorL += 1;
  } else {
    count_motorL -= 1;
  }
}
void motorREncoderAInterrupt() {
  if (digitalRead(motorR_encoderB)) {
    count_motorR += 1;
  } else {
    count_motorR -= 1;
  }
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // CAN initialization
  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, false); // turn off STANDBY
  pinMode(PIN_CAN_BOOSTEN, OUTPUT);
  digitalWrite(PIN_CAN_BOOSTEN, true); // turn on booster 
  if (!CAN.begin(250000)) { // start the CAN bus at 250 kbps
    Serial.println("Starting CAN failed!");
    while (1) delay(10);
  }
  Serial.println("Starting CAN!");
  
  // Stepper initialization
  pinMode(stepper_dir_pin, OUTPUT);
  pinMode(stepper_step_pin, OUTPUT);

  // Motor initialization
  pinMode(motorSTBY, OUTPUT);
  digitalWrite(motorSTBY, HIGH);
  pinMode(motorL_PWM, OUTPUT);
  pinMode(motorL_IN1, OUTPUT);
  pinMode(motorL_IN2, OUTPUT);
  pinMode(motorR_PWM, OUTPUT);
  pinMode(motorR_IN1, OUTPUT);
  pinMode(motorR_IN2, OUTPUT);

  // Encoder initialization
  pinMode(motorL_encoderA, INPUT_PULLUP);
  pinMode(motorL_encoderB, INPUT_PULLUP);
  pinMode(motorR_encoderA, INPUT_PULLUP);
  pinMode(motorR_encoderB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(motorL_encoderA), motorLEncoderAInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(motorR_encoderA), motorREncoderAInterrupt, RISING);

  // Speed control initialization
  for (uint8_t i = 0; i < number_of_speed_measurements; i++) {
    prev_motor_speeds[i] = 0;
  }
  
}

void loop() {
  // put your main code here, to run repeatedly:

  if (millis() - last_speed_measurement > 150) {
    current_index += 1;
  
    if (current_index >= number_of_speed_measurements) {
      current_index = 0;
    }
  
    prev_motor_speeds[current_index] = (count_motorL + count_motorR) / 2 - (last_count_motorL + last_count_motorR) / 2;
  
    
    last_count_motorL = count_motorL;
    last_count_motorR = count_motorR;
    avg_speed = 0;
    for (uint8_t i = 0; i < number_of_speed_measurements; i++) {
      avg_speed += prev_motor_speeds[i];
    }
    avg_speed = (long)(avg_speed/number_of_speed_measurements);

    last_speed_measurement = millis();

    Serial.print(target_speed);
    Serial.print("\t");
    Serial.print(avg_speed);
    Serial.print("\t");
    Serial.print(target_speed - avg_speed);
    Serial.print("\t");
    Serial.print(avg_speed - target_speed);
    Serial.print("\t");
    
    Serial.println(prev_motor_speeds[current_index]);

    if (target_speed - avg_speed > 10) {
      if (speedL < 245) speedL += 5;
      if (speedR < 245) speedR += 5;
    }
    if (avg_speed - target_speed > 10) {
      if (speedL > 10) speedL -= 5;
      if (speedR > 10) speedR -= 5;
    }
    
  }

  int packetSize = CAN.parsePacket();
  if (packetSize) {
    if (CAN.packetId() == 291) { // 291 = 0x123
      stepper_current_step = 0;
      stepper_target_step = 0;
      target_speed = 0;
      while (CAN.available()) {
        CAN.read();
      }
    }
    
    if (CAN.packetId() == 292) { // 292 = 0x124
      if (CAN.available()) angle = (uint8_t)CAN.read();
      stepper_target_step = map(angle, 45, 135, -stepper_max_steps, stepper_max_steps);
      //if (CAN.available()) speedL = (uint8_t)CAN.read();
      //if (CAN.available()) speedR = (uint8_t)CAN.read();
      if (CAN.available()) target_speed = (uint8_t)CAN.read();
      if (CAN.available()) target_speed = (uint8_t)CAN.read();
      while (CAN.available()) {
        CAN.read();
      }
    }
  }

  digitalWrite(motorL_IN1, LOW);
  digitalWrite(motorL_IN2, HIGH);

  digitalWrite(motorR_IN1, LOW);
  digitalWrite(motorR_IN2, HIGH);
  

  if (target_speed == 0) {
    analogWrite(motorL_PWM, 0);
    analogWrite(motorR_PWM, 0);
  } else {
    analogWrite(motorL_PWM, speedL);
    analogWrite(motorR_PWM, speedR);
  }

  if (stepper_target_step < stepper_current_step) {
    digitalWrite(stepper_dir_pin, LOW);
    digitalWrite(stepper_step_pin, HIGH);
    delayMicroseconds(stepper_step_delay);
    digitalWrite(stepper_step_pin, LOW);
    delayMicroseconds(stepper_step_delay);
    stepper_current_step -= 1;
  } else if (stepper_target_step > stepper_current_step) {
    digitalWrite(stepper_dir_pin, HIGH);
    digitalWrite(stepper_step_pin, HIGH);
    delayMicroseconds(stepper_step_delay);
    digitalWrite(stepper_step_pin, LOW);
    delayMicroseconds(stepper_step_delay);
    stepper_current_step += 1;
  }

  Serial.print("Angle: ");
  Serial.print(angle);
  Serial.print("\tstepper_target_step: ");
  Serial.print(stepper_target_step);
  Serial.print("\tstepper_current_step: ");
  Serial.println(stepper_current_step);
  
}
