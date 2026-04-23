#include <MPU6050_tockn.h>
#include <Wire.h>
#include <IRremote.h>

// Motor Pins
#define IN1 13
#define IN2 12
#define ENA 5
#define ENB 7
#define IN3 9
#define IN4 8

// IR Pin
#define IR_RECEIVE_PIN 2

MPU6050 mpu6050(Wire);

// PID Variables
float kp = 40;
float ki = 1;
float kd = 35;

float setpoint = 0;
float targetSetpoint = 0;
float error, lastError, integral, derivative, output;

float turnSpeed = 0;
float targetTurnSpeed = 0;

// ฟิลเตอร์สมูท
float smoothedSetpoint = 0;
float smoothedTurnSpeed = 0;

// IR Remote
unsigned long lastIRTime = 0;
const int remoteTimeout = 2000;

String inputString = "";
bool newInput = false;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  delay(3000);
  Serial.println("เริ่มระบบบาลานซ์ + รีโมท!");
}

void loop() {
  mpu6050.update();
  float angle = mpu6050.getAngleX();

  checkIR();

  // รีเซ็ตถ้าไม่กดรีโมทนาน
  if (millis() - lastIRTime > remoteTimeout) {
    targetSetpoint = 0;
    targetTurnSpeed = 0;
  }

  // Smooth Easing
  smoothedSetpoint = smoothStep(smoothedSetpoint, targetSetpoint, 0.02);  // 2% ต่อรอบ
  setpoint = smoothedSetpoint;

  smoothedTurnSpeed = smoothStep(smoothedTurnSpeed, targetTurnSpeed, 0.05);  // เลี้ยวลื่น
  turnSpeed = smoothedTurnSpeed;

  // ป้องกันล้ม
  if (abs(angle) > 40) {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    return;
  }

  pidControl(angle);
  checkSerial();

  delay(10);
}

void pidControl(float pitch) {
  error = setpoint - pitch;
  integral += error;
  derivative = error - lastError;
  output = kp * error + ki * integral + kd * derivative;
  output = constrain(output, -255, 255);

  lastError = error;
  motorcontrol(output);
}

void motorcontrol(float output) {
  int motorSpeed = abs(output);
  motorSpeed = motorSpeed * 0.7;  // Boost 70%
  motorSpeed = constrain(motorSpeed, 0, 255);

  int leftSpeed = motorSpeed + turnSpeed;
  int rightSpeed = motorSpeed - turnSpeed;

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  if (output > 0) {
    // เดินหน้า
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    // ถอยหลัง
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}

void checkIR() {
  if (IrReceiver.decode()) {
    uint32_t code = IrReceiver.decodedIRData.command;
    Serial.print("IR code: ");
    Serial.println(code, HEX);

    lastIRTime = millis();

    switch (code) {
      case 0x18:  // เดินหน้า
        targetSetpoint = 3;
        break;
      case 0x52:  // ถอยหลัง
        targetSetpoint = -3;
        break;
      case 0x8:   // เลี้ยวซ้าย
        targetTurnSpeed = -40;
        break;
      case 0x5A:  // เลี้ยวขวา
        targetTurnSpeed = 301                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
        ;
        break;
      case 0x1C:  // หยุด
        targetSetpoint = 0;
        targetTurnSpeed = 0;
        break;
    }

    IrReceiver.resume();
  }
}

void checkSerial() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      newInput = true;
    } else {
      inputString += c;
    }
  }

  if (newInput) {
    inputString.trim();
    handleInput(inputString);
    inputString = "";
    newInput = false;
  }
}

void handleInput(String input) {
  if (input.startsWith("kp=")) {
    kp = input.substring(3).toFloat();
    Serial.print("ตั้งค่า kp = "); Serial.println(kp);
  } else if (input.startsWith("ki=")) {
    ki = input.substring(3).toFloat();
    Serial.print("ตั้งค่า ki = "); Serial.println(ki);
  } else if (input.startsWith("kd=")) {
    kd = input.substring(3).toFloat();
    Serial.print("ตั้งค่า kd = "); Serial.println(kd);
  } else {
    Serial.println("ใช้รูปแบบ: kp=60 หรือ kd=35 หรือ ki=1.0");
  }
}

// ฟังก์ชันสมูทเทพ
float smoothStep(float current, f loat target, float smoothFactor) {
  return current + (target - current) * smoothFactor;
}
