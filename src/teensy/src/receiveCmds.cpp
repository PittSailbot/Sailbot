#include "receiveCmds.h"
#include "teensy.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

byte buffer[BUFFER_SIZE]; // Buffer to store received data
int bytesReceived = 0; // Number of bytes received
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(I2C_ADDR);

bool setupReceiver() {
  bool servoDriverFound = i2c_init(I2C_ADDR);
  if (!servoDriverFound){
    return false;
  }

  Wire1.begin(SLAVE_ADDRESS); // Initialize I2C communication as a slave
  Wire1.onReceive(receiveData); // Set callback function for data reception
  Wire1.onRequest(sendData); // Set callback function for data transmission

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  setServo(SERVO_0_CH, 90);
  setServo(SERVO_1_CH, 0);
  delay(1000);
  setServo(SERVO_1_CH, 180);
  delay(1000);

  

  Serial.println("Started Receiver");
  return true;
  
}

void receiveData(int byteCount) {
  while (Wire1.available()) {
    if (bytesReceived < BUFFER_SIZE) {
      // Read data byte by byte and store it in the buffer
      buffer[bytesReceived] = Wire1.read();
      bytesReceived++;
    } else {
      // Buffer overflow, discard additional data
      Wire1.read();
    }
  }
  // for (int i = 0; i < bytesReceived; i++) {
  //   Serial.print(buffer[i]);
  //   Serial.print(" ");
  // }
  // Serial.println(bytesReceived);

  if (bytesReceived > 3){
    if (buffer[1] == CAM_MOVE_ABS_CMD){
      setServo(SERVO_0_CH, buffer[2]);
      setServo(SERVO_1_CH, buffer[3]);
    }
  }

  else if (bytesReceived > 2){
    if (buffer[1] == CAM_H_MOVE_ABS_CMD){
      setServo(SERVO_0_CH, buffer[2]);
    }
    if (buffer[1] == CAM_V_MOVE_ABS_CMD){
      setServo(SERVO_1_CH, buffer[2]);
    }
  }

  clearBuffer();
}

void clearBuffer() {
  bytesReceived = 0;
}

void sendData() {
  // Prepare data to send
  int dataToSend = 42; // Example data
  // Send data byte by byte
  Wire1.write((uint8_t*)&dataToSend, sizeof(dataToSend));
}

void setServo(uint8_t servoIndex, uint8_t angle){
  int pwmVal;
  if (servoIndex == SERVO_0_CH){
    angle = constrain(angle, SERVO_0_MIN_ANGLE, SERVO_0_MAX_ANGLE);
    pwmVal = map(angle, SERVO_0_MIN_ANGLE, SERVO_0_MAX_ANGLE, SERVO_0_MIN_PWM, SERVO_0_MAX_PWM);
  }
  else if (servoIndex == SERVO_1_CH){
    angle = constrain(angle, SERVO_1_MIN_ANGLE, SERVO_1_MAX_ANGLE);
    pwmVal = map(angle, SERVO_1_MIN_ANGLE, SERVO_1_MAX_ANGLE, SERVO_1_MIN_PWM, SERVO_1_MAX_PWM);
  }
  else{
    angle = constrain(angle, 0, 180);
    pwmVal = map(angle, 0, 180, 150, 600);
  }
  pwm.setPWM(servoIndex, 0, pwmVal);
  
}

bool i2c_init(uint8_t i2c_addr) {
    Wire.begin(); // Initialize I2C communication
    Wire.beginTransmission(i2c_addr); // Begin transmission to the specified I2C address
    int res = Wire.endTransmission(); // Check if device responds
    
    if (res != 0) {
        Serial.println("Failed to communicate with Camera Servos over I2C");
        return false;
    }
    
    return true; // Success
}
