#include <Wire.h>

// I2C params
#define I2C_ADDR 0x40
#define BUFFER_SIZE 32  // Adjust buffer size according to your needs
#define SERVO_FREQ 50

#define SERVO_0_MAX_ANGLE 155
#define SERVO_0_MIN_ANGLE 60
#define SERVO_1_MAX_ANGLE 180
#define SERVO_1_MIN_ANGLE 0

#define SERVO_0_MAX_PWM 600
#define SERVO_0_MIN_PWM 140
#define SERVO_1_MAX_PWM 600
#define SERVO_1_MIN_PWM 150

#define STEP 1
#define STEP_DELAY 2
#define SERVO_0_CH 0
#define SERVO_1_CH 1

bool setupReceiver();
void receiveData(int byteCount);
void sendData();
void clearBuffer();
void setServo(uint8_t servoIndex, uint8_t angle);
bool i2c_init(unsigned char i2c_addr);