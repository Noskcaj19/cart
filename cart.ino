// Needs Adafruit_BluefruitLE_nRF51 and Adafruit_LIS3DH libs

#include <Adafruit_BluefruitLE_SPI.h>
#include <Adafruit_LIS3DH.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Wire.h>

// Configuration
const float STEERING_ADJUSTMENT_PERCENTAGE = 0.1;
const float SPEED_ADJUSTMENT_PERCENTAGE    = 0.1;

const bool REVERSE_LEFT  = false;
const bool REVERSE_RIGHT = true;

// Motor controllers
const int MOTOR1_IN1 = 9;
const int MOTOR1_IN2 = 10;
const int MOTOR2_IN1 = 6;
const int MOTOR2_IN2 = 5;

// Bluefruit SPI, hardware
const int BLUEFRUIT_SPI_SCK  = 13;
const int BLUEFRUIT_SPI_MISO = 12;
const int BLUEFRUIT_SPI_MOSI = 11;
const int BLUEFRUIT_SPI_CS   = 8;
const int BLUEFRUIT_SPI_IRQ  = 7;
const int BLUEFRUIT_SPI_RST  = 4;

// I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// Runtime variables
double speed    = 0.0; // -1 = -100% output, 0 = 0%, 1 = 100% output
double steering = 0.0; // -1 = full left, 0 = center, 1 = full right

double last_velocity[3] = {0};
double displacement[3]  = {0};

unsigned long lastMillis = 0;

void updateDisplacement(double accel_x_g, double accel_y_g, double accel_z_g,
                        double time_dt) {
  double accel_g[3]           = {0};
  double accel_m_s2[3]        = {0};
  double curr_velocity_m_s[3] = {0};
  // float sample_time         = (1.0f / update_rate_hz);
  accel_g[0] = accel_x_g;
  accel_g[1] = accel_y_g;
  accel_g[2] = accel_z_g;
  for (int i = 0; i < 3; i++) {
    accel_m_s2[i]        = accel_g[i] * 9.80665f;
    curr_velocity_m_s[i] = last_velocity[i] + (accel_m_s2[i] * time_dt);
    displacement[i] +=
        last_velocity[i] + (0.5f * accel_m_s2[i] * time_dt * time_dt);
    last_velocity[i] = curr_velocity_m_s[i];
  }
}

struct MotorSpeeds {
  double left;
  double right;
};

struct MotorSpeeds calcMotorPercents(double xSpeed, double zRotation) {
  double leftSpeed;
  double rightSpeed;

  double maxInput = copysign(max(abs(xSpeed), abs(zRotation)), xSpeed);

  if (xSpeed >= 0.0) {
    // First quadrant, else second quadrant
    if (zRotation >= 0.0) {
      leftSpeed  = maxInput;
      rightSpeed = xSpeed - zRotation;
    } else {
      leftSpeed  = xSpeed + zRotation;
      rightSpeed = maxInput;
    }
  } else {
    // Third quadrant, else fourth quadrant
    if (zRotation >= 0.0) {
      leftSpeed  = xSpeed + zRotation;
      rightSpeed = maxInput;
    } else {
      leftSpeed  = maxInput;
      rightSpeed = xSpeed - zRotation;
    }
  }

  // Normalize the wheel speeds
  double maxMagnitude = max(abs(leftSpeed), abs(rightSpeed));
  if (maxMagnitude > 1.0) {
    leftSpeed /= maxMagnitude;
    rightSpeed /= maxMagnitude;
  }
  return {.left = leftSpeed, .right = rightSpeed};
}

void updateMotors(double speed, double steering) {
  MotorSpeeds speeds     = calcMotorPercents(speed, steering);
  double      leftSpeed  = int(constrain(speeds.left, -1, 1) * 255);
  double      rightSpeed = int(constrain(speeds.right, -1, 1) * 255);

  if (REVERSE_LEFT) {
    leftSpeed *= -1;
  }
  if (REVERSE_RIGHT) {
    rightSpeed *= -1;
  }

  if (speed == 0) {
    digitalWrite(MOTOR1_IN1, LOW);
    analogWrite(MOTOR1_IN2, 0);
    digitalWrite(MOTOR2_IN1, LOW);
    analogWrite(MOTOR2_IN2, 0);
    return;
  }

  if (rightSpeed >= 0) {
    digitalWrite(MOTOR1_IN1, LOW);
    analogWrite(MOTOR1_IN2, rightSpeed);
  } else {
    analogWrite(MOTOR1_IN1, rightSpeed * -1);
    digitalWrite(MOTOR1_IN2, LOW);
  }

  if (leftSpeed >= 0) {
    digitalWrite(MOTOR2_IN1, LOW);
    analogWrite(MOTOR2_IN2, leftSpeed);
  } else {
    analogWrite(MOTOR2_IN1, leftSpeed * -1);
    digitalWrite(MOTOR2_IN2, LOW);
  }
}

bool checkCRC(uint8_t sum, uint8_t CRCindex) {
  for (uint8_t i = 2; i < CRCindex; i++)
    sum -= (uint8_t)ble.buffer[i];
  return ((uint8_t)ble.buffer[CRCindex] == sum);
}

void setup(void) {
  Serial.begin(9600);
  Serial.println("Booted");
  Serial.println("Waiting for BLE...");
  if (!ble.begin(false))
    for (;;)
      ; // BLE init error?
  ble.echo(false);
  Serial.println("BLE ready");
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);

  if (!lis.begin(0x18)) { // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start Adafruit_LIS3DH");
  }
  lis.setRange(LIS3DH_RANGE_4_G);

  Serial.println("Setup complete");
}

void loop(void) {
  unsigned long currentMillis = millis();
  // TX Command: AT+BLEUARTTX

  lis.read();
  updateDisplacement(lis.x_g, lis.y_g, lis.z_g, currentMillis - lastMillis);

  // Process any pending Bluetooth input
  if (ble.isConnected()) {
    ble.println(F("AT+BLEUARTRX"));      // Request string from BLE module
    ble.readline();                      // Read outcome
    if (!strncmp(ble.buffer, "!B", 2) && // Controller button command
        checkCRC(255 - '!' - 'B', 4) &&  // Verify checksum
        (ble.buffer[3] == '1'))          // Button press? 1=press 0=release
    {
      switch (ble.buffer[2]) {
      case '1':
        speed = 1.0; // Max speed
        break;
      // case '2':
      //   speed = -1.0; // Max reverse
      //   break;
      case '3':
        speed = 0.0; // Stop
        break;
      case '4':
        steering = 0.0; // Center steering
        break;
      case '5': // Up (faster)
        speed = constrain(speed + SPEED_ADJUSTMENT_PERCENTAGE, -1, 1);
        break;
      case '6': // Down (slower)
        speed = constrain(speed - SPEED_ADJUSTMENT_PERCENTAGE, -1, 1);
        break;
      case '7': // Left
        steering = constrain(steering - STEERING_ADJUSTMENT_PERCENTAGE, -1, 1);
        break;
      case '8': // Right
        steering = constrain(steering + STEERING_ADJUSTMENT_PERCENTAGE, -1, 1);
        break;
      }
    }
    ble.println("AT+BLEUARTTX=x:" + String(displacement[0], 2) +
                ",y:" + String(displacement[1], 2) +
                ",z:" + String(displacement[2], 2));
  }
  updateMotors(speed, steering);
  lastMillis = currentMillis;
}
