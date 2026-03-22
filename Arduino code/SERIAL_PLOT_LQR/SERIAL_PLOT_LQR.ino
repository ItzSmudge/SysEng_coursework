// LQR Controller for Inverted Pendulum
// Timer-based polling using mbed::Ticker (Arduino GIGA R1)
// ============================================
// INCLUDES
// ============================================
#include <Motoron.h>
#include "mbed.h"

// ============================================
// MOTORON I2C CONFIGURATION
// ============================================
MotoronI2C mc1(16);
MotoronI2C mc2(17);

const int MAX_SPEED      =  800;
const int MIN_SPEED      = -800;
const int PWM_to_motor   =  1;
const int MOTORS_FORWARD = -1;

// ============================================
// PENDULUM ENCODER CONFIGURATION (AS22)
// ============================================
const int encoderPinA  = 10;
const int encoderPinB  = 11;
const int encoderIndex = 4;
const int COUNTS_PER_REV = 4096;

volatile long encoderCount   = 0;
volatile bool indexFound     = false;
volatile int  lastStateA     = LOW;
volatile int  lastStateB     = LOW;
volatile int  lastIndexState = LOW;

// ============================================
// MOTOR ENCODER CONFIGURATION (Pololu 25D)
// ============================================
const int MOTOR_PIN_A = 12;
const int MOTOR_PIN_B = 13;
const float MOTOR_COUNTS_PER_REV = 232.32;

volatile long motorCount = 0;
volatile int  lastMotorA = LOW;
volatile int  lastMotorB = LOW;

// ============================================
// TICKER CONFIGURATION
// ============================================
const int SAMPLE_INTERVAL_US = 100; // 10kHz

mbed::Ticker encoderTicker;

// ============================================
// SERIAL PLOTTER MODE
// ============================================
// Set to true  → Arduino Serial Plotter (Tools → Serial Plotter)
// Set to false → Human-readable Serial Monitor
bool PLOTTER_MODE = true;

// ============================================
// TICKER CALLBACK: QUADRATURE SAMPLING
// ============================================
void sampleEncoders() {
  // ── Pendulum encoder ──
  int currentStateA     = digitalRead(encoderPinA);
  int currentStateB     = digitalRead(encoderPinB);
  int currentIndexState = digitalRead(encoderIndex);

  if (currentStateA != lastStateA || currentStateB != lastStateB) {
    if (lastStateA == LOW && currentStateA == HIGH) {
      encoderCount += (currentStateB == LOW) ? 1 : -1;
    } else if (lastStateA == HIGH && currentStateA == LOW) {
      encoderCount += (currentStateB == HIGH) ? 1 : -1;
    } else if (lastStateB == LOW && currentStateB == HIGH) {
      encoderCount += (currentStateA == HIGH) ? 1 : -1;
    } else if (lastStateB == HIGH && currentStateB == LOW) {
      encoderCount += (currentStateA == LOW) ? 1 : -1;
    }
    lastStateA = currentStateA;
    lastStateB = currentStateB;
  }

  if (lastIndexState == LOW && currentIndexState == HIGH) {
    encoderCount = 0;
    indexFound   = true;
  }
  lastIndexState = currentIndexState;

  // ── Motor encoder ──
  int currentMotorA = digitalRead(MOTOR_PIN_A);
  int currentMotorB = digitalRead(MOTOR_PIN_B);

  if (currentMotorA != lastMotorA || currentMotorB != lastMotorB) {
    if (lastMotorA == LOW && currentMotorA == HIGH) {
      motorCount += (currentMotorB == LOW) ? 1 : -1;
    } else if (lastMotorA == HIGH && currentMotorA == LOW) {
      motorCount += (currentMotorB == HIGH) ? 1 : -1;
    } else if (lastMotorB == LOW && currentMotorB == HIGH) {
      motorCount += (currentMotorA == HIGH) ? 1 : -1;
    } else if (lastMotorB == HIGH && currentMotorB == LOW) {
      motorCount += (currentMotorA == LOW) ? 1 : -1;
    }
    lastMotorA = currentMotorA;
    lastMotorB = currentMotorB;
  }
}

// ── Safe snapshots ──
long getPendulumCount() {
  core_util_critical_section_enter();
  long snapshot = encoderCount;
  core_util_critical_section_exit();
  return snapshot;
}

long getMotorCount() {
  core_util_critical_section_enter();
  long snapshot = motorCount;
  core_util_critical_section_exit();
  return snapshot;
}

// ============================================
// MOVING AVERAGE FILTER CLASS
// ============================================
class MovingAverageFilter {
private:
  float* buffer;
  int windowSize;
  int bufferIndex;
  int bufferCount;
  float sum;

public:
  MovingAverageFilter(int size) {
    windowSize  = size;
    buffer      = new float[windowSize];
    bufferIndex = 0;
    bufferCount = 0;
    sum         = 0.0;
    for (int i = 0; i < windowSize; i++) buffer[i] = 0.0;
  }

  ~MovingAverageFilter() { delete[] buffer; }

  float apply(float value) {
    sum -= buffer[bufferIndex];
    buffer[bufferIndex] = value;
    sum += value;
    bufferIndex = (bufferIndex + 1) % windowSize;
    if (bufferCount < windowSize) bufferCount++;
    return sum / bufferCount;
  }

  void reset() {
    bufferIndex = 0;
    bufferCount = 0;
    sum         = 0.0;
    for (int i = 0; i < windowSize; i++) buffer[i] = 0.0;
  }
};

// ============================================
// LQR CONTROLLER CLASS
// ============================================
class LQRController {
private:
  float k_x;
  float k_x_dot;
  float k_theta;
  float k_theta_dot;

  bool filter_enabled;
  MovingAverageFilter* filter_theta;
  MovingAverageFilter* filter_x;

  float prev_theta;
  float prev_x;
  float theta_dot;
  float x_dot;
  unsigned long lastTime;
  float dt;

public:
  LQRController(float k_x_gain = 1.0, float k_x_dot_gain = 1.0,
                float k_theta_gain = 50.0, float k_theta_dot_gain = 5.0,
                float timestep = 0.01, int window_size = 10, bool filter_en = false) {
    k_x         = k_x_gain;
    k_x_dot     = k_x_dot_gain;
    k_theta     = k_theta_gain;
    k_theta_dot = k_theta_dot_gain;
    dt          = timestep;
    prev_theta  = 0.0;
    prev_x      = 0.0;
    theta_dot   = 0.0;
    x_dot       = 0.0;
    filter_enabled = filter_en;
    if (filter_enabled) {
      filter_theta = new MovingAverageFilter(window_size);
      filter_x     = new MovingAverageFilter(window_size);
    } else {
      filter_theta = nullptr;
      filter_x     = nullptr;
    }
    lastTime = millis();
  }

  ~LQRController() {
    if (filter_theta) delete filter_theta;
    if (filter_x)     delete filter_x;
  }

  float getAction(float x, float theta, float target_x = 0.0) {
    unsigned long currentTime = millis();
    float actual_dt = (currentTime - lastTime) / 1000.0;
    if (actual_dt <= 0.0) actual_dt = dt;
    lastTime = currentTime;

    float filtered_theta = filter_enabled ? filter_theta->apply(theta) : theta;
    float filtered_x     = filter_enabled ? filter_x->apply(x)         : x;

    theta_dot = (filtered_theta - prev_theta) / actual_dt;
    x_dot     = (filtered_x     - prev_x)     / actual_dt;
    prev_theta = filtered_theta;
    prev_x     = filtered_x;

    float x_error = target_x - filtered_x;

    // LQR control law: u = -K * [x_error, x_dot, theta, theta_dot]
    float force = -(k_x * x_error + k_x_dot * x_dot + k_theta * filtered_theta + k_theta_dot * theta_dot);
    return force;
  }

  void reset() {
    prev_theta = 0.0;
    prev_x     = 0.0;
    theta_dot  = 0.0;
    x_dot      = 0.0;
    if (filter_enabled) {
      filter_theta->reset();
      filter_x->reset();
    }
    lastTime = millis();
  }

  void setGains(float k_x_gain, float k_x_dot_gain, float k_theta_gain, float k_theta_dot_gain) {
    k_x         = k_x_gain;
    k_x_dot     = k_x_dot_gain;
    k_theta     = k_theta_gain;
    k_theta_dot = k_theta_dot_gain;
  }

  float getThetaDot()  { return theta_dot; }
  float getXDot()      { return x_dot; }
  float getKx()        { return k_x; }
  float getKxDot()     { return k_x_dot; }
  float getKtheta()    { return k_theta; }
  float getKthetaDot() { return k_theta_dot; }
};

// ============================================
// GLOBAL OBJECTS & STATE
// ============================================
LQRController lqr(
  0.0,     // k_x
  0.0,     // k_x_dot
  1000.0,  // k_theta
  3000.0,  // k_theta_dot
  0.01,    // dt
  5,       // window_size
  true     // filter_enabled
);

float target_x = 0.0;  // Updated by SPRINT command

// ============================================
// FUNCTION DECLARATIONS
// ============================================
float getAngleRadians();
float getMotorPosition();
int   forceToMotorSpeed(float force);
void  setMotorSpeed(int speed);
void  handleSerialCommands();

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  Wire1.begin();

  pinMode(encoderPinA,  INPUT);
  pinMode(encoderPinB,  INPUT);
  pinMode(encoderIndex, INPUT);
  pinMode(MOTOR_PIN_A,  INPUT);
  pinMode(MOTOR_PIN_B,  INPUT);

  lastStateA     = digitalRead(encoderPinA);
  lastStateB     = digitalRead(encoderPinB);
  lastIndexState = digitalRead(encoderIndex);
  lastMotorA     = digitalRead(MOTOR_PIN_A);
  lastMotorB     = digitalRead(MOTOR_PIN_B);

  encoderTicker.attach_us(&sampleEncoders, SAMPLE_INTERVAL_US);

  delay(100);

  mc1.setBus(&Wire1);
  mc2.setBus(&Wire1);

  mc1.reinitialize();
  mc1.disableCrc();
  mc1.clearResetFlag();
  mc1.disableCommandTimeout();

  mc2.reinitialize();
  mc2.disableCrc();
  mc2.clearResetFlag();
  mc2.disableCommandTimeout();

  if (!PLOTTER_MODE) {
    Serial.println("Driver 1 (Addr 16) Initialized.");
    Serial.println("Driver 2 (Addr 17) Initialized.");
    Serial.println("========================================");
    Serial.println("LQR CONTROLLER WITH DUAL ENCODERS");
    Serial.println("  Board    : Arduino GIGA R1 (mbed)");
    Serial.println("  Encoder  : mbed::Ticker polling 10kHz");
    Serial.println("  Pendulum : AS22  (Pins 10, 11, 4)");
    Serial.println("  Motor    : Pololu 25D (Pins 12, 13)");
    Serial.println("  CPR      : 4096 counts/rev");
    Serial.println("========================================");
    Serial.println("System ready. Starting control loop...");
    Serial.println();
  }

  lqr.reset();
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  static unsigned long lastControl = 0;
  static unsigned long lastPrint   = 0;
  static int motorSpeed            = 0;
  unsigned long currentTime        = millis();

  // Control loop at 100Hz (10ms)
  if (currentTime - lastControl >= 10) {
    float theta = getAngleRadians();
    float x     = getMotorPosition();

    float force = lqr.getAction(x, theta, target_x);
    motorSpeed  = forceToMotorSpeed(force);
    setMotorSpeed(motorSpeed);

    lastControl = currentTime;
  }

  // Print / plot at 20Hz (50ms) — faster rate gives smoother plotter trace
  if (currentTime - lastPrint >= 50) {
    float theta = getAngleRadians();
    float x     = getMotorPosition();

    if (PLOTTER_MODE) {
      // ── Serial Plotter format ──
      // Each label:value pair separated by tabs; println ends the sample row.
      // Open Tools → Serial Plotter to see live traces.
      // Scale Control by /100 so it fits on the same axis as theta (rad).
      Serial.print("Theta_rad:");        Serial.print(theta, 4);
      Serial.print("\t");
      Serial.print("Theta_deg/100:");    Serial.print(theta * (180.0 / PI) / 1.0, 4);
      Serial.print("\t");
      Serial.print("X_m:");             Serial.print(x, 4);
      Serial.print("\t");
      Serial.print("Target_x:");        Serial.print(target_x, 0);
      Serial.print("\t");
      Serial.print("Control/1000:");    Serial.println(motorSpeed / 1.0, 0);
    } else {
      // ── Human-readable Serial Monitor format ──
      Serial.print("Theta: ");          Serial.print(theta, 4);
      Serial.print(" rad | ");          Serial.print(theta * (180.0 / PI), 2);
      Serial.print(" deg | X: ");       Serial.print(x, 4);
      Serial.print(" m | Target: ");    Serial.print(target_x, 4);
      Serial.print(" m | Theta_dot: "); Serial.print(lqr.getThetaDot(), 4);
      Serial.print(" | X_dot: ");       Serial.print(lqr.getXDot(), 4);
      Serial.print(" | Control: ");     Serial.println(motorSpeed);
    }

    lastPrint = currentTime;
  }

  handleSerialCommands();
}

// ============================================
// HELPER FUNCTIONS
// ============================================
float getAngleRadians() {
  long count  = getPendulumCount() % COUNTS_PER_REV;
  float angle = (count * 2.0 * PI) / COUNTS_PER_REV;
  if (angle > PI) angle -= 2.0 * PI;
  return angle;
}

float getMotorPosition() {
  const float WHEEL_DIAMETER      = 0.06;
  const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI;
  return (getMotorCount() / MOTOR_COUNTS_PER_REV) * WHEEL_CIRCUMFERENCE;
}

int forceToMotorSpeed(float force) {
  float num = force * PWM_to_motor;
  if (num > 0) {
    num = num + 150;
  } else {
    num = num - 150;
  }
  int speed = (int)constrain(num, MIN_SPEED, MAX_SPEED);
  return MOTORS_FORWARD * speed;
}

void setMotorSpeed(int speed) {
  mc1.setSpeed(2, speed);
  mc1.setSpeed(3, speed);
  mc2.setSpeed(2, speed);
  mc2.setSpeed(3, speed);
}

// ============================================
// SERIAL COMMAND HANDLER
// ============================================
void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "RESET") {
      lqr.reset();
      if (!PLOTTER_MODE) Serial.println("LQR reset");

    } else if (command.startsWith("SET ")) {
      float gains[4];
      int idx = 0, lastSpace = 3;
      for (int i = 4; i < (int)command.length() && idx < 4; i++) {
        if (command[i] == ' ' || i == (int)command.length() - 1) {
          if (i == (int)command.length() - 1) i++;
          gains[idx++] = command.substring(lastSpace + 1, i).toFloat();
          lastSpace = i;
        }
      }
      if (idx == 4) {
        lqr.setGains(gains[0], gains[1], gains[2], gains[3]);
        if (!PLOTTER_MODE) {
          Serial.print("LQR Gains updated: k_x=");     Serial.print(gains[0]);
          Serial.print(" k_x_dot=");                   Serial.print(gains[1]);
          Serial.print(" k_theta=");                   Serial.print(gains[2]);
          Serial.print(" k_theta_dot=");               Serial.println(gains[3]);
        }
      }

    } else if (command.startsWith("SPRINT ")) {
      target_x = command.substring(7).toFloat();
      if (!PLOTTER_MODE) {
        Serial.print("Sprint target set to: ");
        Serial.print(target_x, 3);
        Serial.println(" m");
      }

    } else if (command == "HOME") {
      target_x = 0.0;
      if (!PLOTTER_MODE) Serial.println("Target reset to 0.0 m");

    } else if (command == "STOP") {
      setMotorSpeed(0);
      if (!PLOTTER_MODE) Serial.println("Motors stopped");

    } else if (command == "GAINS") {
      if (!PLOTTER_MODE) {
        Serial.print("Current LQR Gains: k_x=");   Serial.print(lqr.getKx(), 4);
        Serial.print(" k_x_dot=");                 Serial.print(lqr.getKxDot(), 4);
        Serial.print(" k_theta=");                 Serial.print(lqr.getKtheta(), 4);
        Serial.print(" k_theta_dot=");             Serial.println(lqr.getKthetaDot(), 4);
      }

    } else if (command == "PLOT_ON") {
      PLOTTER_MODE = true;

    } else if (command == "PLOT_OFF") {
      PLOTTER_MODE = false;
    }
  }
}

// ============================================
// SERIAL COMMAND REFERENCE
// ============================================
// RESET             — Reset LQR internal state
// STOP              — Cut motor output immediately
// GAINS             — Print current gains (Monitor mode only)
// SET kx kxd kt ktd — e.g. SET 0 0 1000 3000
// SPRINT x          — Move to target position in metres, e.g. SPRINT 2.0
// HOME              — Return target to 0.0 m
// PLOT_ON           — Switch to Serial Plotter output format
// PLOT_OFF          — Switch to human-readable Serial Monitor format
//
// PLOTTER CHANNELS (Tools → Serial Plotter):
//   Theta_rad        — pendulum angle in radians        (~±0.3 when balancing)
//   Theta_deg/100    — angle in degrees, scaled ÷100    (~±0.17 when balancing)
//   X_m              — cart position in metres
//   Target_x         — current sprint target in metres
//   Control/1000     — motor speed command, scaled ÷1000
