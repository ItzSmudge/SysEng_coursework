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
const int MOTOR_PIN_A = 2;
const int MOTOR_PIN_B = 3;
const float MOTOR_COUNTS_PER_REV = 464;

volatile long motorCount = 0;
volatile int  lastMotorA = LOW;
volatile int  lastMotorB = LOW;

// Sprint control state
bool sprintActive      = false;
float sprintTargetDist = 1.0;  // 1 metre target
float sprintStartPos   = 0.0;

// ============================================
// TICKER CONFIGURATION
// ============================================
const int SAMPLE_INTERVAL_US = 100; // 10kHz

mbed::Ticker encoderTicker;

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
  int made_distance = 0;

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
    made_distance = 0;
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
    if (x > sprintTargetDist && !made_distance) {
      made_distance = 1;
      setGains(0, 0, 750, 2625);
    }

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
  1000.0,     // k_x
  1000.0,     // k_x_dot
  750,        // k_theta    — scaled to match PID kp_theta
  2625,       // k_theta_dot — scaled to match PID kd_theta
  0.01,       // dt
  5,          // window_size
  true        // filter_enabled
);



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
  Serial.println("Driver 1 (Addr 16) Initialized.");

  mc2.reinitialize();
  mc2.disableCrc();
  mc2.clearResetFlag();
  mc2.disableCommandTimeout();
  Serial.println("Driver 2 (Addr 17) Initialized.");

  Serial.println("========================================");
  Serial.println("LQR CONTROLLER WITH DUAL ENCODERS");
  Serial.println("  Board    : Arduino GIGA R1 (mbed)");
  Serial.println("  Encoder  : mbed::Ticker polling 10kHz");
  Serial.println("  Pendulum : AS22  (Pins 10, 11, 4)");
  Serial.println("  Motor    : Pololu 25D (Pins 2, 3)");
  Serial.println("  CPR      : 4096 counts/rev");
  Serial.println("========================================");
  Serial.println("System ready. Waiting for commands...");
  Serial.println();
  printCommandMenu();

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
    float theta    = getAngleRadians();
    float x        = getMotorPosition();
    float target_x = -1.0;

    // If sprint is active, adjust target based on distance travelled
    if (sprintActive) {
      float distanceTravelled = x - sprintStartPos;
       float force = lqr.getAction(x, theta, target_x);
        motorSpeed = forceToMotorSpeed(force);
        setMotorSpeed(motorSpeed);
      
      // Check if 1 metre sprint is complete
      // if (distanceTravelled >= sprintTargetDist) {
      //   // sprintActive = false;
      //   // motorSpeed = 0;
      //   // setMotorSpeed(0);
      //   // Serial.println("\n>>> SPRINT COMPLETE! Robot stopped. <<<");
      //   // Serial.print("Distance travelled: ");
      //   // Serial.print(distanceTravelled, 4);
      //   // Serial.println(" m");
      // } else {
      //   // Continue with LQR control during sprint
      //   float force = lqr.getAction(x, theta, target_x);
      //   motorSpeed = forceToMotorSpeed(force);
      //   setMotorSpeed(motorSpeed);
      // }
    }

    lastControl = currentTime;
  }

  // Print debug info at 10Hz (100ms)
  if (currentTime - lastPrint >= 100) {
    float theta = getAngleRadians();
    float x     = getMotorPosition();
    Serial.print("Theta: ");          Serial.print(theta, 4);
    Serial.print(" rad | ");          Serial.print(theta * (180.0 / PI), 2);
    Serial.print(" deg | X: ");       Serial.print(x, 4);
    Serial.print(" m | Theta_dot: "); Serial.print(lqr.getThetaDot(), 4);
    Serial.print(" | X_dot: ");       Serial.print(lqr.getXDot(), 4);
    Serial.print(" | Sprint: ");      Serial.print(sprintActive ? "ACTIVE" : "IDLE");
    Serial.print(" | Control: ");     Serial.println(motorSpeed);
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
  const float WHEEL_DIAMETER      = 0.07;
  const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI;
  return (getMotorCount() / MOTOR_COUNTS_PER_REV) * WHEEL_CIRCUMFERENCE;
}

int forceToMotorSpeed(float force) {
  // Matches PID scaling exactly
  float num = force * PWM_to_motor;
  if (num > 0){
    num = num + 175;
  } else{
    num = num - 175; 
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

void printCommandMenu() {
  Serial.println("========================================");
  Serial.println("AVAILABLE COMMANDS:");
  Serial.println("  START      - Begin 1 metre sprint");
  Serial.println("  STOP       - Stop motors immediately");
  Serial.println("  RESET      - Reset LQR controller");
  Serial.println("  RESET_MOTOR - Reset motor encoder count");
  Serial.println("  GAINS      - Display current LQR gains");
  Serial.println("  SET gains  - Set gains (k_x k_x_dot k_theta k_theta_dot)");
  Serial.println("               Example: SET 1000 1000 750 2625");
  Serial.println("========================================");
}

void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();

    if (command == "START") {
      if (sprintActive) {
        Serial.println(">>> Sprint already in progress!");
      } else {
        sprintActive = true;
        sprintStartPos = getMotorPosition();
        lqr.reset();
        Serial.println("\n>>> SPRINT STARTED! <<<");
        Serial.println("Robot will travel 1.0 metre while balancing pendulum.");
        Serial.print("Start position: ");
        Serial.print(sprintStartPos, 4);
        Serial.println(" m");
      }

    } else if (command == "STOP") {
      if (sprintActive) {
        sprintActive = false;
        float distanceTravelled = getMotorPosition() - sprintStartPos;
        Serial.println("\n>>> SPRINT STOPPED MANUALLY <<<");
        Serial.print("Distance travelled: ");
        Serial.print(distanceTravelled, 4);
        Serial.println(" m");
      }
      setMotorSpeed(0);
      Serial.println("Motors stopped.");

    } else if (command == "RESET") {
      lqr.reset();
      sprintActive = false;
      setMotorSpeed(0);
      Serial.println(">>> LQR controller reset <<<");
      Serial.println("Motors stopped.");

    } else if (command == "RESET_MOTOR") {
      core_util_critical_section_enter();
      motorCount = 0;
      core_util_critical_section_exit();
      sprintActive = false;
      setMotorSpeed(0);
      Serial.println(">>> MOTOR ENCODER RESET <<<");
      Serial.println("Motor count set to 0. Motors stopped.");

    } else if (command == "GAINS") {
      Serial.println("\n========================================");
      Serial.println("CURRENT LQR GAINS:");
      Serial.print("  k_x        = ");       Serial.println(lqr.getKx(), 4);
      Serial.print("  k_x_dot    = ");       Serial.println(lqr.getKxDot(), 4);
      Serial.print("  k_theta    = ");       Serial.println(lqr.getKtheta(), 4);
      Serial.print("  k_theta_dot= ");       Serial.println(lqr.getKthetaDot(), 4);
      Serial.println("========================================");

    } else if (command.startsWith("SET ")) {
      float gains[4];
      int idx = 0, lastSpace = 3;
      for (int i = 4; i < command.length() && idx < 4; i++) {
        if (command[i] == ' ' || i == command.length() - 1) {
          if (i == command.length() - 1) i++;
          gains[idx++] = command.substring(lastSpace + 1, i).toFloat();
          lastSpace = i;
        }
      }
      if (idx == 4) {
        lqr.setGains(gains[0], gains[1], gains[2], gains[3]);
        Serial.println("\n>>> LQR GAINS UPDATED <<<");
        Serial.print("  k_x        = ");       Serial.println(gains[0]);
        Serial.print("  k_x_dot    = ");       Serial.println(gains[1]);
        Serial.print("  k_theta    = ");       Serial.println(gains[2]);
        Serial.print("  k_theta_dot= ");       Serial.println(gains[3]);
      } else {
        Serial.println("Error: SET command requires 4 gains. Usage: SET k_x k_x_dot k_theta k_theta_dot");
      }

    } else if (command == "HELP") {
      printCommandMenu();

    } else if (command != "") {
      Serial.println("Unknown command. Type HELP for available commands.");
    }
  }
}

// ============================================
// SERIAL COMMAND EXAMPLES
// ============================================
// START              - Begin 1 metre sprint with pendulum balancing
// STOP               - Stop motors immediately
// RESET              - Reset LQR controller and stop motors
// RESET_MOTOR        - Reset motor encoder to zero
// GAINS              - Display current gains
// SET 1000 1000 750 2625  - Update LQR gains
// HELP               - Show command menu
