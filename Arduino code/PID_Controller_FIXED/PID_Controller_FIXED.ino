// ============================================
// FIXED PID Controller for Inverted Pendulum
// Addresses: latency, derivative noise, gain tuning
// ============================================
#include <Motoron.h>

MotoronI2C mc1(16);
MotoronI2C mc2(17);

const int MAX_SPEED = 800;
const int MIN_SPEED = -800;
const int PWM_to_motor = 1;
const int MOTORS_FORWARD = -1;

// ============================================
// ENCODER CONFIGURATION
// ============================================
const int encoderPinA = 10;
const int encoderPinB = 11;
const int encoderIndex = 4;
const int COUNTS_PER_REV = 4096;
volatile long encoderCount = 0;
volatile bool indexFound = false;
volatile int lastStateA = 0;
volatile int lastStateB = 0;

// ============================================
// FIXED: Lightweight Moving Average Filter
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
    windowSize = size;
    buffer = new float[windowSize];
    bufferIndex = 0;
    bufferCount = 0;
    sum = 0.0;
    for (int i = 0; i < windowSize; i++) {
      buffer[i] = 0.0;
    }
  }

  ~MovingAverageFilter() {
    delete[] buffer;
  }

  float apply(float value) {
    sum -= buffer[bufferIndex];
    buffer[bufferIndex] = value;
    sum += value;
    bufferIndex = (bufferIndex + 1) % windowSize;
    if (bufferCount < windowSize) {
      bufferCount++;
    }
    return sum / bufferCount;
  }

  void reset() {
    bufferIndex = 0;
    bufferCount = 0;
    sum = 0.0;
    for (int i = 0; i < windowSize; i++) {
      buffer[i] = 0.0;
    }
  }
};

// ============================================
// FIXED: PID Controller Class
// ============================================
class PIDController {
private:
  // PID gains (now tuned for stability)
  float kp_theta, ki_theta, kd_theta;
  float kp_x, ki_x, kd_x;
  
  // Control parameters
  float dt;
  float integral_theta;
  float integral_x;
  float i_limit_theta;
  float i_limit_x;
  
  // Filtering (now MUCH lighter)
  bool filter_enabled;
  MovingAverageFilter* filter_theta;
  MovingAverageFilter* filter_x;
  
  // State tracking
  float prev_theta;
  float prev_x;
  float theta_dot;
  float x_dot;
  unsigned long lastControlTime;

public:
  PIDController(float kp_t = 300.0, float kd_t = 1000.0, float ki_t = 10.0,
                float kp_pos = 0.0, float kd_pos = 0.0, float ki_pos = 0.0,
                float timestep = 0.01, float i_lim_t = 2.0, float i_lim_pos = 0.5,
                int window_size = 5, bool filter_en = true) {
    kp_theta = kp_t;
    ki_theta = ki_t;
    kd_theta = kd_t;
    kp_x = kp_pos;
    ki_x = ki_pos;
    kd_x = kd_pos;
    dt = timestep;
    i_limit_theta = i_lim_t;
    i_limit_x = i_lim_pos;
    integral_theta = 0.0;
    integral_x = 0.0;
    prev_theta = 0.0;
    prev_x = 0.0;
    theta_dot = 0.0;
    x_dot = 0.0;
    filter_enabled = filter_en;
    
    if (filter_enabled) {
      filter_theta = new MovingAverageFilter(window_size);
      filter_x = new MovingAverageFilter(window_size);
    } else {
      filter_theta = nullptr;
      filter_x = nullptr;
    }
    lastControlTime = millis();
  }

  ~PIDController() {
    if (filter_theta) delete filter_theta;
    if (filter_x) delete filter_x;
  }

  float getAction(float x, float theta, float target_x = 0.0) {
    // FIX #1: Use nominal dt ALWAYS, not variable actual_dt
    // This prevents derivative noise from inconsistent timesteps
    float actual_dt = dt;

    // Apply filters if enabled (now with window_size=5 instead of 50)
    float filtered_theta = theta;
    float filtered_x = x;
    if (filter_enabled) {
      filtered_theta = filter_theta->apply(theta);
      filtered_x = filter_x->apply(x);
    }

    // FIX #2: Calculate derivatives using nominal dt
    theta_dot = (filtered_theta - prev_theta) / actual_dt;
    x_dot = (filtered_x - prev_x) / actual_dt;
    prev_theta = filtered_theta;
    prev_x = filtered_x;

    // Position (x) control - outer loop
    float error_x = target_x - filtered_x;
    integral_x += error_x * actual_dt;
    integral_x = constrain(integral_x, -i_limit_x, i_limit_x);

    // Desired angle from position error
    float desired_theta = (kp_x * error_x + ki_x * integral_x - kd_x * x_dot);

    // FIX #3: Angle (theta) control - inner loop with strong damping
    // Key: kd_theta should be MUCH larger than kp_theta (opposite of typical PID)
    float error_theta = desired_theta - filtered_theta;
    integral_theta += error_theta * actual_dt;
    integral_theta = constrain(integral_theta, -i_limit_theta, i_limit_theta);

    // Calculate force with inverted damping ratio
    float force = (kp_theta * error_theta + ki_theta * integral_theta - kd_theta * theta_dot);
    return force;
  }

  void reset() {
    integral_theta = 0.0;
    integral_x = 0.0;
    prev_theta = 0.0;
    prev_x = 0.0;
    theta_dot = 0.0;
    x_dot = 0.0;
    if (filter_enabled) {
      filter_theta->reset();
      filter_x->reset();
    }
    lastControlTime = millis();
  }

  void setGains(float kp_t, float kd_t, float ki_t, float kp_pos, float kd_pos, float ki_pos) {
    kp_theta = kp_t;
    ki_theta = ki_t;
    kd_theta = kd_t;
    kp_x = kp_pos;
    ki_x = ki_pos;
    kd_x = kd_pos;
  }

  // Getters for debugging
  float getIntegralTheta() { return integral_theta; }
  float getIntegralX() { return integral_x; }
  float getThetaDot() { return theta_dot; }
  float getXDot() { return x_dot; }
};

// ============================================
// GLOBAL OBJECTS
// ============================================
// FIX: Corrected gains for better performance
//      kp_theta=300 (reduced from 1000)
//      kd_theta=1000 (increased from 500) — now 3x stronger!
//      ki_theta=10 (reduced from 30)
//      window_size=5 (reduced from 50) — less latency
PIDController pid(
  300.0,    // kp_theta
  1000.0,   // kd_theta (STRONG DAMPING)
  10.0,     // ki_theta
  0.0,      // kp_x
  0.0,      // kd_x
  0.0,      // ki_x
  0.01,     // dt
  2.0,      // i_limit_theta (reduced from 5.0)
  0.5,      // i_limit_x (reduced from 1.0)
  5,        // window_size (reduced from 50!)
  true      // filter_enabled
);

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  Wire1.begin();

  // Configure encoder pins
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(encoderIndex, INPUT);

  lastStateA = digitalRead(encoderPinA);
  lastStateB = digitalRead(encoderPinB);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderIndex), handleIndex, RISING);

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

  Serial.println("========================================");
  Serial.println("FIXED PID CONTROLLER - INVERTED PENDULUM");
  Serial.println("========================================");
  Serial.println("Changes made:");
  Serial.println("  • Reduced filter window: 50 → 5 samples");
  Serial.println("  • Fixed derivative: nominal dt only");
  Serial.println("  • Inverted gains: kd >> kp (1000 vs 300)");
  Serial.println("  • Lowered integral limits");
  Serial.println("========================================");
  Serial.println();

  pid.reset();
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  static unsigned long lastControl = 0;
  static unsigned long lastPrint = 0;
  unsigned long currentTime = millis();
  int motorSpeed = 0;

  // Control loop at 100Hz (10ms)
  if (currentTime - lastControl >= 10) {
    float theta = getAngleRadians();
    float x = 0.0;  // Placeholder
    float target_x = 0.0;

    float force = pid.getAction(x, theta, target_x);
    motorSpeed = forceToMotorSpeed(force);
    setMotorSpeed(motorSpeed);

    lastControl = currentTime;
  }

  // Print debug info at 10Hz (100ms)
  if (currentTime - lastPrint >= 100) {
    float theta = getAngleRadians();
    Serial.print("Theta: ");
    Serial.print(theta, 4);
    Serial.print(" rad | ");
    Serial.print(theta * (180.0 / 3.14159), 2);
    Serial.print(" Deg | Theta_dot: ");
    Serial.print(pid.getThetaDot(), 4);
    Serial.print(" | Int_theta: ");
    Serial.print(pid.getIntegralTheta(), 4);
    Serial.print(" | Motor: ");
    Serial.print(motorSpeed);
    Serial.println();
    lastPrint = currentTime;
  }

  handleSerialCommands();
}

// ============================================
// HELPER FUNCTIONS
// ============================================
float getAngleRadians() {
  long count = encoderCount % COUNTS_PER_REV;
  float angle = (count * 2.0 * PI) / COUNTS_PER_REV;
  if (angle > PI) {
    angle -= 2.0 * PI;
  }
  return angle;
}

int forceToMotorSpeed(float force) {
  int speed = (int)constrain(force * PWM_to_motor, MIN_SPEED, MAX_SPEED);
  return MOTORS_FORWARD * speed;
}

void setMotorSpeed(int speed) {
  mc1.setSpeed(2, speed);
  mc1.setSpeed(3, speed);
  mc2.setSpeed(2, speed);
  mc2.setSpeed(3, speed);
}

void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "RESET") {
      pid.reset();
      Serial.println("PID reset");
    } else if (command.startsWith("SET ")) {
      float gains[6];
      int idx = 0;
      int lastSpace = 3;
      for (int i = 4; i < command.length() && idx < 6; i++) {
        if (command[i] == ' ' || i == command.length() - 1) {
          if (i == command.length() - 1) i++;
          gains[idx++] = command.substring(lastSpace + 1, i).toFloat();
          lastSpace = i;
        }
      }
      if (idx == 6) {
        pid.setGains(gains[0], gains[1], gains[2], gains[3], gains[4], gains[5]);
        Serial.println("Gains updated: SET kp_t kd_t ki_t kp_x kd_x ki_x");
      }
    } else if (command == "STOP") {
      setMotorSpeed(0);
      Serial.println("Motors stopped");
    }
  }
}

// ============================================
// INTERRUPT SERVICE ROUTINES
// ============================================
void updateEncoder() {
  int currentStateA = digitalRead(encoderPinA);
  int currentStateB = digitalRead(encoderPinB);

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

void handleIndex() {
  encoderCount = 0;
  indexFound = true;
}

// ============================================
// TUNING GUIDE
// ============================================
// Start with the default gains above. If you see:
//
// PROBLEM: Still oscillating/overshooting
//   → Reduce kp_theta (try 200, 150)
//   → Increase kd_theta more (try 1500, 2000)
//
// PROBLEM: Slow response / laggy
//   → Reduce kd_theta (try 800)
//   → Check if control loop is actually running at 100Hz
//
// PROBLEM: Integration drift
//   → Reduce ki_theta (try 5, 2, 0)
//   → Reduce i_limit_theta (try 1.0, 0.5)
//
// TUNING COMMAND:
//   SET 300 1000 10 0 0 0    (restore defaults)
//   SET 200 1500 5 0 0 0     (try if still oscillating)
//   SET 400 800 10 0 0 0     (try if too sluggish)
