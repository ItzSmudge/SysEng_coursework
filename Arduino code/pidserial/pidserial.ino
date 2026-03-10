// PID Controller with Moving Average Filter for Inverted Pendulum
// Compatible with AS22 Encoder (4096 counts per revolution)
// Motoron I2C Motor Driver Integration
// ============================================
// MOTORON I2C CONFIGURATION
// ============================================
#include <Motoron.h>

MotoronI2C mc1(16); // Front motor driver
MotoronI2C mc2(17); // Back motor driver

const int MAX_SPEED = 800;   // Motoron max speed is 800
const int MIN_SPEED = -800;  // Motoron min speed
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
    // Remove oldest value from sum
    sum -= buffer[bufferIndex];
    // Add new value
    buffer[bufferIndex] = value;
    sum += value;
    // Update index (circular buffer)
    bufferIndex = (bufferIndex + 1) % windowSize;
    // Update count until buffer is full
    if (bufferCount < windowSize) {
      bufferCount++;
    }
    // Return average
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
// PID CONTROLLER CLASS
// ============================================
class PIDController {
private:
  // PID gains
  float kp_theta, ki_theta, kd_theta;
  float kp_x, ki_x, kd_x;
  // Integration and limits
  float dt;
  float integral_theta;
  float integral_x;
  float i_limit_theta;
  float i_limit_x;
  // Filters
  bool filter_enabled;
  MovingAverageFilter* filter_theta;
  MovingAverageFilter* filter_x;
  // Previous state for derivative calculation
  float prev_theta;
  float prev_x;
  float theta_dot;
  float x_dot;
  unsigned long lastTime;

public:
  PIDController(float kp_t = 0.0, float kd_t = 0.0, float ki_t = 0.0,
                float kp_pos = 0.0, float kd_pos = 0.0, float ki_pos = 0.0,
                float timestep = 0.01, float i_lim_t = 10.0, float i_lim_pos = 10.0,
                int window_size = 10, bool filter_en = false) {
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
    lastTime = millis();
  }

  ~PIDController() {
    if (filter_theta) delete filter_theta;
    if (filter_x) delete filter_x;
  }

  float getAction(float x, float theta, float target_x = 0.0) {
    // Calculate actual dt
    unsigned long currentTime = millis();
    float actual_dt = (currentTime - lastTime) / 1000.0;
    if (actual_dt <= 0.0) actual_dt = dt;
    lastTime = currentTime;

    // Apply filters if enabled
    float filtered_theta = theta;
    float filtered_x = x;
    if (filter_enabled) {
      filtered_theta = filter_theta->apply(theta);
      filtered_x = filter_x->apply(x);
    }

    // Calculate derivatives
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

    // Angle (theta) control - inner loop
    float error_theta = desired_theta - filtered_theta;
    integral_theta += error_theta * actual_dt;
    integral_theta = constrain(integral_theta, -i_limit_theta, i_limit_theta);

    // Calculate force/torque
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
    lastTime = millis();
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
// Initialize PID with example gains (tune these for your system)
PIDController pid(
  10000.0,   // kp_theta
  3000.0,    // kd_theta
  300.0,    // ki_theta
  30.0,    // kp_x
  15.0,    // kd_x
  0.30,   // ki_x
  0.3,   // dt
  50.0,   // i_limit_theta
  10.0,   // i_limit_x
  25,      // window_size
  true    // filter_enabled
);

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  Wire1.begin(); // Join the I2C bus

  // Configure encoder pins
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(encoderIndex, INPUT);

  // Initialize encoder state
  lastStateA = digitalRead(encoderPinA);
  lastStateB = digitalRead(encoderPinB);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderIndex), handleIndex, RISING);

  // Give the drivers a moment to power up
  delay(100);

  mc1.setBus(&Wire1);
  mc2.setBus(&Wire1);

  // --- SETUP DRIVER 1 (Address 16) ---
  mc1.reinitialize();       // Resets internal state variables in the library
  mc1.disableCrc();         // Disable CRC for simple communication
  mc1.clearResetFlag();     // Clear the "I just turned on" flag so it runs
  mc1.disableCommandTimeout();
  
  Serial.println("Driver 1 (Addr 16) Initialized.");

  // --- SETUP DRIVER 2 (Address 17) ---
  mc2.reinitialize();
  mc2.disableCrc();
  mc2.clearResetFlag();
  mc2.disableCommandTimeout();
  Serial.println("Driver 2 (Addr 17) Initialized.");

  Serial.println("========================================");
  Serial.println("PID CONTROLLER WITH MOTORON DRIVERS");
  Serial.println("========================================");
  Serial.println("System ready. Starting control loop...");
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
    // Get current angle from encoder
    float theta = getAngleRadians();

    // Get position (replace with actual position sensor if available)
    // For now, using 0.0 as placeholder
    float x = 0.0;

    // Target position (can be changed via serial or other input)
    float target_x = 0.0;

    // Calculate control signal
    float force = pid.getAction(x, theta, target_x);

    // Convert force to motor speed (-800 to 800)
    motorSpeed = forceToMotorSpeed(force);

    // Apply motor commands
    setMotorSpeed(motorSpeed);

    lastControl = currentTime;
  }

  // Print debug info at 10Hz (100ms)
  if (currentTime - lastPrint >= 100) {
    float theta = getAngleRadians();
    Serial.print("Theta: ");
    Serial.print(theta, 4);
    Serial.print(" rad | ");
    Serial.print(theta*(180/3.14));
    Serial.print(" Deg | ");
    Serial.print("Theta_dot: ");
    Serial.print(pid.getThetaDot(), 4);
    Serial.print(" | ");
    Serial.print("Int_theta: ");
    Serial.print(pid.getIntegralTheta(), 4);
    Serial.print(" | ");
    Serial.print("Int_x: ");
    Serial.print(pid.getIntegralX(), 4);
    Serial.print(" | Control input: ");
    Serial.print(motorSpeed, 4);
    Serial.println();
    lastPrint = currentTime;
  }

  // Handle serial commands for tuning (optional)
  handleSerialCommands();
}

// ============================================
// HELPER FUNCTIONS
// ============================================
float getAngleRadians() {
  // Convert encoder count to radians
  // Assuming 0 is upright position
  long count = encoderCount % COUNTS_PER_REV;
  float angle = (count * 2.0 * PI) / COUNTS_PER_REV;

  // Normalize to [-PI, PI]
  if (angle > PI) {
    angle -= 2.0 * PI;
  }
  return angle;
}

int forceToMotorSpeed(float force) {
  // Convert force to motor speed in range [-800, 800]
  int speed = (int)constrain(force*PWM_to_motor, MIN_SPEED, MAX_SPEED);
  Serial.println(speed);
  return MOTORS_FORWARD * speed;
}

void setMotorSpeed(int speed) {
  // Set speed for both motors (front and back)
  // Motors 2 and 3 on each driver
  
  // Front driver (mc1)
  mc1.setSpeed(2, speed);
  mc1.setSpeed(3, speed);
  
  // Back driver (mc2)
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
      // Example: SET 50 5 0.1 1 0.5 0.01
      // Format: SET kp_theta kd_theta ki_theta kp_x kd_x ki_x
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
        Serial.println("Gains updated");
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
    if (currentStateB == LOW) {
      encoderCount++;
    } else {
      encoderCount--;
    }
  } else if (lastStateA == HIGH && currentStateA == LOW) {
    if (currentStateB == HIGH) {
      encoderCount++;
    } else {
      encoderCount--;
    }
  } else if (lastStateB == LOW && currentStateB == HIGH) {
    if (currentStateA == HIGH) {
      encoderCount++;
    } else {
      encoderCount--;
    }
  } else if (lastStateB == HIGH && currentStateB == LOW) {
    if (currentStateA == LOW) {
      encoderCount++;
    } else {
      encoderCount--;
    }
  }

  lastStateA = currentStateA;
  lastStateB = currentStateB;
}

void handleIndex() {
  encoderCount = 0;
  indexFound = true;
}

// RESET
// SET 50 5 0.1 1 0.5 0.01
// SET 1000 5 0.1 1 0.5 0.01
// STOP