#include <Motoron.h>
#include <RotaryEncoder.h>

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// Motor Drivers
MotoronI2C mc_front(16);  // Front motors (address 16)
MotoronI2C mc_back(17);   // Back motors (address 17)

// Encoder Pins
const int ANGLE_ENCODER_A = 2;     // AS5048 - Channel A (Pendulum angle)
const int ANGLE_ENCODER_B = 3;     // AS5048 - Channel B
const int POSITION_ENCODER_A = 18; // Pololu 25D - Channel A (Cart position)
const int POSITION_ENCODER_B = 19; // Pololu 25D - Channel B

// Encoder Objects
RotaryEncoder angleEncoder(ANGLE_ENCODER_A, ANGLE_ENCODER_B, RotaryEncoder::LatchMode::TWO03);
RotaryEncoder positionEncoder(POSITION_ENCODER_A, POSITION_ENCODER_B, RotaryEncoder::LatchMode::TWO03);

// ============================================================================
// SYSTEM PARAMETERS
// ============================================================================

// Physical parameters (ADJUST TO YOUR SYSTEM)
const float M = 1.0;          // Cart mass (kg)
const float m = 0.3;          // Pendulum mass (kg)
const float L = 1.0;          // Pendulum length (m)
const float b = 0.2;          // Friction coefficient
const float g = 9.81;         // Gravity (m/s^2)
const float I = (1.0/3.0) * m * L * L;  // Moment of inertia

// Encoder specifications
const float ANGLE_CPR = 4096.0;     // AS5048: 4096 counts per revolution
const float POSITION_CPR = 464.64;  // Pololu 25D: 48 CPR * 9.68 gear ratio / 2

// Motor specifications
const int MAX_MOTOR_SPEED = 800;    // Motoron maximum speed command
const float VOLTAGE_MAX = 12.0;     // Battery voltage
const float WHEEL_RADIUS = 0.05;    // Wheel radius in meters
const float FORCE_TO_SPEED = MAX_MOTOR_SPEED / 50.0;  // Conversion factor

// ============================================================================
// CONTROL STATE MACHINE
// ============================================================================

enum ControlMode {
  WAITING,      // Waiting for start command
  LAUNCHING,    // Accelerating to lift pendulum
  STABILIZING,  // LQR control near upright
  BALANCED,     // Successfully balanced
  FAILED        // Recovery failed
};

ControlMode controlMode = WAITING;

// ============================================================================
// LQR CONTROLLER PARAMETERS
// ============================================================================

// LQR Gains
float K[4] = {0.0, 0.0, 0.0, 0.0};

// Cost matrices (use same as Evaluation A)
float Q[4] = {10.0, 1.0, 100.0, 1.0};
float R = 0.01;

// ============================================================================
// SWING-UP PARAMETERS
// ============================================================================

// Starting angle (set based on V-block position)
float theta_start = 0.0;  // Will be read from encoder at start
float theta_liftoff = 0.0;  // Angle when pendulum leaves holder

// Swing-up control parameters (TUNE THESE)
const float LAUNCH_ACCELERATION = 0.8 * MAX_MOTOR_SPEED;  // Initial burst
const float LAUNCH_DURATION_MS = 200;  // How long to accelerate (ms)

// Switch to stabilization when angle is within this range
const float STABILIZATION_THRESHOLD = 0.35;  // ~20 degrees from upright

// ============================================================================
// STATE VARIABLES
// ============================================================================

float x = 0.0;
float x_dot = 0.0;
float theta = 0.0;
float theta_dot = 0.0;

float x_prev = 0.0;
float theta_prev = 0.0;
float x_dot_prev = 0.0;
float theta_dot_prev = 0.0;

float x_target = 0.0;
float theta_target = 0.0;

float u = 0.0;

// ============================================================================
// TIMING VARIABLES
// ============================================================================

unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long launchStartTime = 0;
unsigned long recoveryStartTime = 0;
float dt = 0.001;
const unsigned long LOOP_TIME_US = 1000;

// ============================================================================
// PERFORMANCE MONITORING
// ============================================================================

float max_angle_reached = 0.0;
unsigned long time_to_recovery = 0;
bool recovery_successful = false;
int attempt_number = 1;

const float SUCCESS_ANGLE = 0.1;  // Within 5.7 degrees of upright = success
const float FAILURE_ANGLE = 0.7;  // Beyond 40 degrees = likely failed

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C
  Wire.begin();
  
  // Setup encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ANGLE_ENCODER_A), updateAngleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ANGLE_ENCODER_B), updateAngleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(POSITION_ENCODER_A), updatePositionEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(POSITION_ENCODER_B), updatePositionEncoder, CHANGE);
  
  // Initialize motor drivers
  delay(100);
  
  mc_front.reinitialize();
  mc_front.disableCrc();
  mc_front.clearResetFlag();
  mc_front.setMaxAcceleration(2, 3000);  // Higher acceleration for swing-up
  mc_front.setMaxDeceleration(2, 3000);
  mc_front.setMaxAcceleration(3, 3000);
  mc_front.setMaxDeceleration(3, 3000);
  
  mc_back.reinitialize();
  mc_back.disableCrc();
  mc_back.clearResetFlag();
  mc_back.setMaxAcceleration(2, 3000);
  mc_back.setMaxDeceleration(2, 3000);
  mc_back.setMaxAcceleration(3, 3000);
  mc_back.setMaxDeceleration(3, 3000);
  
  // Calculate LQR gains
  calculateLQRGains();
  
  // Print header
  Serial.println("============================================");
  Serial.println("INVERTED PENDULUM - LQR CONTROLLER");
  Serial.println("Evaluation B: Deep Fall Recovery");
  Serial.println("============================================");
  Serial.print("LQR Gains: K = [");
  for (int i = 0; i < 4; i++) {
    Serial.print(K[i], 4);
    if (i < 3) Serial.print(", ");
  }
  Serial.println("]");
  Serial.println("--------------------------------------------");
  Serial.println("Commands:");
  Serial.println("  'g' = Start recovery attempt");
  Serial.println("  'r' = Reset for new attempt");
  Serial.println("  's' = Emergency stop");
  Serial.println("  'a' = Adjust V-block angle");
  Serial.println("--------------------------------------------");
  
  // Wait and read starting angle
  Serial.println("\n>>> Place pendulum on V-block <<<");
  Serial.println(">>> Press 'g' to start when ready <<<");
  
  delay(1000);
  
  // Zero position encoder
  positionEncoder.setPosition(0);
  
  previousTime = micros();
}

// ============================================================================
// MAIN CONTROL LOOP
// ============================================================================

void loop() {
  currentTime = micros();
  
  // Maintain fixed control loop frequency
  if (currentTime - previousTime >= LOOP_TIME_US) {
    dt = (currentTime - previousTime) / 1000000.0;
    previousTime = currentTime;
    
    // 1. READ SENSORS
    readState();
    
    // 2. STATE MACHINE CONTROL
    switch (controlMode) {
      case WAITING:
        // Do nothing, wait for start command
        u = 0;
        applyMotorControl();
        break;
        
      case LAUNCHING:
        swingUpControl();
        break;
        
      case STABILIZING:
        stabilizeControl();
        break;
        
      case BALANCED:
        // Keep balancing
        stabilizeControl();
        break;
        
      case FAILED:
        emergencyStop();
        break;
    }
    
    // 3. MONITOR PROGRESS
    updatePerformanceMetrics();
    
    // 4. PRINT STATUS (every 50ms for fast updates during launch)
    static unsigned long lastPrint = 0;
    if (currentTime - lastPrint >= 50000) {
      printStatus();
      lastPrint = currentTime;
    }
    
    // 5. SAFETY CHECK
    if (abs(theta) > 1.0 && controlMode != WAITING) {  // 57 degrees
      controlMode = FAILED;
      Serial.println(">>> FAILED: Angle too large <<<");
      emergencyStop();
      printAttemptResult();
    }
    
    // 6. Handle commands
    handleSerialCommands();
  }
}

// ============================================================================
// ENCODER INTERRUPT HANDLERS
// ============================================================================

void updateAngleEncoder() {
  angleEncoder.tick();
}

void updatePositionEncoder() {
  positionEncoder.tick();
}

// ============================================================================
// STATE ESTIMATION
// ============================================================================

void readState() {
  long currentAngleCount = angleEncoder.getPosition();
  long currentPositionCount = positionEncoder.getPosition();
  
  // Calculate position and angle
  x = (currentPositionCount / POSITION_CPR) * (2.0 * PI * WHEEL_RADIUS);
  theta = (currentAngleCount / ANGLE_CPR) * 2.0 * PI;
  
  // Normalize angle to [-pi, pi]
  while (theta > PI) theta -= 2.0 * PI;
  while (theta < -PI) theta += 2.0 * PI;
  
  // Calculate velocities with filtering
  const float alpha = 0.7;
  
  float x_dot_raw = (x - x_prev) / dt;
  float theta_dot_raw = (theta - theta_prev) / dt;
  
  x_dot = alpha * x_dot_raw + (1.0 - alpha) * x_dot_prev;
  theta_dot = alpha * theta_dot_raw + (1.0 - alpha) * theta_dot_prev;
  
  // Update previous values
  x_prev = x;
  theta_prev = theta;
  x_dot_prev = x_dot;
  theta_dot_prev = theta_dot;
}

// ============================================================================
// CONTROL STRATEGIES
// ============================================================================

void calculateLQRGains() {
  // Same gains as Evaluation A
  // TODO: Replace with your computed values
  K[0] = -31.6228;   // k_x
  K[1] = -20.0000;   // k_x_dot
  K[2] = -316.2278;  // k_theta
  K[3] = -31.6228;   // k_theta_dot
  
  // Alternative manual tuning:
  /*
  K[0] = -5.0;
  K[1] = -3.0;
  K[2] = -80.0;
  K[3] = -15.0;
  */
}

void swingUpControl() {
  unsigned long elapsed = millis() - launchStartTime;
  
  // Phase 1: Initial burst acceleration
  if (elapsed < LAUNCH_DURATION_MS) {
    // Apply maximum acceleration in direction to lift pendulum
    // Direction depends on which side pendulum is leaning
    int direction = (theta_start > 0) ? -1 : 1;
    u = direction * LAUNCH_ACCELERATION;
    applyMotorControl();
  }
  // Phase 2: Check if close enough to switch to stabilization
  else {
    if (abs(theta) < STABILIZATION_THRESHOLD) {
      controlMode = STABILIZING;
      Serial.println(">>> Switching to STABILIZING <<<");
      theta_liftoff = theta;
    }
    // Otherwise continue with energy-based swing-up
    else {
      energyBasedSwingUp();
    }
  }
}

void energyBasedSwingUp() {
  // Energy-based swing-up controller
  // Goal: pump energy into system to bring pendulum near upright
  
  // Calculate total energy
  float E_kinetic = 0.5 * m * L * L * theta_dot * theta_dot;
  float E_potential = m * g * L * (cos(theta) - 1.0);
  float E_total = E_kinetic + E_potential;
  
  // Desired energy (at upright position with zero velocity)
  float E_desired = 0.0;
  float E_error = E_desired - E_total;
  
  // Control law: push in direction that increases energy
  // u = k_e * E_error * sign(theta_dot * cos(theta))
  float k_e = 10.0;  // Energy gain (TUNE THIS)
  
  if (abs(theta_dot) > 0.1) {  // Only pump when moving
    u = k_e * E_error * sign(theta_dot * cos(theta));
  } else {
    u = 0.0;
  }
  
  // Saturation
  u = constrain(u, -MAX_MOTOR_SPEED / 2, MAX_MOTOR_SPEED / 2);
  applyMotorControl();
  
  // Check if close enough to switch to stabilization
  if (abs(theta) < STABILIZATION_THRESHOLD) {
    controlMode = STABILIZING;
    Serial.println(">>> Switching to STABILIZING <<<");
  }
}

void stabilizeControl() {
  // Standard LQR control
  float e_x = x - x_target;
  float e_x_dot = x_dot - 0.0;
  float e_theta = theta - theta_target;
  float e_theta_dot = theta_dot - 0.0;
  
  u = -(K[0] * e_x + K[1] * e_x_dot + K[2] * e_theta + K[3] * e_theta_dot);
  
  const float MAX_FORCE = 50.0;
  u = constrain(u, -MAX_FORCE, MAX_FORCE);
  applyMotorControl();
  
  // Check if successfully balanced
  if (abs(theta) < SUCCESS_ANGLE && abs(theta_dot) < 0.5) {
    if (controlMode == STABILIZING) {
      controlMode = BALANCED;
      recovery_successful = true;
      time_to_recovery = millis() - recoveryStartTime;
      Serial.println(">>> SUCCESS: Pendulum balanced! <<<");
      printAttemptResult();
    }
  }
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================

void applyMotorControl() {
  int motorSpeed = (int)(u * FORCE_TO_SPEED);
  motorSpeed = constrain(motorSpeed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  
  mc_front.setSpeed(2, motorSpeed);
  mc_front.setSpeed(3, motorSpeed);
  mc_back.setSpeed(2, motorSpeed);
  mc_back.setSpeed(3, motorSpeed);
}

void emergencyStop() {
  mc_front.setSpeed(2, 0);
  mc_front.setSpeed(3, 0);
  mc_back.setSpeed(2, 0);
  mc_back.setSpeed(3, 0);
}

// ============================================================================
// PERFORMANCE MONITORING
// ============================================================================

void updatePerformanceMetrics() {
  float angle_magnitude = abs(theta);
  
  if (angle_magnitude > max_angle_reached) {
    max_angle_reached = angle_magnitude;
  }
}

void printAttemptResult() {
  Serial.println("\n============================================");
  Serial.print("ATTEMPT #");
  Serial.println(attempt_number);
  Serial.println("============================================");
  Serial.print("Starting Angle: ");
  Serial.print(theta_start * 57.2958, 2);
  Serial.println(" degrees");
  
  if (recovery_successful) {
    Serial.println("Result: SUCCESS");
    Serial.print("Time to Recovery: ");
    Serial.print(time_to_recovery);
    Serial.println(" ms");
  } else {
    Serial.println("Result: FAILED");
  }
  
  Serial.print("Max Angle Reached: ");
  Serial.print(max_angle_reached * 57.2958, 2);
  Serial.println(" degrees");
  Serial.println("============================================\n");
}

// ============================================================================
// STATUS REPORTING
// ============================================================================

void printStatus() {
  Serial.print(millis());
  Serial.print("\t");
  Serial.print(x, 4);
  Serial.print("\t");
  Serial.print(x_dot, 3);
  Serial.print("\t");
  Serial.print(theta * 57.2958, 2);
  Serial.print("\t");
  Serial.print(theta_dot, 3);
  Serial.print("\t");
  
  float voltage = (u / MAX_MOTOR_SPEED * FORCE_TO_SPEED) * VOLTAGE_MAX;
  Serial.print(voltage, 2);
  Serial.print("\t");
  
  // Print mode
  switch (controlMode) {
    case WAITING: Serial.println("WAITING"); break;
    case LAUNCHING: Serial.println("LAUNCHING"); break;
    case STABILIZING: Serial.println("STABILIZING"); break;
    case BALANCED: Serial.println("BALANCED"); break;
    case FAILED: Serial.println("FAILED"); break;
  }
}

// ============================================================================
// SERIAL COMMANDS
// ============================================================================

void handleSerialCommands() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    while (Serial.available()) Serial.read();
    
    switch (cmd) {
      case 'g':  // GO - start recovery attempt
      case 'G':
        if (controlMode == WAITING) {
          // Read starting angle
          theta_start = theta;
          Serial.print(">>> STARTING from angle: ");
          Serial.print(theta_start * 57.2958, 2);
          Serial.println(" degrees <<<");
          
          // Reset metrics
          max_angle_reached = abs(theta_start);
          recovery_successful = false;
          time_to_recovery = 0;
          
          // Start launch
          controlMode = LAUNCHING;
          launchStartTime = millis();
          recoveryStartTime = millis();
        }
        break;
        
      case 'r':  // RESET - prepare for new attempt
      case 'R':
        emergencyStop();
        controlMode = WAITING;
        attempt_number++;
        
        // Zero position
        positionEncoder.setPosition(0);
        x = 0.0;
        x_prev = 0.0;
        
        Serial.println("\n>>> RESET - Place pendulum on V-block <<<");
        Serial.println(">>> Press 'g' to start when ready <<<");
        break;
        
      case 's':  // STOP
      case 'S':
        emergencyStop();
        controlMode = FAILED;
        Serial.println(">>> EMERGENCY STOP <<<");
        printAttemptResult();
        break;
        
      case 'a':  // Adjust angle info
      case 'A':
        Serial.print("Current angle: ");
        Serial.print(theta * 57.2958, 2);
        Serial.println(" degrees");
        Serial.println("Adjust V-block and press 'g' to start");
        break;
    }
  }
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

float sign(float x) {
  if (x > 0) return 1.0;
  if (x < 0) return -1.0;
  return 0.0;
}
