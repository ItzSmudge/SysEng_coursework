/*
 * ============================================================================
 * INVERTED PENDULUM - LQR CONTROLLER
 * Evaluation A: Disturbance Rejection (Robustness Test)
 * ============================================================================
 * 
 * System: Mobile cart with inverted pendulum
 * Control: Linear Quadratic Regulator (LQR)
 * Test: Maintain stability at x=0, theta=0 under external disturbances
 * 
 * Hardware:
 * - Arduino UNO R4
 * - 2x Motoron Motor Drivers (addresses 16, 17)
 * - AS5048 Encoder (Pendulum angle)
 * - Pololu 25D Encoder (Cart position)
 * - 4x DC Motors with encoders
 * 
 * Performance Metrics:
 * - Settling time after disturbance
 * - Maximum angle deviation
 * - Position error
 * - Recovery success rate
 * 
 * ============================================================================
 */

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
// SYSTEM PARAMETERS (TUNE THESE TO MATCH YOUR HARDWARE)
// ============================================================================

// Physical parameters
const float M = 1.0;          // Cart mass (kg) - MEASURE YOUR CART
const float m = 0.3;          // Pendulum mass (kg) - MEASURE YOUR PENDULUM
const float L = 1.0;          // Pendulum length (m) - MEASURE (60-100cm range)
const float b = 0.2;          // Friction coefficient - TUNE AFTER TESTING
const float g = 9.81;         // Gravity (m/s^2)
const float I = (1.0/3.0) * m * L * L;  // Moment of inertia

// Encoder specifications
const float ANGLE_CPR = 4096.0;     // AS5048: 4096 counts per revolution
const float POSITION_CPR = 464.64;  // Pololu 25D: 48 CPR * 9.68 gear ratio / 2 (half resolution)

// Motor specifications
const int MAX_MOTOR_SPEED = 800;    // Motoron maximum speed command
const float VOLTAGE_MAX = 12.0;     // Battery voltage
const float WHEEL_RADIUS = 0.05;    // Wheel radius in meters (MEASURE YOUR WHEELS)
const float FORCE_TO_SPEED = MAX_MOTOR_SPEED / 50.0;  // Conversion factor (TUNE THIS)

// ============================================================================
// LQR CONTROLLER PARAMETERS
// ============================================================================

// State: [x, x_dot, theta, theta_dot]
// LQR Gains (computed offline - from your Python simulation)
float K[4] = {0.0, 0.0, 0.0, 0.0};  // Will be calculated in setup()

// Cost matrices (from your simulation)
float Q[4] = {10.0, 1.0, 100.0, 1.0};  // State cost weights
float R = 0.01;                         // Control effort cost

// ============================================================================
// STATE VARIABLES
// ============================================================================

// Current state
float x = 0.0;           // Cart position (m)
float x_dot = 0.0;       // Cart velocity (m/s)
float theta = 0.0;       // Pendulum angle (rad, 0 = upright)
float theta_dot = 0.0;   // Pendulum angular velocity (rad/s)

// Previous state (for derivative calculation)
float x_prev = 0.0;
float theta_prev = 0.0;
float x_dot_prev = 0.0;
float theta_dot_prev = 0.0;

// Target state
float x_target = 0.0;      // Target position (m)
float theta_target = 0.0;  // Target angle (rad, upright)

// Control output
float u = 0.0;  // Control force (N)

// ============================================================================
// TIMING VARIABLES
// ============================================================================

unsigned long currentTime = 0;
unsigned long previousTime = 0;
float dt = 0.001;  // Control loop time step (1ms = 1000Hz)
const unsigned long LOOP_TIME_US = 1000;  // 1000 microseconds = 1ms

// ============================================================================
// PERFORMANCE MONITORING
// ============================================================================

float max_angle_deviation = 0.0;
float max_position_error = 0.0;
unsigned long disturbance_time = 0;
unsigned long settling_time = 0;
bool disturbance_detected = false;
bool system_stable = true;

const float ANGLE_THRESHOLD = 0.05;     // 2.86 degrees - stability threshold
const float POSITION_THRESHOLD = 0.05;  // 5 cm - position threshold
const float SETTLING_THRESHOLD = 0.02;  // Settling criteria (radians)

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C for motor drivers
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
  mc_front.setMaxAcceleration(2, 2000);
  mc_front.setMaxDeceleration(2, 2000);
  mc_front.setMaxAcceleration(3, 2000);
  mc_front.setMaxDeceleration(3, 2000);
  
  mc_back.reinitialize();
  mc_back.disableCrc();
  mc_back.clearResetFlag();
  mc_back.setMaxAcceleration(2, 2000);
  mc_back.setMaxDeceleration(2, 2000);
  mc_back.setMaxAcceleration(3, 2000);
  mc_back.setMaxDeceleration(3, 2000);
  
  // Calculate LQR gains
  calculateLQRGains();
  
  // Print header
  Serial.println("============================================");
  Serial.println("INVERTED PENDULUM - LQR CONTROLLER");
  Serial.println("Evaluation A: Disturbance Rejection");
  Serial.println("============================================");
  Serial.print("LQR Gains: K = [");
  for (int i = 0; i < 4; i++) {
    Serial.print(K[i], 4);
    if (i < 3) Serial.print(", ");
  }
  Serial.println("]");
  Serial.println("--------------------------------------------");
  Serial.println("Commands: 'm' = metrics, 'r' = reset, 's' = stop");
  Serial.println("--------------------------------------------");
  Serial.println("Time(ms)\tx(m)\tx_dot\ttheta(deg)\ttheta_dot\tu(V)\tStatus");
  Serial.println("--------------------------------------------");
  
  // Wait for system to be manually placed upright
  Serial.println(">>> Place pendulum upright and press Enter <<<");
  while (!Serial.available()) {
    delay(100);
  }
  while (Serial.available()) Serial.read();
  
  // Zero encoders at starting position
  angleEncoder.setPosition(0);
  positionEncoder.setPosition(0);
  
  // Start timer
  previousTime = micros();
  
  Serial.println(">>> CONTROL STARTED <<<");
}

// ============================================================================
// MAIN CONTROL LOOP
// ============================================================================

void loop() {
  currentTime = micros();
  
  // Maintain fixed control loop frequency
  if (currentTime - previousTime >= LOOP_TIME_US) {
    dt = (currentTime - previousTime) / 1000000.0;  // Convert to seconds
    previousTime = currentTime;
    
    // 1. READ SENSORS
    readState();
    
    // 2. DETECT DISTURBANCES
    detectDisturbance();
    
    // 3. COMPUTE LQR CONTROL
    computeLQRControl();
    
    // 4. APPLY CONTROL TO MOTORS
    applyMotorControl();
    
    // 5. UPDATE PERFORMANCE METRICS
    updateMetrics();
    
    // 6. PRINT STATUS (every 100ms)
    static unsigned long lastPrint = 0;
    if (currentTime - lastPrint >= 100000) {
      printStatus();
      lastPrint = currentTime;
    }
    
    // 7. SAFETY CHECK
    if (abs(theta) > 0.5) {  // 28.6 degrees
      emergencyStop();
      Serial.println(">>> SAFETY STOP: Angle too large <<<");
      printFinalMetrics();
      while (1);  // Halt
    }
    
    // 8. Handle serial commands
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
  // Read encoder counts
  long currentAngleCount = angleEncoder.getPosition();
  long currentPositionCount = positionEncoder.getPosition();
  
  // Calculate position (meters) from wheel encoder
  // Position = (counts / CPR) * circumference
  x = (currentPositionCount / POSITION_CPR) * (2.0 * PI * WHEEL_RADIUS);
  
  // Calculate angle (radians, 0 = upright, positive = CCW when viewed from right)
  theta = (currentAngleCount / ANGLE_CPR) * 2.0 * PI;
  
  // Normalize angle to [-pi, pi]
  while (theta > PI) theta -= 2.0 * PI;
  while (theta < -PI) theta += 2.0 * PI;
  
  // Calculate velocities using finite differences with low-pass filter
  const float alpha = 0.7;  // Filter coefficient (0-1, higher = less filtering)
  
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
// LQR CONTROL COMPUTATION
// ============================================================================

void calculateLQRGains() {
  // LQR gain calculation for inverted pendulum
  // These are pre-computed gains based on your Python simulation
  
  // IMPORTANT: Run your Python LQR code to compute the optimal K matrix
  // Then paste those values here. The values below are placeholders!
  
  // For Q = diag([10, 1, 100, 1]), R = 0.01
  // State: [x, x_dot, theta, theta_dot]
  
  // OPTION 1: Use pre-computed gains from Python (RECOMMENDED)
  // TODO: Replace these with your actual computed values!
  K[0] = -31.6228;   // k_x - Position feedback
  K[1] = -20.0000;   // k_x_dot - Velocity damping
  K[2] = -316.2278;  // k_theta - Angle feedback (MOST IMPORTANT!)
  K[3] = -31.6228;   // k_theta_dot - Angular velocity damping
  
  // OPTION 2: Manual tuning (if LQR gains don't work well)
  // Start with these and tune empirically:
  /*
  K[0] = -5.0;    // Position feedback (keep small, focus on angle)
  K[1] = -3.0;    // Velocity damping
  K[2] = -80.0;   // Angle feedback (start here and increase if needed)
  K[3] = -15.0;   // Angular velocity damping
  */
}

void computeLQRControl() {
  // State error from target
  float e_x = x - x_target;
  float e_x_dot = x_dot - 0.0;
  float e_theta = theta - theta_target;
  float e_theta_dot = theta_dot - 0.0;
  
  // LQR control law: u = -K * e
  u = -(K[0] * e_x + K[1] * e_x_dot + K[2] * e_theta + K[3] * e_theta_dot);
  
  // Saturation limits (in Newtons)
  const float MAX_FORCE = 50.0;  // TUNE THIS based on your motors
  u = constrain(u, -MAX_FORCE, MAX_FORCE);
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================

void applyMotorControl() {
  // Convert control force to motor speed command
  // This is a simplified model - you may need to tune FORCE_TO_SPEED
  int motorSpeed = (int)(u * FORCE_TO_SPEED);
  motorSpeed = constrain(motorSpeed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  
  // Apply to all four motors (all wheels move together)
  // Positive speed = forward, negative = reverse
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
// DISTURBANCE DETECTION & PERFORMANCE MONITORING
// ============================================================================

void detectDisturbance() {
  // Detect sudden changes in angle or position (indicating disturbance)
  float angle_change_rate = abs(theta_dot);
  float position_change_rate = abs(x_dot);
  
  const float DISTURBANCE_ANGLE_THRESHOLD = 1.5;   // rad/s
  const float DISTURBANCE_POSITION_THRESHOLD = 0.3; // m/s
  
  // Mark disturbance when exceeding thresholds
  if ((angle_change_rate > DISTURBANCE_ANGLE_THRESHOLD || 
       position_change_rate > DISTURBANCE_POSITION_THRESHOLD) &&
      system_stable) {
    disturbance_detected = true;
    disturbance_time = millis();
    system_stable = false;
    Serial.println(">>> DISTURBANCE DETECTED <<<");
  }
}

void updateMetrics() {
  // Track maximum deviations since last disturbance
  float angle_deviation = abs(theta);
  float position_error = abs(x - x_target);
  
  if (angle_deviation > max_angle_deviation) {
    max_angle_deviation = angle_deviation;
  }
  
  if (position_error > max_position_error) {
    max_position_error = position_error;
  }
  
  // Check if system has settled after disturbance
  if (disturbance_detected && !system_stable) {
    if (angle_deviation < SETTLING_THRESHOLD && position_error < SETTLING_THRESHOLD) {
      system_stable = true;
      settling_time = millis() - disturbance_time;
      Serial.print(">>> SETTLED in ");
      Serial.print(settling_time);
      Serial.println(" ms <<<");
      disturbance_detected = false;
    }
  }
}

// ============================================================================
// STATUS REPORTING
// ============================================================================

void printStatus() {
  // Format: Time x x_dot theta theta_dot u status
  Serial.print(millis());
  Serial.print("\t");
  Serial.print(x, 4);
  Serial.print("\t");
  Serial.print(x_dot, 3);
  Serial.print("\t");
  Serial.print(theta * 57.2958, 2);  // Convert to degrees
  Serial.print("\t");
  Serial.print(theta_dot, 3);
  Serial.print("\t");
  
  // Convert control force to approximate voltage
  float voltage = (u / MAX_MOTOR_SPEED * FORCE_TO_SPEED) * VOLTAGE_MAX;
  Serial.print(voltage, 2);
  Serial.print("\t");
  
  if (system_stable) {
    Serial.println("STABLE");
  } else {
    Serial.println("RECOVERING");
  }
}

void printFinalMetrics() {
  Serial.println("\n============================================");
  Serial.println("PERFORMANCE METRICS");
  Serial.println("============================================");
  Serial.print("Max Angle Deviation: ");
  Serial.print(max_angle_deviation * 57.2958, 2);
  Serial.println(" degrees");
  Serial.print("Max Position Error: ");
  Serial.print(max_position_error * 100.0, 2);
  Serial.println(" cm");
  
  if (settling_time > 0) {
    Serial.print("Last Settling Time: ");
    Serial.print(settling_time);
    Serial.println(" ms");
  }
  
  Serial.println("============================================");
}

// ============================================================================
// SERIAL COMMANDS (for testing and tuning)
// ============================================================================

void handleSerialCommands() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    while (Serial.available()) Serial.read();  // Clear buffer
    
    switch (cmd) {
      case 'm':  // Print metrics
      case 'M':
        printFinalMetrics();
        break;
        
      case 'r':  // Reset metrics
      case 'R':
        max_angle_deviation = 0.0;
        max_position_error = 0.0;
        disturbance_detected = false;
        system_stable = true;
        settling_time = 0;
        Serial.println(">>> METRICS RESET <<<");
        break;
        
      case 's':  // Emergency stop
      case 'S':
        emergencyStop();
        Serial.println(">>> EMERGENCY STOP <<<");
        printFinalMetrics();
        while (1);
        break;
        
      case 'z':  // Zero position
      case 'Z':
        positionEncoder.setPosition(0);
        x = 0.0;
        x_prev = 0.0;
        Serial.println(">>> POSITION ZEROED <<<");
        break;
    }
  }
}
