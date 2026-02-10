#include <Wire.h>
#include <Motoron.h>
#include <RotaryEncoder.h>

// --- System Constants (From your simulation/specs) ---
const float GEAR_RATIO = 50.0; 
const float ENCODER_CPR = 64.0;
const float OUTPUT_CPR = ENCODER_CPR * GEAR_RATIO;
const float PIVOT_CPR = 1858.0; // Adjust based on your AS22/Pololu test results

// --- LQR Gains (Replace with values from your Python simulation) ---
// Order: [x, x_dot, theta, theta_dot]
const float K[4] = {-15.5, -12.2, 180.0, 25.5}; 

// --- Hardware Setup ---
MotoronI2C mc1(16); 
MotoronI2C mc2(17);
RotaryEncoder angleEnc(2, 3, RotaryEncoder::LatchMode::TWO03); // Pendulum
RotaryEncoder distEnc(4, 5, RotaryEncoder::LatchMode::TWO03);  // Wheel/Cart

// --- State Variables ---
float x = 0, x_dot = 0, theta = 0, theta_dot = 0;
long lastAnglePos = 0, lastDistPos = 0;
unsigned long lastMicros = 0;

// --- Filtering (Moving Average) ---
const int AVG_SIZE = 5;
float theta_buffer[AVG_SIZE];
int buffer_idx = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  mc1.reinitialize(); mc1.disableCrc(); mc1.clearResetFlag();
  mc2.reinitialize(); mc2.disableCrc(); mc2.clearResetFlag();

  // Attach Interrupts for real-time tracking
  attachInterrupt(digitalPinToInterrupt(2), []{ angleEnc.tick(); }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), []{ angleEnc.tick(); }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(4), []{ distEnc.tick(); }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(5), []{ distEnc.tick(); }, CHANGE);

  lastMicros = micros();
}

void loop() {
  unsigned long currentMicros = micros();
  float dt = (currentMicros - lastMicros) / 1000000.0;
  if (dt < 0.005) return; // Run loop at 200Hz

  // 1. Read Sensors & Convert to SI Units
  long currAnglePos = angleEnc.getPosition();
  long currDistPos = distEnc.getPosition();

  // Calculate Theta (radians) - 0 is upright
  float raw_theta = (currAnglePos / PIVOT_CPR) * 2.0 * PI;
  
  // 2. Simple Low-Pass Filter for Noise [cite: 215]
  theta_buffer[buffer_idx] = raw_theta;
  buffer_idx = (buffer_idx + 1) % AVG_SIZE;
  float filtered_theta = 0;
  for(int i=0; i<AVG_SIZE; i++) filtered_theta += theta_buffer[i];
  filtered_theta /= AVG_SIZE;

  // Calculate Positions and Velocities
  float new_x = (currDistPos / OUTPUT_CPR) * 0.20; // 0.20m wheel circumference?
  theta_dot = (filtered_theta - theta) / dt;
  x_dot = (new_x - x) / dt;
  
  theta = filtered_theta;
  x = new_x;
  lastMicros = currentMicros;

  // 3. LQR Control Law: u = -K * state [cite: 57]
  // Target state for Evaluation A: [0, 0, 0, 0]
  float control_effort = -(K[0]*x + K[1]*x_dot + K[2]*theta + K[3]*theta_dot);

  // 4. Map to Motoron Speed (-800 to 800)
  int motor_output = constrain((int)control_effort, -800, 800);

  // Safety: Stop if angle is too large (> 45 degrees)
  if (abs(theta) > 0.78) {
    stopMotors();
  } else {
    driveMotors(motor_output);
  }
}

void driveMotors(int speed) {
  mc1.setSpeed(2, speed); mc1.setSpeed(3, speed);
  mc2.setSpeed(2, speed); mc2.setSpeed(3, speed);
}

void stopMotors() {
  mc1.setSpeed(2, 0); mc1.setSpeed(3, 0);
  mc2.setSpeed(2, 0); mc2.setSpeed(3, 0);
}