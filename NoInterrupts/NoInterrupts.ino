// Pololu 37D Motor (4862) with Motoron Shield
// Manual Quadrature Encoder Reading using Basic Electronics Principles
// No external encoder library needed!

#include <Motoron.h>

// ============ MOTORON CONFIGURATION ============
MotoronI2C mc;
const uint8_t MOTOR_NUM = 2;

// ============ ENCODER PINS ============
const int ENCODER_A = 3;      // Channel A (Yellow wire)
const int ENCODER_B = 2;      // Channel B (White wire)

// ============ ENCODER SPECIFICATIONS ============
const int ENCODER_CPR = 64;           // Counts per revolution at motor shaft
const float GEAR_RATIO = 50.0;        // CHANGE to your motor's gear ratio
const float OUTPUT_CPR = ENCODER_CPR * GEAR_RATIO;  // Counts at output shaft

// ============ ENCODER STATE VARIABLES ============
volatile long encoderCount = 0;       // Total encoder counts
int lastStateA = 0;                   // Previous state of channel A
int lastStateB = 0;                   // Previous state of channel B

// ============ CONTROL VARIABLES ============
int16_t motorSpeed = 0;
long targetPosition = 0;
bool positionMode = false;

void setup() {
  Serial.begin(115200);
  
  // Setup encoder pins with pull-up resistors
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  
  // Read initial encoder states
  lastStateA = digitalRead(ENCODER_A);
  lastStateB = digitalRead(ENCODER_B);
  
  // Initialize Motoron
  Wire.begin();
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();
  mc.setMaxAcceleration(MOTOR_NUM, 2000);
  mc.setMaxDeceleration(MOTOR_NUM, 2000);
  
  Serial.println("========================================");
  Serial.println("Pololu Motor - Quadrature Encoder");
  Serial.println("========================================");
  Serial.println("Encoder Principle: Quadrature Decoding");
  Serial.println("  - 2 channels (A & B) 90° out of phase");
  Serial.println("  - Direction from phase relationship");
  Serial.println("  - Position from counting edges");
  Serial.println("========================================");
  Serial.print("Motor Channel: ");
  Serial.println(MOTOR_NUM);
  Serial.print("Encoder CPR (motor): ");
  Serial.println(ENCODER_CPR);
  Serial.print("Gear Ratio: ");
  Serial.println(GEAR_RATIO, 1);
  Serial.print("Output CPR (wheel): ");
  Serial.println(OUTPUT_CPR, 1);
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  s<value>  - Set speed (-800 to 800)");
  Serial.println("  p<value>  - Move to position");
  Serial.println("  r         - Reset position to 0");
  Serial.println("  x         - Stop motor");
  Serial.println("  d         - Show debug info");
  Serial.println("========================================");
  Serial.println();
}

void loop() {
  // ============ QUADRATURE ENCODER READING ============
  // This MUST be called frequently to catch all state changes
  readQuadratureEncoder();
  
  // ============ STATUS AND CONTROL ============
  static long lastPosition = 0;
  static unsigned long lastPrint = 0;
  static unsigned long lastSpeedCalc = 0;
  static long lastSpeedPosition = 0;
  
  long currentPosition = encoderCount;
  
  // Calculate speed (RPM) every 100ms
  float rpm = 0;
  if (millis() - lastSpeedCalc >= 100) {
    long positionChange = currentPosition - lastSpeedPosition;
    // RPM = (counts / time) * (60 sec/min) / (counts/rev)
    // 600 = 60 seconds / 0.1 seconds
    rpm = (positionChange * 600.0) / OUTPUT_CPR;
    lastSpeedPosition = currentPosition;
    lastSpeedCalc = millis();
  }
  
  // Position control mode
  if (positionMode) {
    positionControl(currentPosition);
  }
  
  // Print status every 200ms
  if (millis() - lastPrint >= 200) {
    if (currentPosition != lastPosition || positionMode) {
      printStatus(currentPosition, rpm);
      lastPosition = currentPosition;
    }
    lastPrint = millis();
  }
  
  // Handle serial commands
  handleSerialCommands();
}

// ============================================================
// QUADRATURE ENCODER READING - BASIC ELECTRONICS PRINCIPLE
// ============================================================
/*
 * QUADRATURE ENCODING PRINCIPLE:
 * 
 * Two channels (A and B) generate square waves 90° out of phase
 * 
 * Forward (Clockwise) Rotation:
 *    A: ¯¯|__|¯¯|__|¯¯
 *    B: __|¯¯|__|¯¯|__
 *    When A changes, if B is different from A → Forward
 * 
 * Reverse (Counter-clockwise) Rotation:
 *    A: ¯¯|__|¯¯|__|¯¯
 *    B: |¯¯|__|¯¯|__|¯
 *    When A changes, if B is same as A → Reverse
 * 
 * State Transition Table:
 * Previous | Current | Direction
 * A  B     | A  B    | 
 * ---------|---------|----------
 * 0  0     | 0  1    | -1 (CCW)
 * 0  0     | 1  0    | +1 (CW)
 * 0  1     | 0  0    | +1 (CW)
 * 0  1     | 1  1    | -1 (CCW)
 * 1  0     | 0  0    | -1 (CCW)
 * 1  0     | 1  1    | +1 (CW)
 * 1  1     | 0  1    | +1 (CW)
 * 1  1     | 1  0    | -1 (CCW)
 */

void readQuadratureEncoder() {
  // Read current states
  int currentStateA = digitalRead(ENCODER_A);
  int currentStateB = digitalRead(ENCODER_B);
  
  // Check if channel A changed
  if (currentStateA != lastStateA) {
    // A changed - determine direction based on B
    if (currentStateA == currentStateB) {
      // A and B are same → Counter-clockwise
      encoderCount--;
    } else {
      // A and B are different → Clockwise
      encoderCount++;
    }
  }
  
  // Check if channel B changed
  if (currentStateB != lastStateB) {
    // B changed - determine direction based on A
    if (currentStateA == currentStateB) {
      // A and B are same → Clockwise
      encoderCount++;
    } else {
      // A and B are different → Counter-clockwise
      encoderCount--;
    }
  }
  
  // Update previous states
  lastStateA = currentStateA;
  lastStateB = currentStateB;
}

// ============================================================
// ALTERNATIVE: Full State Machine Decoder (More Robust)
// ============================================================
// Uncomment this version if you want more noise immunity

/*
void readQuadratureEncoder() {
  // Read current state as 2-bit value: A=bit1, B=bit0
  uint8_t currentState = (digitalRead(ENCODER_A) << 1) | digitalRead(ENCODER_B);
  
  // Combine with previous state: previous=bits[3:2], current=bits[1:0]
  static uint8_t lastState = 0;
  uint8_t combined = (lastState << 2) | currentState;
  
  // Decode using state transition lookup
  // Valid transitions increment or decrement counter
  switch (combined) {
    // Clockwise transitions
    case 0b0001:  // 00 -> 01
    case 0b0111:  // 01 -> 11
    case 0b1110:  // 11 -> 10
    case 0b1000:  // 10 -> 00
      encoderCount++;
      break;
      
    // Counter-clockwise transitions
    case 0b0010:  // 00 -> 10
    case 0b1011:  // 10 -> 11
    case 0b1101:  // 11 -> 01
    case 0b0100:  // 01 -> 00
      encoderCount--;
      break;
      
    // Invalid transitions (noise or missed states)
    // No change to counter
    default:
      break;
  }
  
  lastState = currentState;
}
*/

// ============ MOTOR CONTROL FUNCTIONS ============

void setMotorSpeed(int16_t speed) {
  speed = constrain(speed, -800, 800);
  motorSpeed = speed;
  mc.setSpeed(MOTOR_NUM, speed);
  if (speed == 0) {
    positionMode = false;
  }
}

void stopMotor() {
  mc.setSpeed(MOTOR_NUM, 0);
  motorSpeed = 0;
  positionMode = false;
}

void positionControl(long currentPosition) {
  // Simple proportional controller
  long error = targetPosition - currentPosition;
  
  // Tolerance: ±10 counts
  if (abs(error) < 10) {
    stopMotor();
    positionMode = false;
    Serial.println(">>> Target position reached! <<<");
  } else {
    // P-controller: speed proportional to error
    // Gain = 0.5 (divide by 2)
    int16_t speed = constrain(error / 2, -600, 600);
    
    // Minimum speed to overcome friction
    if (abs(speed) < 100) {
      speed = (speed > 0) ? 100 : -100;
    }
    
    setMotorSpeed(800);
  }
}

// ============ ANGLE CALCULATIONS ============

float calculateAngleDegrees(long position) {
  // Calculate angle at OUTPUT shaft (0-360°)
  long positionMod = position % (long)OUTPUT_CPR;
  float angle = positionMod * (360.0 / OUTPUT_CPR);
  
  // Ensure positive angle
  if (angle < 0) {
    angle += 360.0;
  }
  
  return angle;
}

float calculateAngleRadians(long position) {
  // Calculate angle in radians (0-2π)
  long positionMod = position % (long)OUTPUT_CPR;
  float angle = positionMod * (TWO_PI / OUTPUT_CPR);
  
  if (angle < 0) {
    angle += TWO_PI;
  }
  
  return angle;
}

float calculateRevolutions(long position) {
  // Total revolutions at OUTPUT shaft
  return (float)position / OUTPUT_CPR;
}

float calculateMotorRevolutions(long position) {
  // Total revolutions at MOTOR shaft (before gearbox)
  return (float)position / ENCODER_CPR;
}

// ============ STATUS DISPLAY ============

void printStatus(long position, float rpm) {
  float angle = calculateAngleDegrees(position);
  float revolutions = calculateRevolutions(position);
  
  Serial.print("Pos: ");
  Serial.print(position);
  Serial.print(" | Angle: ");
  Serial.print(angle, 1);
  Serial.print("° | Rev: ");
  Serial.print(revolutions, 3);
  Serial.print(" | Speed: ");
  Serial.print(motorSpeed);
  Serial.print(" | RPM: ");
  Serial.print(rpm, 1);
  
  if (positionMode) {
    Serial.print(" | Target: ");
    Serial.print(targetPosition);
    Serial.print(" | Error: ");
    Serial.print(targetPosition - position);
  }
  
  Serial.println();
}

void printDebugInfo() {
  Serial.println("\n========== DEBUG INFO ==========");
  Serial.print("Channel A state: ");
  Serial.println(digitalRead(ENCODER_A) ? "HIGH" : "LOW");
  Serial.print("Channel B state: ");
  Serial.println(digitalRead(ENCODER_B) ? "HIGH" : "LOW");
  Serial.print("Encoder count: ");
  Serial.println(encoderCount);
  Serial.print("Motor speed: ");
  Serial.println(motorSpeed);
  Serial.print("Angle: ");
  Serial.print(calculateAngleDegrees(encoderCount), 2);
  Serial.println("°");
  Serial.print("Motor revolutions: ");
  Serial.println(calculateMotorRevolutions(encoderCount), 3);
  Serial.print("Output revolutions: ");
  Serial.println(calculateRevolutions(encoderCount), 3);
  Serial.println("================================\n");
}

// ============ SERIAL COMMAND HANDLER ============

void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.length() == 0) return;
    
    char cmd = command.charAt(0);
    long value = command.substring(1).toInt();
    
    switch (cmd) {
      case 's':
      case 'S':
        // Speed control: s200 or s-200
        positionMode = false;
        setMotorSpeed(800);
        Serial.print("Speed set to: ");
        Serial.println(value);
        break;
        
      case 'p':
      case 'P':
        // Position control: p3200
        targetPosition = value;
        positionMode = true;
        Serial.print("Moving to position: ");
        Serial.print(value);
        Serial.print(" (");
        Serial.print(calculateAngleDegrees(value), 1);
        Serial.println("°)");
        break;
        
      case 'r':
      case 'R':
        // Reset encoder
        encoderCount = 0;
        Serial.println("Position reset to 0");
        break;
        
      case 'x':
      case 'X':
        // Stop motor
        stopMotor();
        Serial.println("Motor stopped");
        break;
        
      case 'd':
      case 'D':
        // Debug info
        printDebugInfo();
        break;
        
      case '?':
      case 'h':
      case 'H':
        // Help
        Serial.println("\n========== COMMANDS ==========");
        Serial.println("s<value>  - Set speed (-800 to 800)");
        Serial.println("            Example: s200, s-150");
        Serial.println("p<value>  - Move to position");
        Serial.println("            Example: p3200, p-1600");
        Serial.println("r         - Reset position to 0");
        Serial.println("x         - Stop motor");
        Serial.println("d         - Show debug info");
        Serial.println("?         - Show this help");
        Serial.println("==============================\n");
        break;
        
      default:
        Serial.println("Unknown command. Type ? for help");
        break;
    }
  }
}