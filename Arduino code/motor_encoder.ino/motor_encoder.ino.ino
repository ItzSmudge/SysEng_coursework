// Pololu 37D Motor with Encoder (4862) using Motoron Motor Shield
// Compatible with Motoron M3S256 and M3H256 Motor Controllers

#define ENCODER_DO_NOT_USE_INTERRUPTS  // Add this line FIRST!
#include <Motoron.h>
#include <Encoder.h>

// ============ MOTORON CONFIGURATION ============
// Create Motoron object
// For I2C: MotoronI2C mc;
// For Serial: MotoronSerial mc;
MotoronI2C mc;

// Motor channel (1, 2, or 3 depending on which port you're using)
const uint8_t MOTOR_NUM = 1;

// ============ ENCODER PINS ============
const int ENCODER_A = 2;      // Encoder Channel A (Yellow wire)
const int ENCODER_B = 3;      // Encoder Channel B (White wire)

// ============ ENCODER SPECIFICATIONS ============
const int ENCODER_CPR = 64;           // Encoder counts per revolution (at motor shaft)
const float GEAR_RATIO = 50.0;        // CHANGE THIS to your motor's gear ratio
const float OUTPUT_CPR = ENCODER_CPR * GEAR_RATIO;  // Counts at output shaft

// Create encoder object
Encoder motorEncoder(ENCODER_A, ENCODER_B);

// ============ CONTROL VARIABLES ============
int16_t motorSpeed = 0;       // -800 to +800 (Motoron speed range)
long targetPosition = 0;      // Target encoder position
bool positionMode = false;    // false = speed mode, true = position mode

void setup() {
  Serial.begin(115200);
  
  // Initialize Motoron
  Wire.begin();
  
  // Reset Motoron to default settings
  mc.reinitialize();
  mc.disableCrc();
  
  // Clear any existing errors
  mc.clearResetFlag();
  
  // Set max acceleration and deceleration (optional)
  // Values from 0 to 200 (higher = faster acceleration)
  mc.setMaxAcceleration(MOTOR_NUM, 100);
  mc.setMaxDeceleration(MOTOR_NUM, 100);
  
  Serial.println("========================================");
  Serial.println("Pololu 37D Motor with Motoron Shield");
  Serial.println("========================================");
  Serial.print("Motor Channel: ");
  Serial.println(MOTOR_NUM);
  Serial.print("Encoder CPR (motor shaft): ");
  Serial.println(ENCODER_CPR);
  Serial.print("Gear Ratio: ");
  Serial.println(GEAR_RATIO, 1);
  Serial.print("Output CPR (output shaft): ");
  Serial.println(OUTPUT_CPR, 1);
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  s100  = Set speed to 100 (-800 to 800)");
  Serial.println("  p3200 = Move to position 3200");
  Serial.println("  r     = Reset position to 0");
  Serial.println("  x     = Stop motor");
  Serial.println("  b     = Brake motor");
  Serial.println("  i     = Show Motoron info");
  Serial.println("========================================");
  Serial.println();
  
  checkMotoronStatus();
}

void loop() {
  static long lastPosition = 0;
  static unsigned long lastPrint = 0;
  static unsigned long lastSpeedCalc = 0;
  static long lastSpeedPosition = 0;
  
  // Read current encoder position
  long currentPosition = motorEncoder.read();
  
  // Calculate speed (RPM) every 100ms
  float rpm = 0;
  if (millis() - lastSpeedCalc >= 100) {
    long positionChange = currentPosition - lastSpeedPosition;
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

// ============ MOTORON CONTROL FUNCTIONS ============

void setMotorSpeed(int16_t speed) {
  // Motoron speed range: -800 to +800
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

void brakeMotor() {
  // Set speed to 0 with braking (Motoron handles this automatically)
  mc.setSpeed(MOTOR_NUM, 0);
  motorSpeed = 0;
  positionMode = false;
  Serial.println("Motor braking...");
}

void positionControl(long currentPosition) {
  // Simple P controller for position
  long error = targetPosition - currentPosition;
  
  if (abs(error) < 10) {
    // Close enough - stop
    stopMotor();
    positionMode = false;
    Serial.println(">>> Target reached!");
  } else {
    // Calculate speed based on error
    // Motoron uses -800 to +800 range
    int16_t speed = constrain(error / 2, -600, 600);  // P gain
    if (abs(speed) < 100) speed = (speed > 0) ? 100 : -100;  // Minimum speed
    setMotorSpeed(speed);
  }
}

// ============ ANGLE CALCULATION FUNCTIONS ============

float calculateAngleDegrees(long position) {
  long positionMod = position % (long)OUTPUT_CPR;
  float angle = positionMod * (360.0 / OUTPUT_CPR);
  if (angle < 0) angle += 360.0;
  return angle;
}

float calculateAngleRadians(long position) {
  long positionMod = position % (long)OUTPUT_CPR;
  float angle = positionMod * (TWO_PI / OUTPUT_CPR);
  if (angle < 0) angle += TWO_PI;
  return angle;
}

float calculateRevolutions(long position) {
  return (float)position / OUTPUT_CPR;
}

// ============ MOTORON STATUS FUNCTIONS ============

void checkMotoronStatus() {
  uint16_t status = mc.getStatusFlags();
  
  Serial.println("\n--- Motoron Status ---");
  
  if (status & (1 << MOTORON_STATUS_FLAG_PROTOCOL_ERROR)) {
    Serial.println("⚠️  Protocol Error");
  }
  if (status & (1 << MOTORON_STATUS_FLAG_CRC_ERROR)) {
    Serial.println("⚠️  CRC Error");
  }
  if (status & (1 << MOTORON_STATUS_FLAG_COMMAND_TIMEOUT_LATCHED)) {
    Serial.println("⚠️  Command Timeout");
  }
  if (status & (1 << MOTORON_STATUS_FLAG_MOTOR_FAULT_LATCHED)) {
    Serial.println("⚠️  Motor Fault");
  }
  if (status & (1 << MOTORON_STATUS_FLAG_NO_POWER)) {
    Serial.println("⚠️  No Power");
  }
  if (status & (1 << MOTORON_STATUS_FLAG_RESET)) {
    Serial.println("ℹ️  Reset Flag Set");
    mc.clearResetFlag();
  }
  
  if (status == 0) {
    Serial.println("✓ All systems OK");
  }
  
  Serial.println("----------------------\n");
}

void printMotoronInfo() {
  Serial.println("\n========== MOTORON INFO ==========");
  
  // Get firmware version
  uint16_t version = mc.getFirmwareVersion();
  Serial.print("Firmware Version: ");
  Serial.print(version >> 8);
  Serial.print(".");
  Serial.println(version & 0xFF);
  
  // Get VIN voltage (in millivolts)
  uint16_t vinVoltage = mc.getVinVoltageMv(MOTORON_VIN_TYPE_POWER);
  Serial.print("VIN Voltage: ");
  Serial.print(vinVoltage / 1000.0, 2);
  Serial.println(" V");
  
  // Get current speed
  int16_t currentSpeed = mc.getCurrentSpeed(MOTOR_NUM);
  Serial.print("Current Speed: ");
  Serial.println(currentSpeed);
  
  // Check status
  checkMotoronStatus();
  
  Serial.println("==================================\n");
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
  }
  
  Serial.println();
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
        // Speed mode: s100 or s-100 (range: -800 to +800)
        positionMode = false;
        setMotorSpeed((int16_t)value);
        Serial.print("Speed set to: ");
        Serial.println(value);
        break;
        
      case 'p':
      case 'P':
        // Position mode: p3200
        targetPosition = value;
        positionMode = true;
        Serial.print("Moving to position: ");
        Serial.println(value);
        break;
        
      case 'r':
      case 'R':
        // Reset encoder position
        motorEncoder.write(0);
        Serial.println("Position reset to 0");
        break;
        
      case 'x':
      case 'X':
        // Stop motor (coast)
        stopMotor();
        Serial.println("Motor stopped");
        break;
        
      case 'b':
      case 'B':
        // Brake motor
        brakeMotor();
        break;
        
      case 'i':
      case 'I':
        // Show Motoron info
        printMotoronInfo();
        break;
        
      case 'c':
      case 'C':
        // Clear errors
        mc.clearResetFlag();
        mc.clearMotorFaultUnconditional();
        Serial.println("Errors cleared");
        break;
        
      case '?':
      case 'h':
      case 'H':
        // Help
        Serial.println("\nCommands:");
        Serial.println("  s<speed>   - Set speed (-800 to 800)");
        Serial.println("  p<pos>     - Move to position");
        Serial.println("  r          - Reset position to 0");
        Serial.println("  x          - Stop motor (coast)");
        Serial.println("  b          - Brake motor");
        Serial.println("  i          - Show Motoron info");
        Serial.println("  c          - Clear errors");
        Serial.println("  ?          - Show this help\n");
        break;
        
      default:
        Serial.println("Unknown command. Type ? for help");
        break;
    }
  }
}
// ```

// ## Wiring Diagram for Motoron Shield
// ```
// ENCODER CONNECTIONS:
// ────────────────────────────────────
// Encoder Wire    →   Arduino Pin
// ────────────────────────────────────
// Red    (Vcc)    →   5V
// Black  (GND)    →   GND
// Yellow (Ch A)   →   Pin 2 (interrupt)
// White  (Ch B)   →   Pin 3 (interrupt)


// MOTORON SHIELD CONNECTIONS:
// ────────────────────────────────────
// Motor terminals →   M1 port (or M2, M3)
// Power supply    →   VIN and GND
// Arduino         →   I2C (SDA/SCL) or Serial


// POWER:
// ────────────────────────────────────
// Motor Power     →   VIN: 4.5V to 24V
// Logic Power     →   Via Arduino 5V