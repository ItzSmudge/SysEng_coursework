// AS22 Encoder - Angle Tracking
// Converts encoder pulses to precise angle measurements

const int encoderPinA = 10;  // Channel A
const int encoderPinB = 11;  // Channel B
const int encoderIndex = 4; // Index pulse

// AS22 Encoder specs: 500 PPR (Pulses Per Revolution)
// In quadrature mode: 500 PPR × 4 = 2000 counts per revolution
const int PULSES_PER_REV = 4096;
const int COUNTS_PER_REV = PULSES_PER_REV * 1;  // 2000 counts

volatile long encoderCount = 0;
volatile bool indexFound = false;
volatile int lastStateA = 0;
volatile int lastStateB = 0;

void setup() {
  Serial.begin(115200);
  
  // Configure pins as inputs
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(encoderIndex, INPUT);
  
  // Read initial states
  lastStateA = digitalRead(encoderPinA);
  lastStateB = digitalRead(encoderPinB);
  
  // Attach interrupts for quadrature decoding
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderIndex), handleIndex, RISING);
  
  Serial.println("========================================");
  Serial.println("AS22 ENCODER - ANGLE TRACKING");
  Serial.println("========================================");
  Serial.println("Resolution: 500 PPR (2000 counts/rev)");
  Serial.println("Angle precision: 0.18°");
  Serial.println();
  Serial.println("Rotate shaft to find INDEX position...");
  Serial.println("========================================");
  Serial.println();
}

void loop() {
  static unsigned long lastPrint = 0;
  
  // Print angle data every 100ms
  if (millis() - lastPrint >= 100) {
    // Calculate angle from encoder counts
    float angle = (encoderCount % COUNTS_PER_REV) * 360.0 / COUNTS_PER_REV;
    
    // Ensure angle is always positive (0-360°)
    // if (angle < 0) {
    //   angle += 360.0;
    // }
    
    // Calculate total rotations
    long totalRotations = encoderCount / COUNTS_PER_REV;
    
    // // Print formatted output
    // Serial.print("Angle: ");
    // Serial.print(angle, 2);  // 2 decimal places
    // Serial.print("° | ");
    
    // Serial.print("Count: ");
    // Serial.print(encoderCount);
    // Serial.print(" | ");
    
    // Serial.print("Rotations: ");
    // Serial.print(totalRotations);
    
    // if (indexFound) {
    //   Serial.print(" | ✓ INDEXED");
    // } else {
    //   Serial.print(" | ⚠ Not indexed");
    // }
    
    // Serial.println();

    Serial.println(angle);
    
    lastPrint = millis();
  }
}

// Interrupt Service Routine for quadrature decoding
void updateEncoder() {
  int currentStateA = digitalRead(encoderPinA);
  int currentStateB = digitalRead(encoderPinB);
  
  // Quadrature decoding logic
  // Determines direction based on state transitions
  if (lastStateA == LOW && currentStateA == HIGH) {
    if (currentStateB == LOW) {
      encoderCount++;  // Clockwise
    } else {
      encoderCount--;  // Counter-clockwise
    }
  } else if (lastStateA == HIGH && currentStateA == LOW) {
    if (currentStateB == HIGH) {
      encoderCount++;  // Clockwise
    } else {
      encoderCount--;  // Counter-clockwise
    }
  } else if (lastStateB == LOW && currentStateB == HIGH) {
    if (currentStateA == HIGH) {
      encoderCount++;  // Clockwise
    } else {
      encoderCount--;  // Counter-clockwise
    }
  } else if (lastStateB == HIGH && currentStateB == LOW) {
    if (currentStateA == LOW) {
      encoderCount++;  // Clockwise
    } else {
      encoderCount--;  // Counter-clockwise
    }
  }
  
  lastStateA = currentStateA;
  lastStateB = currentStateB;
}

// Interrupt Service Routine for index pulse
void handleIndex() {
  // Reset count to zero when index pulse is detected
  encoderCount = 0;
  indexFound = true;
}