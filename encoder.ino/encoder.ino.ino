// AS22 Encoder - Basic Signal Test
// This will show you if ANY signals are coming from the encoder

const int encoderPinA = 2;  // Channel A
const int encoderPinB = 3;  // Channel B
const int encoderIndex = 4; // Index pulse

void setup() {
  Serial.begin(115200);
  
  // Configure pins as inputs
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(encoderIndex, INPUT);
  
  Serial.println("========================================");
  Serial.println("AS22 ENCODER SIGNAL TEST");
  Serial.println("========================================");
  Serial.println();
  
  // Step 1: Check power and initial state
  Serial.println("STEP 1: Checking Initial Pin States");
  Serial.println("------------------------------------");
  delay(500);
  
  int stateA = digitalRead(encoderPinA);
  int stateB = digitalRead(encoderPinB);
  int stateI = digitalRead(encoderIndex);
  
  Serial.print("Channel A (Pin 2): ");
  Serial.println(stateA ? "HIGH (1)" : "LOW (0)");
  
  Serial.print("Channel B (Pin 3): ");
  Serial.println(stateB ? "HIGH (1)" : "LOW (0)");
  
  Serial.print("Index (Pin 4): ");
  Serial.println(stateI ? "HIGH (1)" : "LOW (0)");
  
  Serial.println();
  Serial.println("If all pins show LOW (0), check:");
  Serial.println("  - Is encoder powered? (5V connected)");
  Serial.println("  - Is GND connected?");
  Serial.println();
  
  Serial.println("STEP 2: Rotate the encoder shaft NOW!");
  Serial.println("------------------------------------");
  Serial.println("Watching for signal changes...");
  Serial.println();
}

void loop() {
  static int lastStateA = -1;
  static int lastStateB = -1;
  static int lastStateI = -1;
  static unsigned long changeCountA = 0;
  static unsigned long changeCountB = 0;
  static unsigned long changeCountI = 0;
  static unsigned long lastSummary = 0;
  
  // Read current pin states
  int currentStateA = digitalRead(encoderPinA);
  int currentStateB = digitalRead(encoderPinB);
  int currentStateI = digitalRead(encoderIndex);
  
  // Detect changes on Channel A
  if (currentStateA != lastStateA && lastStateA != -1) {
    changeCountA++;
    Serial.print("★ Channel A changed: ");
    Serial.print(lastStateA);
    Serial.print(" → ");
    Serial.print(currentStateA);
    Serial.print(" [Total: ");
    Serial.print(changeCountA);
    Serial.println("]");
  }
  lastStateA = currentStateA;
  
  // Detect changes on Channel B
  if (currentStateB != lastStateB && lastStateB != -1) {
    changeCountB++;
    Serial.print("★ Channel B changed: ");
    Serial.print(lastStateB);
    Serial.print(" → ");
    Serial.print(currentStateB);
    Serial.print(" [Total: ");
    Serial.print(changeCountB);
    Serial.println("]");
  }
  lastStateB = currentStateB;
  
  // Detect changes on Index
  if (currentStateI != lastStateI && lastStateI != -1) {
    changeCountI++;
    Serial.print("★★★ INDEX pulse detected! [Total: ");
    Serial.print(changeCountI);
    Serial.println("]");
  }
  lastStateI = currentStateI;
  
  // Print summary every 3 seconds
  if (millis() - lastSummary >= 3000) {
    Serial.println();
    Serial.println("========== SUMMARY ==========");
    Serial.print("A changes: ");
    Serial.print(changeCountA);
    Serial.print(" | B changes: ");
    Serial.print(changeCountB);
    Serial.print(" | Index: ");
    Serial.println(changeCountI);
    
    if (changeCountA == 0 && changeCountB == 0) {
      Serial.println("⚠️  NO SIGNALS DETECTED!");
      Serial.println("Check: Wiring, Power (5V), Shaft rotation");
    } else {
      Serial.println("✓ Encoder is working!");
    }
    Serial.println("=============================");
    Serial.println();
    
    lastSummary = millis();
  }
  
  delay(1);  // Small delay for stability
}