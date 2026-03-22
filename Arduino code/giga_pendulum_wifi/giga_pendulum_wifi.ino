// PID Controller with Moving Average Filter for Inverted Pendulum
// Arduino Giga with WiFi Control
// Compatible with AS22 Encoder (4096 counts per revolution)
// Motoron I2C Motor Driver Integration
// ============================================

#include <Motoron.h>
#include <WiFi.h>
#include <ArduinoJson.h>

// ============================================
// WIFI CONFIGURATION
// ============================================
const char* ssid = "Conn";           // Change this
const char* password = "12345678";   // Change this
const int WIFI_PORT = 8080;

WiFiServer server(WIFI_PORT);
WiFiClient wifiClient;

// ============================================
// MOTORON I2C CONFIGURATION
// ============================================
MotoronI2C mc1(16); // Front motor driver
MotoronI2C mc2(17); // Back motor driver

const int MAX_SPEED = 800;   // Motoron max speed is 800
const int MIN_SPEED = -800;  // Motoron min speed
const int PWM_to_motor = 1;
const int MOTORS_FORWARD = -1;

// ============================================
// CONTROL STATE
// ============================================
volatile bool systemRunning = false;
volatile bool systemEnabled = false;

// ============================================
// ENCODER CONFIGURATION
// ============================================
const int encoderPinA = 2;
const int encoderPinB = 3;
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
// PID CONTROLLER CLASS
// ============================================
class PIDController {
private:
  float kp_theta, ki_theta, kd_theta;
  float kp_x, ki_x, kd_x;
  float dt;
  float integral_theta;
  float integral_x;
  float i_limit_theta;
  float i_limit_x;
  bool filter_enabled;
  MovingAverageFilter* filter_theta;
  MovingAverageFilter* filter_x;
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
    unsigned long currentTime = millis();
    float actual_dt = (currentTime - lastTime) / 1000.0;
    if (actual_dt <= 0.0) actual_dt = dt;
    lastTime = currentTime;

    float filtered_theta = theta;
    float filtered_x = x;
    if (filter_enabled) {
      filtered_theta = filter_theta->apply(theta);
      filtered_x = filter_x->apply(x);
    }

    theta_dot = (filtered_theta - prev_theta) / actual_dt;
    x_dot = (filtered_x - prev_x) / actual_dt;
    prev_theta = filtered_theta;
    prev_x = filtered_x;

    float error_x = target_x - filtered_x;
    integral_x += error_x * actual_dt;
    integral_x = constrain(integral_x, -i_limit_x, i_limit_x);

    float desired_theta = (kp_x * error_x + ki_x * integral_x - kd_x * x_dot);

    float error_theta = desired_theta - filtered_theta;
    integral_theta += error_theta * actual_dt;
    integral_theta = constrain(integral_theta, -i_limit_theta, i_limit_theta);

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

  float getIntegralTheta() { return integral_theta; }
  float getIntegralX() { return integral_x; }
  float getThetaDot() { return theta_dot; }
  float getXDot() { return x_dot; }
};

// ============================================
// GLOBAL OBJECTS
// ============================================
PIDController pid(
  10000.0,   // kp_theta
  3000.0,    // kd_theta
  300.0,     // ki_theta
  30.0,      // kp_x
  15.0,      // kd_x
  0.30,      // ki_x
  0.3,       // dt
  50.0,      // i_limit_theta
  10.0,      // i_limit_x
  25,        // window_size
  true       // filter_enabled
);

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  delay(2000); // Wait for serial to stabilize
  
  Serial.println("\n\n========================================");
  Serial.println("Arduino Giga Inverted Pendulum Controller");
  Serial.println("========================================\n");

  Wire.begin();

  // Configure encoder pins
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(encoderIndex, INPUT);

  lastStateA = digitalRead(encoderPinA);
  lastStateB = digitalRead(encoderPinB);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderIndex), handleIndex, RISING);

  delay(100);

  // Initialize Motoron drivers
  mc1.reinitialize();
  mc1.disableCrc();
  mc1.clearResetFlag();
  Serial.println("Driver 1 (Addr 16) Initialized.");

  mc2.reinitialize();
  mc2.disableCrc();
  mc2.clearResetFlag();
  Serial.println("Driver 2 (Addr 17) Initialized.");

  // Initialize WiFi
  initializeWiFi();

  pid.reset();
  
  Serial.println("\nSystem ready. Waiting for commands...\n");
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  static unsigned long lastControl = 0;
  static unsigned long lastPrint = 0;
  unsigned long currentTime = millis();
  int motorSpeed = 0;

  // Handle WiFi client connections
  handleWiFiClients();

  // Control loop at 100Hz (10ms)
  if (currentTime - lastControl >= 10) {
    if (systemRunning && systemEnabled) {
      float theta = getAngleRadians();
      float x = 0.0;
      float target_x = 0.0;

      float force = pid.getAction(x, theta, target_x);
      motorSpeed = forceToMotorSpeed(force);
      setMotorSpeed(motorSpeed);
    } else {
      setMotorSpeed(0);
    }

    lastControl = currentTime;
  }

  // Print debug info at 10Hz (100ms)
  if (currentTime - lastPrint >= 100) {
    float theta = getAngleRadians();
    Serial.print("Status: ");
    Serial.print(systemRunning ? "RUNNING" : "STOPPED");
    Serial.print(" | Theta: ");
    Serial.print(theta, 4);
    Serial.print(" rad (");
    Serial.print(theta * (180.0 / 3.14159), 1);
    Serial.print("°) | Motor: ");
    Serial.print(motorSpeed);
    Serial.println();
    lastPrint = currentTime;
  }

  // Handle serial commands (for backward compatibility)
  handleSerialCommands();
}

// ============================================
// WIFI FUNCTIONS
// ============================================
void initializeWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  // Attempt to connect to WiFi network
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    WiFi.begin(ssid, password);
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Server listening on port: ");
    Serial.println(WIFI_PORT);
    server.begin();
  } else {
    Serial.println("\nFailed to connect to WiFi!");
    Serial.println("Continuing without WiFi...");
  }
}

void handleWiFiClients() {
  // Check for new clients
  WiFiClient client = server.available();

  if (client) {
    Serial.println("New WiFi client connected");

    while (client.connected()) {
      if (client.available()) {
        String request = client.readStringUntil('\n');
        request.trim();

        String response = processWiFiCommand(request);

        // Send response back to client
        client.print(response);
        client.flush();

        // If not a persistent command, close connection
        if (!request.startsWith("STREAM")) {
          client.stop();
          break;
        }
      }
      yield(); // Allow other processes to run
    }

    Serial.println("WiFi client disconnected");
  }
}

String processWiFiCommand(String command) {
  StaticJsonDocument<512> jsonResponse;
  jsonResponse["timestamp"] = millis();

  if (command == "START") {
    systemRunning = true;
    systemEnabled = true;
    pid.reset();
    jsonResponse["status"] = "ok";
    jsonResponse["message"] = "System started";
    Serial.println(">> START command received");

  } else if (command == "STOP") {
    systemRunning = false;
    setMotorSpeed(0);
    jsonResponse["status"] = "ok";
    jsonResponse["message"] = "System stopped";
    Serial.println(">> STOP command received");

  } else if (command == "RESET_ENCODER") {
    encoderCount = 0;
    indexFound = true;
    jsonResponse["status"] = "ok";
    jsonResponse["message"] = "Encoder reset to 0";
    Serial.println(">> RESET_ENCODER command received");

  } else if (command.startsWith("SET_GAINS ")) {
    // Format: SET_GAINS kp_t kd_t ki_t kp_x kd_x ki_x
    float gains[6];
    String params = command.substring(10);
    int count = parseFloats(params, gains, 6);

    if (count == 6) {
      pid.setGains(gains[0], gains[1], gains[2], gains[3], gains[4], gains[5]);
      jsonResponse["status"] = "ok";
      jsonResponse["message"] = "Gains updated";
      jsonResponse["gains"][0] = gains[0];
      jsonResponse["gains"][1] = gains[1];
      jsonResponse["gains"][2] = gains[2];
      jsonResponse["gains"][3] = gains[3];
      jsonResponse["gains"][4] = gains[4];
      jsonResponse["gains"][5] = gains[5];
      Serial.println(">> SET_GAINS command received");
    } else {
      jsonResponse["status"] = "error";
      jsonResponse["message"] = "Invalid gains format";
    }

  } else if (command == "STATUS") {
    float theta = getAngleRadians();
    jsonResponse["status"] = "ok";
    jsonResponse["system_running"] = systemRunning;
    jsonResponse["system_enabled"] = systemEnabled;
    jsonResponse["theta_rad"] = theta;
    jsonResponse["theta_deg"] = theta * (180.0 / 3.14159);
    jsonResponse["encoder_count"] = encoderCount;
    jsonResponse["theta_dot"] = pid.getThetaDot();
    jsonResponse["int_theta"] = pid.getIntegralTheta();
    jsonResponse["int_x"] = pid.getIntegralX();

  } else {
    jsonResponse["status"] = "error";
    jsonResponse["message"] = "Unknown command";
  }

  String response;
  serializeJson(jsonResponse, response);
  response += "\n";
  return response;
}

int parseFloats(String input, float* output, int maxCount) {
  int count = 0;
  int startIdx = 0;

  for (int i = 0; i < input.length() && count < maxCount; i++) {
    if (input[i] == ' ' || i == input.length() - 1) {
      int endIdx = (i == input.length() - 1) ? i + 1 : i;
      String numStr = input.substring(startIdx, endIdx);
      numStr.trim();
      if (numStr.length() > 0) {
        output[count++] = numStr.toFloat();
      }
      startIdx = i + 1;
    }
  }

  return count;
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

    if (command == "START") {
      systemRunning = true;
      systemEnabled = true;
      pid.reset();
      Serial.println("System started");

    } else if (command == "STOP") {
      systemRunning = false;
      setMotorSpeed(0);
      Serial.println("System stopped");

    } else if (command == "RESET_ENCODER") {
      encoderCount = 0;
      indexFound = true;
      Serial.println("Encoder reset");

    } else if (command == "RESET") {
      pid.reset();
      Serial.println("PID reset");

    } else if (command == "STATUS") {
      float theta = getAngleRadians();
      Serial.print("Theta: ");
      Serial.print(theta, 4);
      Serial.print(" rad | Running: ");
      Serial.println(systemRunning ? "YES" : "NO");

    } else if (command.startsWith("SET_GAINS ")) {
      float gains[6];
      String params = command.substring(10);
      int count = parseFloats(params, gains, 6);

      if (count == 6) {
        pid.setGains(gains[0], gains[1], gains[2], gains[3], gains[4], gains[5]);
        Serial.println("Gains updated");
      }
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
