# Arduino Giga Inverted Pendulum - WiFi Control Setup Guide

## Overview

This package upgrades your Arduino R4 inverted pendulum controller to use the faster Arduino Giga with WiFi control capabilities. The system allows you to:

- **Start/Stop** the pendulum controller remotely via WiFi
- **Reset the encoder** over WiFi
- **Adjust PID gains** in real-time without recompiling
- **Monitor system status** with live data streaming
- **Quick-tune** gains with an interactive wizard

## Hardware Requirements

- **Arduino Giga R1 WiFi**
- **Motoron I2C Motor Driver** (x2) - Addresses 16 & 17
- **AS22 Encoder** (4096 counts per revolution)
- **Motor Setup**: Same as R4 (should work with minimal changes)
- **WiFi Network**: 2.4GHz WiFi (Arduino Giga WiFiS3 limitation)

## Arduino IDE Setup

### 1. Install Arduino Giga Board Support

1. Open Arduino IDE → Preferences
2. Add board manager URL: `https://downloads.arduino.cc/packages/staging/package_staging_index.json`
3. Go to Tools → Board Manager
4. Search for "Arduino Giga" and install
5. Select Board: **Arduino Giga R1 WiFi**

### 2. Install Required Libraries

In Arduino IDE, go to Sketch → Include Library → Manage Libraries:

1. **Motoron Motor Controller Library**
   - Search: `motoron`
   - Install by Pololu

2. **ArduinoJson**
   - Search: `arduinojson`
   - Install by Benoit Blanchon

3. **WiFiS3**
   - Usually comes with Giga board package
   - If not: Search `WiFiS3` in library manager

### 3. Upload the Sketch

1. Open `giga_pendulum_wifi.ino` in Arduino IDE
2. **IMPORTANT**: Edit these lines to match your WiFi:
   ```cpp
   const char* ssid = "YOUR_SSID";           // Change this
   const char* password = "YOUR_PASSWORD";   // Change this
   ```
3. Select correct board and port
4. Click Upload

### 4. Verify Upload

Open Serial Monitor (115200 baud) and you should see:
```
Arduino Giga Inverted Pendulum Controller
========================================

Connecting to WiFi: YOUR_SSID
....
WiFi connected!
IP Address: 192.168.X.X
Server listening on port: 8080

System ready. Waiting for commands...
```

**Note the IP Address** - you'll need it for the Python script!

## Python Control Script Setup

### 1. Install Python Requirements

```bash
# Ensure you have Python 3.7+
python3 --version

# No external dependencies needed!
# (Uses only standard library: socket, json, cmd)
```

### 2. Make Script Executable (Linux/Mac)

```bash
chmod +x pendulum_control.py
```

### 3. Run the Script

**Interactive Mode:**
```bash
python3 pendulum_control.py
```

**Direct Connection:**
```bash
python3 pendulum_control.py 192.168.1.100
```

Replace `192.168.1.100` with your Giga's IP address from the serial monitor.

## Usage Guide

### Interactive CLI Commands

Once connected, you can use these commands:

#### Connection Management
```bash
giga> connect 192.168.1.100        # Connect to Giga
giga> connect 192.168.1.100 8080   # Connect with custom port
giga> disconnect                   # Disconnect
```

#### System Control
```bash
giga> start              # Start the pendulum controller
giga> stop               # Stop the pendulum controller
giga> reset_encoder      # Reset encoder count to zero
```

#### Status & Monitoring
```bash
giga> status             # Get current system status
giga> monitor            # Real-time monitoring (updates every 1 second)
giga> monitor 0.5        # Real-time monitoring (updates every 0.5 seconds)
```

#### Gain Tuning
```bash
# Set all 6 gains at once
giga> set_gains 10000 3000 300 30 15 0.3
#               kp_t  kd_t  ki_t  kp_x kd_x ki_x

# Interactive tuning wizard
giga> quick_tune

# Load preset configurations
giga> preset balanced      # balanced (default)
giga> preset aggressive    # more responsive
giga> preset conservative  # more stable
```

#### Help
```bash
giga> help               # Show all commands
giga> help set_gains     # Show help for specific command
giga> exit               # Exit program
```

### Example Workflow

```bash
$ python3 pendulum_control.py

giga> connect 192.168.1.100
✓ Connected to Giga at 192.168.1.100:8080

giga> status
==================================================
SYSTEM STATUS
==================================================
Status: STOPPED (ENABLED)
Theta: 0.0234 rad (1.34°)
Theta rate: 0.0000 rad/s
Encoder count: 0
Integral theta: 0.0000
Integral x: 0.0000
==================================================

giga> preset balanced
✓ Loaded 'balanced' preset

giga> start
✓ System started

giga> monitor
Monitoring (Ctrl+C to stop)...
● θ=  1.34° | θ̇=   0.1234 rad/s | count=     23
● θ=  1.35° | θ̇=   0.1245 rad/s | count=     24
● θ=  1.36° | θ̇=   0.1256 rad/s | count=     25
(Ctrl+C to stop)

giga> quick_tune
==================================================
PID GAIN QUICK TUNE WIZARD
==================================================
Theta Kp [10000]: 12000
Theta Kd [3000]: 3500
Theta Ki [300]: 300
Position Kp [30]: 30
Position Kd [15]: 15
Position Ki [0.3]: 0.3

Applying gains...
✓ PID gains updated:
  Theta: Kp=12000, Kd=3500, Ki=300
  Position: Kp=30, Kd=15, Ki=0.3
==================================================

giga> stop
✓ System stopped

giga> exit
Goodbye!
```

## Command Format (Advanced)

The WiFi protocol uses JSON for responses. Commands sent to the Giga are simple strings:

### Available Commands

| Command | Response | Purpose |
|---------|----------|---------|
| `START` | JSON status | Start the controller |
| `STOP` | JSON status | Stop the controller |
| `RESET_ENCODER` | JSON status | Reset encoder to 0 |
| `STATUS` | JSON with full state | Get system status |
| `SET_GAINS kp_t kd_t ki_t kp_x kd_x ki_x` | JSON status | Update PID gains |

### Response Format

All responses are JSON with this structure:
```json
{
  "timestamp": 12345,
  "status": "ok",
  "message": "System started",
  "system_running": true,
  "system_enabled": true,
  "theta_rad": 0.0234,
  "theta_deg": 1.34,
  "encoder_count": 23,
  "theta_dot": 0.1234,
  "int_theta": 0.0005,
  "int_x": 0.0001
}
```

## Troubleshooting

### Can't Connect to Giga

1. **Check IP Address**: Open Serial Monitor (115200 baud) and verify the Giga's IP
2. **WiFi Network**: Ensure Giga is connected to the same WiFi network as your computer
3. **Firewall**: Check if your firewall is blocking port 8080
4. **Credentials**: Double-check WiFi SSID and password in the Arduino sketch

### Connection Timeout

1. The Giga may be resetting - wait a few seconds and try again
2. Try pinging the Giga: `ping 192.168.1.100`
3. Ensure 2.4GHz WiFi is available (Giga doesn't support 5GHz)

### Commands Not Working

1. Use `status` command to verify connection
2. Check Arduino Serial Monitor for debug output
3. Ensure you're connected with `connect` command before sending other commands

### Motors Not Responding

1. Verify Motoron drivers are properly initialized (check serial output)
2. Check I2C connections to motor drivers (addresses 16 & 17)
3. Ensure motor power is connected
4. Try `STOP` and `START` commands to reset motor state

### Gain Tuning Not Working

1. System must be `STOPPED` before changing gains
2. Use exact format: `set_gains 10000 3000 300 30 15 0.3`
3. Check that all 6 gain values are provided (space-separated)

## Serial Port Commands (Backward Compatibility)

The Arduino code still supports serial commands for debugging:

```
START                           # Start system
STOP                            # Stop system
RESET_ENCODER                   # Reset encoder
RESET                           # Reset PID controller
STATUS                          # Print status
SET_GAINS kp_t kd_t ki_t kp_x kd_x ki_x  # Update gains
```

Example via Arduino Serial Monitor:
```
START
>> START command received
System started
```

## Performance Notes

### Timing

- **Control Loop**: 100 Hz (10 ms cycle)
- **Status Print**: 10 Hz (100 ms)
- **WiFi Command Response**: < 100 ms typical

### WiFi Stability

- WiFi updates are non-blocking
- Loss of WiFi connection does not affect local control
- Arduino Giga handles both serial and WiFi simultaneously

### Encoder Performance

- **Interrupt-driven**: Uses digital pins 2, 3, 4
- **Sampling**: Real-time encoder update at every pulse
- **Resolution**: 4096 counts per revolution (AS22 encoder)

## Pin Configuration Reference

```
ENCODER PINS:
- Pin 2:  Encoder A (Interrupt)
- Pin 3:  Encoder B (Interrupt)
- Pin 4:  Encoder Index (Interrupt)

I2C PINS (for Motoron drivers):
- SDA: Pin 20
- SCL: Pin 21

Motoron I2C Addresses:
- Motor Driver 1: Address 16
- Motor Driver 2: Address 17
```

## Customization

### Change WiFi Port

Edit in `giga_pendulum_wifi.ino`:
```cpp
const int WIFI_PORT = 8080;  // Change this to desired port
```

### Change Control Loop Frequency

In the main loop:
```cpp
if (currentTime - lastControl >= 10) {  // 10 ms = 100 Hz
  // Change to different value as needed
}
```

### Adjust Default Gains

Modify the PID initialization:
```cpp
PIDController pid(
  10000.0,   // kp_theta - adjust here
  3000.0,    // kd_theta
  300.0,     // ki_theta
  30.0,      // kp_x
  15.0,      // kd_x
  0.30,      // ki_x
  // ... other parameters
);
```

### Add Custom Commands

In `processWiFiCommand()` function, add new branches:
```cpp
} else if (command == "YOUR_COMMAND") {
    // Add your logic here
    jsonResponse["status"] = "ok";
    jsonResponse["message"] = "Command executed";
}
```

## Safety Warnings

⚠️ **Important Safety Notes:**

1. **Strong Motors**: The Giga is faster and more powerful than the R4. Test carefully!
2. **Encoder Timeout**: Reset encoder regularly during development
3. **WiFi Lag**: Don't rely on WiFi for emergency stop - use serial port or hardware E-stop
4. **Current Limits**: Monitor motor current - the Motoron drivers have built-in protection
5. **Mechanical Stress**: The stronger control may stress mechanical components

## Next Steps

1. Upload the Arduino code
2. Verify WiFi connection in Serial Monitor
3. Run the Python control script
4. Start with `preset conservative` for initial tuning
5. Gradually increase gains while monitoring with `monitor` command
6. Use `quick_tune` for fine adjustments

## Support & Debugging

For debugging, enable verbose output:
1. Open Serial Monitor at 115200 baud
2. Watch for debug messages during WiFi communication
3. Use `status` command frequently to check system state

Good luck with your faster inverted pendulum system! 🚀
