#include <Motoron.h>

MotoronI2C mc1(16); // The board configured to address 16
MotoronI2C mc2(17); // The board configured to address 17
// motor drivers set to front back 
// 16 is front , 17 is back 

const int MAX_SPEED = 800;   // Motoron max speed is 800
const int TEST_SPEED = 400;  // Run at 50% speed for safety

void setup() {
  Serial.begin(9600);
  Wire.begin(); // Join the I2C bus
  
  // Give the drivers a moment to power up
  delay(100);


  // --- SETUP DRIVER 1 (Address 16) ---
  mc1.reinitialize();       // Resets internal state variables in the library
  mc1.disableCrc();         // Disable CRC for simple communication
  mc1.clearResetFlag();     // Clear the "I just turned on" flag so it runs
  

  Serial.println("Driver 1 (Addr 16) Initialized.");

  // --- SETUP DRIVER 2 (Address 17) ---
  mc2.reinitialize(); 
  mc2.disableCrc();
  mc2.clearResetFlag();
  Serial.println("Driver 2 (Addr 17) Initialized.");
  
  Serial.println("Starting Test Loop...");
}

void loop() {


  //reverse 
  Serial.println("All wheels forward");
  mc1.setSpeed(2,800);
  mc2.setSpeed(2,800);
  mc1.setSpeed(3,800);
  mc2.setSpeed(3,800);
  delay(2000);

  //reverse 
  Serial.println("All wheels reverse");
  mc1.setSpeed(2,-800);
  mc2.setSpeed(2,-800);
  mc1.setSpeed(3,-800);
  mc2.setSpeed(3,-800);
  delay(2000);

  //stop motors 
  Serial.println("Stopping wheels");
  mc1.setSpeed(2,0);
  mc2.setSpeed(2,0);
  mc1.setSpeed(3,0);
  mc2.setSpeed(3,0);
  delay(2000);

}