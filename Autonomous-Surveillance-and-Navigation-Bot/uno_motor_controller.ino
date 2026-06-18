#include <SoftwareSerial.h>

// --- SOFTWARE SERIAL ASSIGNMENTS ---
// Pin 11 acts as RX (receives from ESP32 TX2)
// Pin 12 acts as TX (transmits to ESP32 RX2 via voltage divider)
SoftwareSerial espSerial(11, 12); 

// --- NEW MOTOR DRIVER PIN DEFINITIONS ---
// Driver 1: Outer Left Track (Front-Left & Back-Left)
const int IN1 = 2;  const int IN2 = 3;  const int ENA = 4;  

// Driver 2: Outer Right Track (Front-Right & Back-Right)
const int IN3 = 5;  const int IN4 = 6;  const int ENB = 7;  

// Driver 3: Center Pivot Axis (Center-Left & Center-Right)
const int IN5 = 8;  // Labeled as IN1 on Driver 3 (Controls Center-Left)
const int IN6 = 9;  // Labeled as IN3 on Driver 3 (Controls Center-Right)
const int ENC = 10; // Labeled as ENA on Driver 3 (Shares ENA/ENB via jumper)

void setup() {
  Serial.begin(9600); 
  espSerial.begin(9600);
  
  // Configure pins 2 through 10 as outputs
  for(int i = 2; i <= 10; i++) {
    pinMode(i, OUTPUT);
  }
  
  stopRobot();
}

void loop() {
  if (espSerial.available() > 0) {
    char command = espSerial.read();
    Serial.print("Executing Command: ");
    Serial.println(command);
    
    switch(command) {
      case 'F': moveForward();  break;
      case 'B': moveBackward(); break;
      case 'L': turnLeft();     break;
      case 'R': turnRight();    break;
      case 'S': stopRobot();    break;
    }
  }
}

void moveForward() {
  // Turn on power to all three driver modules
  digitalWrite(ENA, HIGH); digitalWrite(ENB, HIGH); digitalWrite(ENC, HIGH);
  
  // Driver 1: Outer Left Track Forward
  digitalWrite(IN1, HIGH);  digitalWrite(IN2, LOW);
  
  // Driver 2: Outer Right Track Forward
  digitalWrite(IN3, HIGH);  digitalWrite(IN4, LOW);
  
  // Driver 3: Center Track Forward
  digitalWrite(IN5, HIGH);  // Center-Left Forward
  digitalWrite(IN6, HIGH);  // Center-Right Forward
}

void moveBackward() {
  digitalWrite(ENA, HIGH); digitalWrite(ENB, HIGH); digitalWrite(ENC, HIGH);
  
  // Driver 1: Outer Left Track Backward
  digitalWrite(IN1, LOW);   digitalWrite(IN2, HIGH);
  
  // Driver 2: Outer Right Track Backward
  digitalWrite(IN3, LOW);   digitalWrite(IN4, HIGH);
  
  // Driver 3: Center Track Backward
  digitalWrite(IN5, LOW);   // Center-Left Backward
  digitalWrite(IN6, LOW);   // Center-Right Backward
}

void turnLeft() {
  digitalWrite(ENA, HIGH); digitalWrite(ENB, HIGH); digitalWrite(ENC, HIGH);
  
  // Skid Steering: Left side spins backward, Right side spins forward
  digitalWrite(IN1, LOW);   digitalWrite(IN2, HIGH); // Outer Left Backward
  digitalWrite(IN3, HIGH);  digitalWrite(IN4, LOW);  // Outer Right Forward
  digitalWrite(IN5, LOW);                            // Center-Left Backward
  digitalWrite(IN6, HIGH);                           // Center-Right Forward
}

void turnRight() {
  digitalWrite(ENA, HIGH); digitalWrite(ENB, HIGH); digitalWrite(ENC, HIGH);
  
  // Skid Steering: Left side spins forward, Right side spins backward
  digitalWrite(IN1, HIGH);  digitalWrite(IN2, LOW);  // Outer Left Forward
  digitalWrite(IN3, LOW);   digitalWrite(IN4, HIGH); // Outer Right Backward
  digitalWrite(IN5, HIGH);                           // Center-Left Forward
  digitalWrite(IN6, LOW);                            // Center-Right Backward
}

void stopRobot() {
  // Cut the enable lines to cut mechanical power instantly across all drivers
  digitalWrite(ENA, LOW);  digitalWrite(ENB, LOW);  digitalWrite(ENC, LOW);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW);  digitalWrite(IN6, LOW);
}