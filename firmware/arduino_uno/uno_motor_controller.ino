#include <SoftwareSerial.h>

// --- SOFTWARE SERIAL ASSIGNMENTS ---
// Pin 11 acts as RX (receives from ESP32 TX2)
// Pin 12 acts as TX (transmits to ESP32 RX2 via voltage divider)
SoftwareSerial espSerial(11, 12); 

// --- MOTOR DRIVER PINS ---
const int IN1 = 2;  const int IN2 = 3;  const int ENA = 4;  // Driver 1
const int IN3 = 5;  const int IN4 = 6;  const int ENB = 7;  // Driver 2
const int IN5 = 8;  const int IN6 = 9;  const int ENC = 10; // Driver 3

void setup() {
  // Initialize native serial for debugging via Pi terminal if needed
  Serial.begin(9600); 
  
  // Initialize software serial bus to listen to the ESP32 master
  espSerial.begin(9600);
  
  for(int i = 2; i <= 10; i++) {
    pinMode(i, OUTPUT);
  }
  
  stopRobot();
}

void loop() {
  // Read from the software serial stream instead of standard Serial
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
  digitalWrite(ENA, HIGH); digitalWrite(ENB, HIGH); digitalWrite(ENC, HIGH);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  digitalWrite(IN5, HIGH); digitalWrite(IN6, LOW);
}

void moveBackward() {
  digitalWrite(ENA, HIGH); digitalWrite(ENB, HIGH); digitalWrite(ENC, HIGH);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  digitalWrite(IN5, LOW);  digitalWrite(IN6, HIGH);
}

void turnLeft() {
  digitalWrite(ENA, HIGH); digitalWrite(ENB, HIGH); digitalWrite(ENC, HIGH);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); 
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  
  digitalWrite(IN5, LOW);  digitalWrite(IN6, LOW);   
}

void turnRight() {
  digitalWrite(ENA, HIGH); digitalWrite(ENB, HIGH); digitalWrite(ENC, HIGH);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); 
  digitalWrite(IN5, HIGH); digitalWrite(IN6, HIGH);  
}

void stopRobot() {
  digitalWrite(ENA, LOW);  digitalWrite(ENB, LOW);  digitalWrite(ENC, LOW);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW);  digitalWrite(IN6, LOW);
}
