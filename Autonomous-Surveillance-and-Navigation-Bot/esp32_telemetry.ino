#include <ArduinoJson.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- OLED DISPLAY CONFIGURATION ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1  
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- DHT11 CONFIGURATION ---
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// --- MQ-2 GAS CONFIGURATION ---
#define GAS_ANALOG_PIN 33  

// --- ULTRASONIC SONAR CONFIGURATION ---
const int trigLeft  = 5;   const int echoLeft  = 18;  
const int trigRight = 26;  const int echoRight = 27;
const int trigBack  = 15;  const int echoBack  = 23;

// --- IR PROXIMITY CONFIGURATION ---
const int irLeft  = 32;   
const int irRight = 34;   
const int irBack  = 35;   

// --- SYSTEM RUNTIME VARIABLES ---
unsigned long lastTelemetryTime = 0;
const unsigned long telemetryInterval = 100; 

String systemMode = "Manual";
String navStatus = "SYSTEMS ONLINE";
float currentLat = 23.75610; 
float currentLon = 90.42440;
float compassHeading = 59.0;
int batteryPercentage = 100;

void setup() {
  // Primary Serial over USB link connecting to Raspberry Pi
  Serial.begin(115200);
  
  // Hardware Serial2 initialization to command the Arduino Uno
  // Pin 16 acts as RX2 (from Uno TX), Pin 17 acts as TX2 (to Uno RX)
  Serial2.begin(9600, SERIAL_8N1, 16, 17);

  // Explicitly initialize I2C on pins 21 and 22
  Wire.begin(21, 22); 
  Wire.setClock(400000); // Set to fast I2C clock speed for smoother screen rendering

  // Initialize display across alternate standard addresses with dynamic fallback
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("[WARN]: Failed on 0x3C. Trying alternate 0x3D..."));
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
      Serial.println(F("[ERROR]: SSD1306 screen allocation failed completely."));
    }
  }

  // Initial Boot Screen Flush
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("=====================");
  display.println("  MISSION FAIL BOT   ");
  display.println("  TELEMETRY ONLINE   ");
  display.println("=====================");
  display.display();
  delay(1000);

  dht.begin();
  pinMode(GAS_ANALOG_PIN, INPUT);

  // Initialize Ultrasonic Pins
  pinMode(trigLeft, OUTPUT);  pinMode(echoLeft, INPUT);
  pinMode(trigRight, OUTPUT); pinMode(echoRight, INPUT);
  pinMode(trigBack, OUTPUT);  pinMode(echoBack, INPUT);

  // Initialize IR Pins
  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);
  pinMode(irBack, INPUT);
}

void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastTelemetryTime >= telemetryInterval) {
    lastTelemetryTime = currentTime;
    
    // Read Environmental Matrix Values
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    int gasVal = analogRead(GAS_ANALOG_PIN); 
    
    if (isnan(h) || isnan(t)) { h = 0.0; t = 0.0; }

    // Read Sonic Proximity Data
    float distanceLeft  = readUltrasonic(trigLeft, echoLeft);
    float distanceRight = readUltrasonic(trigRight, echoRight);
    float distanceBack  = readUltrasonic(trigBack, echoBack);

    // Read IR Pin Vectors
    int stateIRLeft  = digitalRead(irLeft);
    int stateIRRight = digitalRead(irRight);
    int stateIRBack  = digitalRead(irBack);

    // Alert Status Determination Logic
    if (stateIRLeft == 0 || stateIRRight == 0 || stateIRBack == 0) {
      navStatus = "PROXIMITY CRITICAL: IR collision vector active!";
    } else if (distanceLeft < 30.0 || distanceRight < 30.0 || distanceBack < 30.0) {
      navStatus = "PROXIMITY CRITICAL: Ultrasonic obstacle stop triggered!";
    } else if (gasVal > 900) { 
      navStatus = "ALERT: HAZARDOUS GAS THRESHOLD DETECTED";
    } else {
      navStatus = "PATH CLEAR / NOMINAL";
    }

    // Refresh Local Physical OLED Display Readouts
    display.clearDisplay();
    display.setCursor(0,0);
    display.printf("MODE: %s\n", systemMode.c_str());
    display.printf("GAS VALUE: %d\n", gasVal);
    display.println("SONAR RANGES (cm):");
    display.printf("L:%.1f R:%.1f B:%.1f\n", distanceLeft, distanceRight, distanceBack);
    display.printf("IR NODES: %d | %d | %d\n", stateIRLeft, stateIRRight, stateIRBack);
    display.printf("TEMP: %.1fC HUM:%.1f%%\n", t, h);
    display.display();

    // Pack telemetry payload frame configurations to send over USB to Raspberry Pi
    StaticJsonDocument<512> doc;
    doc["mode"] = systemMode;
    doc["status"] = navStatus;
    doc["lat"] = currentLat;
    doc["lon"] = currentLon;
    doc["heading"] = compassHeading;
    doc["battery"] = batteryPercentage;
    doc["temperature"] = t;
    doc["humidity"] = h;
    doc["gas_level"] = gasVal;
    doc["sonar_left"] = distanceLeft;
    doc["sonar_right"] = distanceRight;
    doc["sonar_back"] = distanceBack;
    doc["ir_left"] = stateIRLeft;
    doc["ir_right"] = stateIRRight;
    doc["ir_back"] = stateIRBack;

    serializeJson(doc, Serial);
    Serial.println(); 
  }
  
  // Check incoming controller commands from Python application running on the Pi
  if (Serial.available() > 0) {
    String inboundData = Serial.readStringUntil('\n');
    inboundData.trim();
    
    // Parse commands and push single character bytes out to the Uno via Serial2
    if (inboundData == "FORWARD") { 
      systemMode = "Manual"; 
      Serial2.print('F'); 
    }
    else if (inboundData == "BACKWARD") { 
      systemMode = "Manual"; 
      Serial2.print('B'); 
    }
    else if (inboundData == "LEFT") { 
      systemMode = "Manual"; 
      Serial2.print('L'); 
    }
    else if (inboundData == "RIGHT") { 
      systemMode = "Manual"; 
      Serial2.print('R'); 
    }
    else if (inboundData == "STOP") { 
      systemMode = "Manual"; 
      Serial2.print('S'); 
    }
  }
}

float readUltrasonic(int trigPin, int echoPin) {
  // Ensure trigger pin is low and clear beforehand
  digitalWrite(trigPin, LOW);
  delayMicroseconds(4);
  
  // Issue clean square wave pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Capture echo pulse with a relaxed timeout window (45000us) to handle mixed logic voltage shifts
  long duration = pulseIn(echoPin, HIGH, 45000); 
  
  // Standard out-of-range limit catch
  if (duration == 0 || duration > 40000) return 400.0; 
  
  // Calculate distance in centimeters
  return (duration * 0.0343) / 2.0;
}