#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
// Install Adafruit_MPU6050 and Adafruit_Sensor libraries in Arduino IDE
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ----------------- Pins for Arduino Uno -----------------
// Note: These are suitable digital pins for the Uno.

#define SOS_BUTTON     2      // Digital Pin 2
#define CANCEL_BUTTON  3      // Digital Pin 3
#define BUZZER         4      // Digital Pin 4
#define VIB_MOTOR1     5      // Digital Pin 5
#define VIB_MOTOR2     6      // Digital Pin 6
#define TRIG_PIN       7      // Digital Pin 7
#define ECHO_PIN       8      // Digital Pin 8
// The LDR pin was removed because it was not used in your loop.
// If you need it, assign it to one of A0-A5.
// #define LDR_PIN        A0   // Analog Pin A0

// MPU6050 is connected via I2C (A4=SDA, A5=SCL on Uno)
Adafruit_MPU6050 mpu;

// GPS module
// New SoftwareSerial pins
#define GPS_RX_PIN 10
#define GPS_TX_PIN 11
TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN); // RX (Uno) to GPS TX, TX (Uno) to GPS RX

// SIM800L
// New SoftwareSerial pins
#define SIM_RX_PIN 12
#define SIM_TX_PIN 13
SoftwareSerial simSerial(SIM_RX_PIN, SIM_TX_PIN); // RX (Uno) to SIM TX, TX (Uno) to SIM RX
String emergencyNumber = "phone no.";

// ----------------- Variables -----------------
bool countdownActive = false;
unsigned long countdownStart = 0;
const unsigned long COUNTDOWN_TIME = 180000; // 3 minutes

// ----------------- Setup -----------------
void setup() {
  Serial.begin(9600);

  // Pins
  pinMode(SOS_BUTTON, INPUT_PULLUP);
  pinMode(CANCEL_BUTTON, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);
  pinMode(VIB_MOTOR1, OUTPUT);
  pinMode(VIB_MOTOR2, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(BUZZER, LOW);
  digitalWrite(VIB_MOTOR1, LOW);
  digitalWrite(VIB_MOTOR2, LOW);

  // Initialize MPU6050
  // MPU6050 must be connected to I2C (SDA/SCL) on A4 and A5
  if (!mpu.begin()) {
    Serial.println("MPU6050 not detected! Check wiring: VCC-5V, GND-GND, SDA-A4, SCL-A5");
    // Instead of stopping here, we will let it continue
  } else {
    Serial.println("MPU6050 ready.");
  }

  // Initialize GPS (9600 baud rate)
  gpsSerial.begin(9600);
  Serial.println("GPS ready.");

  // Initialize SIM800L (9600 baud rate)
  simSerial.begin(9600);
  Serial.println("SIM800L ready.");
  delay(1000);

  Serial.println("System initialized.");
}

// ----------------- Main Loop -----------------
void loop() {
  // Read GPS data
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // Manual SOS
  if (digitalRead(SOS_BUTTON) == LOW) {
    Serial.println("Manual SOS triggered!");
    alertBuzzAndVibrate(1000);
    sendSOS();
    delay(800); // debounce
  }

  // Ultrasonic detection
  float dist = getUltrasonicDistance();
  // When ultrasonic distance is between 0 and 50 cm
  if (dist > 0 && dist < 50) {
    Serial.print("Obstacle detected: "); Serial.println(dist);
    alertBuzzAndVibrate(200);
    delay(600); // to avoid re-trigger
  }

  // MPU6050 tilt detection (example)
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Simple tilt detection on X/Y axis
  if ((abs(a.acceleration.x) > 9.0 || abs(a.acceleration.y) > 9.0) && !countdownActive) {
    Serial.println("Tilt detected, starting countdown.");
    countdownActive = true;
    countdownStart = millis();
    alertBuzzAndVibrate(50);
  }

  // Countdown logic
  if (countdownActive) {
    // If CANCEL button is pressed
    if (digitalRead(CANCEL_BUTTON) == LOW) {
      countdownActive = false;
      Serial.println("Countdown cancelled by user.");
      delay(500);
    } 
    // If 3 minutes (COUNTDOWN_TIME) are complete
    else if (millis() - countdownStart >= COUNTDOWN_TIME) {
      countdownActive = false;
      Serial.println("Countdown finished - sending SOS!");
      alertBuzzAndVibrate(1500);
      sendSOS();
    }
  }

  delay(100);
}

// ----------------- Helper Functions -----------------

// Buzzer + motors alert
void alertBuzzAndVibrate(unsigned long duration_ms) {
  digitalWrite(BUZZER, HIGH);
  digitalWrite(VIB_MOTOR1, HIGH);
  digitalWrite(VIB_MOTOR2, HIGH);
  delay(duration_ms);
  digitalWrite(BUZZER, LOW);
  digitalWrite(VIB_MOTOR1, LOW);
  digitalWrite(VIB_MOTOR2, LOW);
}

// Ultrasonic distance function
float getUltrasonicDistance() {
  // Send the Trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Measure the duration of the Echo pulse
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
  
  if (duration == 0) return 9999;
  
  // Calculate distance (in centimeters)
  // Distance = (Duration / 2) / 29.1
  float cm = (duration / 2.0) / 29.1;
  return cm;
}

// Send SOS via SIM800L
void sendSOS() {
  String msg = "SOS! I need help!";
  
  // If GPS location is valid, append it to the message
  if (gps.location.isValid()) {
    msg += " Location: ";
    msg += String(gps.location.lat(), 6) + ",";
    msg += String(gps.location.lng(), 6);
  }
  
  // SIM800L Commands
  simSerial.println("AT+CMGF=1"); // Set SMS mode
  delay(500);
  simSerial.println("AT+CMGS=\"" + emergencyNumber + "\"");
  delay(500);
  simSerial.print(msg);
  delay(500);
  simSerial.write(26); // Ctrl+Z (ASCII 26) to send SMS
  
  Serial.println("SOS SMS sent: " + msg);
}
