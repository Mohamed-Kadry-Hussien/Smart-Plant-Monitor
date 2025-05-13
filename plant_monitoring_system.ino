#define BLYNK_TEMPLATE_ID "TMPL2xHjzNLEn"
#define BLYNK_TEMPLATE_NAME "SMART PLANT MONITOR"
#define BLYNK_AUTH_TOKEN "ZM13BPFqg2sbRK9GpyNWTJBAMA8kBWmB"

#include <WiFi.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <BlynkSimpleEsp32.h>

// ===== Configuration =====
#define BLYNK_PRINT Serial
char auth[] = "ZM13BPFqg2sbRK9GpyNWTJBAMA8kBWmB";
char ssid[] = "SuperNova";
char pass[] = "22222222";

// ===== Pin Definitions =====
#define DHTPIN          14
#define SOIL_MOISTURE_PIN 34
#define TRIG_PIN        12
#define ECHO_PIN        13
#define RELAY_PIN       27  
#define BUZZER_PIN      26
#define RED_LED         25
#define YELLOW_LED      33
#define GREEN_LED       32
#define PHYSICAL_BUTTON_PIN 4 

// I2C Configuration
#define I2C_SDA         21
#define I2C_SCL         22

// ===== System Settings =====
#define SOIL_DRY_THRESHOLD      40
#define SOIL_WET_THRESHOLD      70
#define TANK_LOW_THRESHOLD      3.5
#define TANK_MEDIUM_THRESHOLD   7
#define MIN_PUMP_LEVEL         2
#define TANK_HEIGHT            14
#define HIGH_TEMP_THRESHOLD    30.0
#define LOW_HUMIDITY_THRESHOLD 30.0

// Timing Intervals
#define SENSOR_READ_INTERVAL   1000
#define WATER_EMERG_INTERVAL   200
#define PUMP_MIN_TIME         2000
#define ULTRASONIC_TIMEOUT    5000
#define DEBOUNCE_DELAY        50
#define DATA_SEND_INTERVAL    2000

// ===== Blynk Virtual Pins =====
#define VPIN_WATER_LEVEL    V0
#define VPIN_TEMPERATURE    V1
#define VPIN_HUMIDITY       V2
#define VPIN_SOIL_MOISTURE  V3
#define VPIN_PUMP_CONTROL   V4
#define VPIN_RED_LED        V5
#define VPIN_YELLOW_LED     V6
#define VPIN_GREEN_LED      V7

// ===== Global ذVariables =====
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);
unsigned long lastSensorRead = 0;
unsigned long lastWaterCheck = 0;
unsigned long lastDataSend = 0;
bool pumpRunning = false;
unsigned long pumpStartTime = 0;
bool wifiConnected = false;
BlynkTimer timer;

// Button variables
bool lastButtonState = HIGH;
bool currentButtonState = HIGH;
unsigned long lastDebounceTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  // Initialize sensors
  dht.begin();
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Initialize outputs
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); 
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(PHYSICAL_BUTTON_PIN, INPUT_PULLUP);

  // LCD setup
  lcd.init();
  lcd.backlight();
  lcd.print("Initializing...");

  // Connect to WiFi and Blynk
  connectToWiFi();
  
  // Setup timers
  timer.setInterval(DATA_SEND_INTERVAL, sendSensorData);
}

void loop() {
  Blynk.run();
  timer.run();
  
  handlePhysicalButton(); // Handle physical button press
  
  unsigned long currentMillis = millis();

  // Fast emergency water check
  if (currentMillis - lastWaterCheck >= WATER_EMERG_INTERVAL) {
    lastWaterCheck = currentMillis;
    checkWaterEmergency();
  }

  // Main sensor reading and control
  if (currentMillis - lastSensorRead >= SENSOR_READ_INTERVAL) {
    lastSensorRead = currentMillis;
    readSensorsAndControl();
  }

  // Pump timeout control
  if (pumpRunning && (currentMillis - pumpStartTime > PUMP_MIN_TIME)) {
    stopWatering();
  }
}

// ===== Physical Button Handling =====
void handlePhysicalButton() {
  int reading = digitalRead(PHYSICAL_BUTTON_PIN);
  
  // Debounce logic
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != currentButtonState) {
      currentButtonState = reading;
      
      // Button pressed (LOW because of pull-up)
      if (currentButtonState == LOW) {
        togglePump();
      }
    }
  }
  
  lastButtonState = reading;
}

void togglePump() {
  if (pumpRunning) {
    stopWatering();
    Blynk.virtualWrite(VPIN_PUMP_CONTROL, 0);
  } 
  else if (readWaterLevelAccurate() > MIN_PUMP_LEVEL) {
    startWatering();
    Blynk.virtualWrite(VPIN_PUMP_CONTROL, 1); 
  }
  else {
    tone(BUZZER_PIN, 1000, 200);
  }
}

// ===== Blynk Functions =====
BLYNK_WRITE(VPIN_PUMP_CONTROL) {
  if (param.asInt() == 1) {
    if (!pumpRunning && readWaterLevelAccurate() > MIN_PUMP_LEVEL) {
      startWatering();
    }
  } else {
    if (pumpRunning) {
      stopWatering();
    }
  }
}

void sendSensorData() {
  if (!wifiConnected) return;
  
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  int soilMoisture = readSoilMoisture();
  float waterLevel = readWaterLevelAccurate();

  // Send data to Blynk
  Blynk.virtualWrite(VPIN_TEMPERATURE, temperature);
  Blynk.virtualWrite(VPIN_HUMIDITY, humidity);
  Blynk.virtualWrite(VPIN_SOIL_MOISTURE, soilMoisture);
  Blynk.virtualWrite(VPIN_WATER_LEVEL, waterLevel);
  
  // Sync LED states
  Blynk.virtualWrite(VPIN_RED_LED, digitalRead(RED_LED));
  Blynk.virtualWrite(VPIN_YELLOW_LED, digitalRead(YELLOW_LED));
  Blynk.virtualWrite(VPIN_GREEN_LED, digitalRead(GREEN_LED));
}

// ===== Pump Control Functions =====
void startWatering() {
  digitalWrite(RELAY_PIN, HIGH);
  delay(1000);
  pumpRunning = true;
  pumpStartTime = millis();
  lcd.setCursor(15, 1);
  lcd.print("P");
}

void stopWatering() {
  digitalWrite(RELAY_PIN, LOW); 
  pumpRunning = false;
  lcd.setCursor(15, 1);
  lcd.print(" ");
}

// ===== Sensor Reading Functions =====
int readSoilMoisture() {
  int soil_read = analogRead(SOIL_MOISTURE_PIN);
  delay(20); 
  int moisture = map(soil_read, 4095, 1200, 0, 100);
  return constrain(moisture, 0, 100);
}

float readWaterLevelFast() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT);
  return (duration == 0) ? 0 : constrain(TANK_HEIGHT - (duration * 0.034 / 2), 0, TANK_HEIGHT);
}

float readWaterLevelAccurate() {
  float sum = 0;
  for (int i = 0; i < 5; i++) {
    sum += readWaterLevelFast();
    delay(50);
  }
  return sum / 5;
}

// ===== Control Logic Functions =====
void checkWaterEmergency() {
  float waterLevel = readWaterLevelFast();

  if (waterLevel <= TANK_LOW_THRESHOLD) {
    digitalWrite(RED_LED, HIGH);
    tone(BUZZER_PIN, 1000, 100);
    if (wifiConnected) {
      Blynk.logEvent("water_alert", "⚠️ Low Water Level!");
    }
  } else {
    digitalWrite(RED_LED, LOW);
    noTone(BUZZER_PIN);
  }
}

void readSensorsAndControl() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  int soilMoisture = readSoilMoisture();
  float waterLevel = readWaterLevelAccurate();

  // Automatic watering logic
  if (shouldWaterPlant(soilMoisture, waterLevel, temperature, humidity)) {
    startWatering();
  }

  // Update indicators
  updateWaterIndicators(waterLevel);
  updateDisplay(temperature, humidity, soilMoisture, waterLevel);
}

bool shouldWaterPlant(int soil, float water, float temp, float hum) {
  if (water <= MIN_PUMP_LEVEL || pumpRunning) return false;

  return (soil < SOIL_DRY_THRESHOLD) ||
         (temp > HIGH_TEMP_THRESHOLD && soil < SOIL_WET_THRESHOLD) ||
         (hum < LOW_HUMIDITY_THRESHOLD && soil < (SOIL_DRY_THRESHOLD + 10));
}

void updateWaterIndicators(float level) {
  bool redState = level <= TANK_LOW_THRESHOLD;
  bool yellowState = level > TANK_LOW_THRESHOLD && level <= TANK_MEDIUM_THRESHOLD;
  bool greenState = level > TANK_MEDIUM_THRESHOLD;
  
  digitalWrite(RED_LED, redState);
  digitalWrite(YELLOW_LED, yellowState);
  digitalWrite(GREEN_LED, greenState);
}

void updateDisplay(float temp, float hum, int soil, float water) {
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temp, 1);
  lcd.print("C H:");
  lcd.print(hum, 0);
  lcd.print("%  ");

  lcd.setCursor(0, 1);
  lcd.print("S:");
  lcd.print(soil);
  lcd.print("% W:");
  lcd.print(water, 1);
  lcd.print("cm");
}

// ===== WiFi Functions =====
void connectToWiFi() {
  WiFi.begin(ssid, pass);
  Blynk.config(auth);

  lcd.setCursor(0, 1);
  lcd.print("Connecting WiFi...");

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 15) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  wifiConnected = (WiFi.status() == WL_CONNECTED);

  lcd.clear();
  if (wifiConnected) {
    lcd.print("WiFi Connected");
    Serial.println("\nWiFi Connected!");
    Blynk.connect();
  } else {
    lcd.print("WiFi Failed!");
    Serial.println("\nConnection Failed!");
  }
  delay(1000);
}