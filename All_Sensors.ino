#include <LiquidCrystal_I2C.h> 
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

// ULTRA_SONIC SENSOR
#define TRIGGER_PIN 3
#define ECHO_PIN 4
#define MIN_DISTANCE 0
#define MAX_DISTANCE 40
double Tank_Capacity = 10.18;
double Volume;
double Available_Capacity;

// DHT11 SENSOR
#include "DHT.h"
#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);


// Pressure & FlowRate
int const Pressure_Sensor_pin = A1;
int const Flow_Sensor_pin = A2;
int pot1Val;
int pot2Val;
int Pressure;
int FlowRate;

// MQ2 GAS Sensoe
const int mq2Pin = A0; // Analog input pin for the MQ-2 sensor
const float R0 = 10000; // The resistance of the sensor in fresh air
const float RLPG = 10000; // The resistance in clean air for LPG
const float RHydrogen = 10000; // The resistance in clean air for Hydrogen
const float RMethane = 10000; // The resistance in clean air for Methane

unsigned long previousMillis1 = 1000;  // Timer for Sensor Reading
const unsigned long interval1 = 2000; // Reading interval in milliseconds

unsigned long previousMillis2 = 0;  // Timer for JSON Transmission
const unsigned long interval2 = 500; // Transmission interval in milliseconds

SoftwareSerial nodeMCU(4, 5);  // Initialise Arduino to NodeMCU (4 Rx & 5 Tx)
LiquidCrystal_I2C lcd(0x27, 20, 4); // I2C address 0x27, 20 column and 4 rows

void setup()
{
  
  Serial.begin(9600);
  nodeMCU.begin(9600); 
  dht.begin();
  lcd.backlight();
  lcd.init();
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

}

void loop() {

  // Get current time
  unsigned long currentMillis = millis();

  // Task 1: Read sensor values every 2 seconds
  if (currentMillis - previousMillis1 >= interval1) {
    previousMillis1 = currentMillis;
  
  // Read Sensor values
  // Ultra-Sonic Sensor
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  int distance = duration * 0.034 / 2;
  Volume= (distance * 254.47 )* 0.001;
  Available_Capacity = (Tank_Capacity-Volume);

  // MQ2 GAS SENSOR
  int sensorValue = analogRead(mq2Pin);
  float sensorVoltage = (sensorValue / 1023.0) * 5.0; // Convert to voltage
  float Rr = ((5.0 - sensorVoltage) / sensorVoltage) * R0; // Calculate the resistance of the sensor
  float concentrationLPG = Rr / RLPG;// Calculate gas concentrations for LPG, Hydrogen, and Methane
  float concentrationHydrogen = Rr / RHydrogen;
  float concentrationMethane = Rr / RMethane;

  // FLOWRATE AND PPRESSURE SENSOR 
  pot1Val =analogRead(Pressure_Sensor_pin);
  pot2Val =analogRead(Flow_Sensor_pin); 
  Pressure = map(pot1Val, 0, 1023, 0, 40); // Scale potentiometer value to the Pressure sensor Maximum & Minimum value
  FlowRate = map(pot2Val, 0, 1023, 1, 30);

 

lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Pressure: ");
  lcd.print(Pressure);
  lcd.print(" KPa");
  
  lcd.setCursor(0,1);
  lcd.print("FlowRate: ");
  lcd.print(FlowRate);
  lcd.print(" L/min");

  lcd.setCursor(0,2);
   lcd.print("Humidity: ");
   lcd.print((float)dht.readHumidity());
   lcd.print(" %");

   lcd.setCursor(0,3);
   lcd.print("Temperature: ");
   lcd.print((float)dht.readTemperature());
   lcd.print("C");
   delay (3000);
   
lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(" GAS CONCENTRATION: ");
  lcd.setCursor(0,1);
  lcd.print("LPG      : ");
  lcd.print(concentrationLPG);
  lcd.print(" ppm");

  lcd.setCursor(0,2);
  lcd.print("Hydrogen : ");
  lcd.print(concentrationHydrogen);
  lcd.print(" ppm");

  lcd.setCursor(0,3);
  lcd.print("Methane  : ");
  lcd.print(concentrationMethane);
  lcd.print(" ppm");
  delay(3000);

  lcd.clear();
  lcd.setCursor(0,0);

  if (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE)
   {lcd.setCursor(3,0);
    lcd.print("Fuel Volume:");
    lcd.setCursor(4,2);
    lcd.print(Available_Capacity);
    lcd.print(" Litres");
  } else {
    
    lcd.print("Out of range");
  }
   
  delay(3000); // 500 milliseconds (adjust as needed)
}

// Task 2: Send data to NodeMCU every 0.5 seconds
  if (currentMillis - previousMillis2 >= interval2) {
    previousMillis2 = currentMillis;

    StaticJsonDocument<1000> doc;

    // Assign collected data to JSON Object
    doc["Pressure"] = Pressure;
    doc["FlowRate"] = FlowRate;

    // Send data to NodeMCU
    serializeJson(doc, nodeMCU);
    nodeMCU.println(); // Add a newline to separate JSON objects
  }

  // Add any other non-blocking tasks here

  

  // Allow other tasks to run (avoid blocking)
  yield();
}