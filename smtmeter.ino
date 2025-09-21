#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "EmonLib.h"
#include "math.h"
#include "cmath"

//Instance of energy monitor for Voltage and Current
EnergyMonitor emon1;  // for current
EnergyMonitor emon2;  // for voltage

bool neighborHasPower = true;
bool myPowerStatus;
//declaring constatnts for number of samples
const int NO_OF_VOLTAGE_SAMPLES = 1480;
const int NO_OF_CURRENT_SAMPLES = 3000;

//constants for VoltageSensor Pin and CurrentSensor Pin
const int currentSensorPin = 34;
const int voltageSensorPin = 36;
const int potPin = 33;

//rates of domestic and commercial
const double domesticRate = 6.2;
const double commercialRate = 8.0;

//Variables to store the voltage and curent values
double Irms = 0;
double Vrms = 0;
double smoothedCurrent = 0;
double power = 0;
double pf = 0;
double energyWh = 0;
double domesticHourRate = 0;
double commercialHourRate = 0;

unsigned long previousMillis = 0;  // Previous time in milliseconds
unsigned long interval = 1000;     // Sampling interval (e.g., 1 second)

//variables to store the rebate/penalty prices
int value = 0;
int timein24hrs = 0;
double basePrice = 8.0;
double penaltyPrice = 1.0;
double rebatePrice = -1.0;
double adjustedPrice = 0;
double finalPrice = 0;

//variables for status
bool rebateStatus = false;
bool normalStatus = false;
bool penaltyStatus = false;

//Callibaration Factor
const float voltageSensitivity = 0.0048;  // Volts per ADC step for ZMPT101B
const float currentSensitivity = 0.001;   // Amps per ADC step for ZMCT103
const float voltageOffset = 0.9;          // Zero point for voltage sensor (centered at VCC/2)
const float currentOffset = 0.9;          // Zero point for current sensor (centered at VCC/2)

// WiFi Credentials
const char* ssid = "OPPO A3 Pro 5G";  // Replace with your Wi-Fi SSID
const char* password = "abhi@123";    // Replace with your Wi-Fi Password

// HiveMQ Cloud Broker Settings
const char* mqtt_server = "http://4e5daecda0c04538ba2de3cabb2a8d43.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;                 // Secure MQTT Port
const char* mqtt_username = "hemanth";      // Replace with HiveMQ Username
const char* mqtt_password = "Hemanth@123";  // Replace with HiveMQ Password

// Hardware Pins
#define LED_PIN 2
const int potPin1 = 34;  // Potentiometer 1 (GPIO 34)
const int potPin2 = 36;  // Potentiometer 2 (GPIO 35)

// Secure WiFi Client and MQTT Client
WiFiClientSecure espClient;
PubSubClient client(espClient);

// Connect to Wi-Fi
void setup_wifi() {
  Serial.print("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWi-Fi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// Callback for handling incoming MQTT messages
void callback(char* topic, byte* payload, unsigned int length) {
  char msg[50];
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);

  for (unsigned int i = 0; i < length; i++) {
    msg[i] = (char)payload[i];
  }
  msg[length] = '\0';



  if (strcmp(topic, "/neighbor_power_status") == 0) {
    if (strcmp(msg, "1") == 0) {
      neighborHasPower = true;
      Serial.println("Neighbor has power");
    } else {
      neighborHasPower = false;
      Serial.println("Neighbor has no power");
    }
  }
}

// Reconnect to MQTT Broker if disconnected
void reconnect() {
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    if (client.connect("ESP32Client", mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT broker!");
      client.subscribe("/LedControl");  // Subscribe to LED control topic
    } else {
      Serial.print("Failed to connect, rc=");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

double calcPowerFactor() {

  // Sampling parameters
  const int samples = 2000;              // Number of samples
  const float samplingFrequency = 5000;  // Sampling frequency (Hz)

  float sumVoltageSquared = 0;
  float sumCurrentSquared = 0;
  float sumProduct = 0;

  // Variables to store sampled values
  float voltage = 0;
  float current = 0;

  for (int i = 0; i < samples; i++) {
    // Read raw sensor values
    float rawVoltage = analogRead(voltageSensorPin);
    float rawCurrent = analogRead(currentSensorPin);

    // Convert raw readings to actual voltage and current
    voltage = (rawVoltage / 4095.0 * 3.3 - voltageOffset) / voltageSensitivity;
    current = (rawCurrent / 4095.0 * 3.3 - currentOffset) / currentSensitivity;

    // Accumulate values for RMS and real power calculation
    sumVoltageSquared += voltage * voltage;
    sumCurrentSquared += current * current;
    sumProduct += voltage * current;

    // Delay to control sampling rate
    delayMicroseconds(1000000 / samplingFrequency);
  }

  // Calculate RMS for voltage and current
  float voltageRMS = sqrt(sumVoltageSquared / samples);
  float currentRMS = sqrt(sumCurrentSquared / samples);

  // Calculate real power (average of the product of voltage and current)
  float realPower = sumProduct / samples;

  // Calculate apparent power (RMS voltage * RMS current)
  float apparentPower = voltageRMS * currentRMS;

  // Return power factor
  //Serial.print("Power Factor: ");
  return fabs(realPower / apparentPower);  // Power Factor = Real Power / Apparent Power
}

void setup() {
  Serial.begin(115200);
  emon1.current(currentSensorPin, 4);           // Pin, callibration constant
  emon2.voltage(voltageSensorPin, 110.0, 1.7);  // Pin, calibration factor, phase shift

  // Configure pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(potPin, INPUT);  //using potentiometer to vary time

  // Connect to Wi-Fi
  setup_wifi();

  // Set up MQTT connection
  espClient.setInsecure();  // Use secure connection without explicit certificate
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  // Ensure MQTT client is connected
  if (!client.connected()) {
    reconnect();
  }
  client.loop();


  emon2.calcVI(NO_OF_VOLTAGE_SAMPLES, 2000);  // Samples, timeout in milliseconds
  Vrms = emon2.Vrms;
  Serial.print("Voltage: ");
  Serial.print(Vrms);
  Serial.println(" V");

  if (Vrms < 30) {  // Assuming 1000 as threshold for no power
    myPowerStatus = false;
  } else {
    myPowerStatus = true;
  }

  // Check if you have no power but the neighbor has power
  if (!myPowerStatus && neighborHasPower) {
    Serial.println("No power detected, contact technician!");
    digitalWrite(LED_PIN, HIGH);  // Turn on LED to indicate no power
    // Here, you can add logic to trigger a notification, send an alert, etc.
  } else {
    digitalWrite(LED_PIN, LOW);  // Turn off LED if power is restored
  }

  Irms = emon1.calcIrms(NO_OF_CURRENT_SAMPLES);
  smoothedCurrent = (smoothedCurrent * 0.5) + (Irms * 0.1);  //smothing the current output
  Serial.print("Current: ");
  Serial.print(smoothedCurrent);
  Serial.println(" A");


  power = (Vrms * smoothedCurrent);
  Serial.print("Power: ");
  Serial.print(power);
  Serial.println(" W");

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;

    float deltaTime = interval / 3600000.0;  // Interval in hours
    energyWh += power * deltaTime;
  }
  Serial.print("Energy Consumed: ");
  Serial.print(energyWh);
  Serial.println(" Wh");

  pf = calcPowerFactor();
  Serial.print("Power Factor: ");
  Serial.println(pf);

  domesticHourRate = (power / 1000) * domesticRate;
  Serial.print("Domestic Per Hour Rate: ");
  Serial.println(domesticHourRate);

  value = analogRead(potPin);
  timein24hrs = map(value, 0, 4095, 0, 23);

  if (timein24hrs >= 22 || timein24hrs <= 2) {
    rebateStatus = true;
    normalStatus = false;
    penaltyStatus = false;
    adjustedPrice = commercialRate + rebatePrice;
  }

  else if (timein24hrs > 2 && timein24hrs <= 18) {
    rebateStatus = false;
    normalStatus = true;
    penaltyStatus = false;
    adjustedPrice = commercialRate;
    Serial.println("");
  }

  else if (timein24hrs > 18 && timein24hrs < 22) {
    rebateStatus = false;
    normalStatus = false;
    penaltyStatus = true;
    adjustedPrice = commercialRate + penaltyPrice;
  }

  commercialHourRate = (power / 1000) * adjustedPrice;
  Serial.print("Commercial Per Hour Rate: ");
  Serial.println(commercialHourRate);

  finalPrice = (energyWh / 1000) * domesticRate;
  Serial.print("Final Price: ");
  Serial.println(finalPrice);
  char voltageValue[50], currentValue[50], instantaneousPowerValue[50];
  char powerFactorValue[50], domesticHrRate[50], commercialHrRate[50], energy[50];
  char normal[50], penalty[50], rebate[50], final[50];

  snprintf(voltageValue, sizeof(voltageValue), "%lf", Vrms);
  snprintf(currentValue, sizeof(currentValue), "%lf", smoothedCurrent);
  snprintf(instantaneousPowerValue, sizeof(instantaneousPowerValue), "%lf", power);
  snprintf(powerFactorValue, sizeof(powerFactorValue), "%lf", pf);
  snprintf(domesticHrRate, sizeof(domesticHrRate), "%lf", domesticHourRate);
  snprintf(commercialHrRate, sizeof(commercialHrRate), "%lf", commercialHourRate);
  snprintf(energy, sizeof(energy), "%lf", energyWh);
  snprintf(normal, sizeof(normal), "%d", normalStatus);
  snprintf(penalty, sizeof(penalty), "%d", penaltyStatus);
  snprintf(rebate, sizeof(rebate), "%d", rebateStatus);
  snprintf(final, sizeof(final), "%lf", finalPrice);

  client.publish("/Pot1", currentValue);
  client.publish("/Pot2", voltageValue);
  client.publish("/Pot3", instantaneousPowerValue);
  client.publish("/Pot4", powerFactorValue);
  client.publish("/Pot5", domesticHrRate);
  client.publish("/Pot6", commercialHrRate);
  client.publish("/Pot7", energy);
  client.publish("/Pot8", final);


  delay(200);  // Publish values every 1 second
}