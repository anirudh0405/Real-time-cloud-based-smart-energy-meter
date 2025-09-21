#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// Wi-Fi credentials
const char* ssid = "OPPO A3 Pro 5G";  // Replace with your Wi-Fi SSID
const char* password = "abhi@123";  // Replace with your Wi-Fi Password

// HiveMQ Cloud MQTT broker credentials
const char* mqtt_server = "6e146f116f5e406ebec5cfebc1f8385b.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_username = "hemanth";  // Replace with HiveMQ username
const char* mqtt_password = "Hemanth@123";  // Replace with HiveMQ password

// Hardware Pins
#define BUTTON_PIN 27 // Pin to connect the push button (GPIO 0)
//#define LED_PIN 2     // Pin to indicate button press status (LED)

WiFiClientSecure espClient;
PubSubClient client(espClient);

// Function to connect to Wi-Fi
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

// Callback function for incoming MQTT messages (not used in this case)
void callback(char* topic, byte* payload, unsigned int length) {
  // Not used in this case, but can be expanded if needed.
}

// Reconnect to the MQTT broker if disconnected
void reconnect() {
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    if (client.connect("NeighborBoard", mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT broker as Neighbor Board!");
    } else {
      Serial.print("Failed to connect, rc=");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT);  // Set the button pin as input (with internal pull-up)
  //pinMode(LED_PIN, OUTPUT);           // Set the LED pin as output

  // Connect to Wi-Fi
  setup_wifi();

  // Set up MQTT connection
  espClient.setInsecure(); // Use secure connection without explicit certificate
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  // Ensure MQTT client is connected
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Read the button state (HIGH means no press, LOW means button is pressed)
  int buttonState = digitalRead(BUTTON_PIN);

  // If button is pressed (LOW), send "1" (indicating neighbor has power)
  if (buttonState == LOW) {
    Serial.println("Button pressed: Neighbor has power!");
    //digitalWrite(LED_PIN, HIGH);  // Turn on LED to indicate button press
    client.publish("/neighbor_power_status", "1");  // Send "1" to indicate power
  }
  else {
    // If button is not pressed, send "0" (indicating no power)
    Serial.println("Button not pressed: Neighbor has no power.");
   // digitalWrite(LED_PIN, LOW);   // Turn off LED to indicate no button press
    client.publish("/neighbor_power_status", "0");  // Send "0" to indicate no power
  }

  delay(1000);  // Check the button status every second
}