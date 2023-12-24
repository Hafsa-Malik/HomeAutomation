//Linking the Bynk to Wokwi
#define BLYNK_TEMPLATE_ID "TMPL6wa-X6X56"
#define BLYNK_TEMPLATE_NAME "Smart Home"
#define BLYNK_AUTH_TOKEN "1oOVfb4M1D4wLbwKfVVbmZX5rWEyqxzy"

// Comment this out to disable prints and save space
#define BLYNK_PRINT Serial
#define I2C_ADDR 0x27
#define LCD_COLUMNS 16
#define LCD_LINES 2

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include "DHT.h"
#include "ThingSpeak.h"

// Variable for motion sensing
#define PIR_SENSOR  4
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);


char auth[] = BLYNK_AUTH_TOKEN;

//  WiFi credentials.
char ssid[] = "Wokwi-GUEST";
char pass[] = "";

BlynkTimer timer;
const int myChannelNumber = 2383821;
const char* myApiKey = "O93O8S2L7TZVR14J";
const char* server = "api.thingspeak.com";
WiFiClient client;

// Define pin assignments
//fire detection
#define ldrPin 34
#define buzzerPin 15

#define button1_pin 26
#define button4_pin 32

#define relay1_pin 12
#define relay4_pin 27

int relay1_state = 0;
int relay4_state = 0;

//virtual pins 
#define button1_vpin V1
#define button4_vpin V4

// DHT22
#define DHTPIN 2     // Pin where the DHT11 is connected
#define DHTTYPE DHT22   // DHT 11 sensor type

DHT dht(2, DHT22);

// This function is called every time the device is connected to the Blynk.Cloud
BLYNK_CONNECTED() {
  Blynk.syncVirtual(button1_vpin);
  Blynk.syncVirtual(button4_vpin);
}

// This function is called every time the Virtual Pin state changes

BLYNK_WRITE(button4_vpin) {
  relay4_state = param.asInt();
  digitalWrite(relay4_pin, relay4_state);
}

void setup() {
  pinMode(PIR_SENSOR, INPUT);

  // Debug console
  //fire detection
  pinMode(ldrPin, INPUT);
  pinMode(buzzerPin, OUTPUT);

  pinMode(button1_pin, INPUT_PULLUP);
  pinMode(button4_pin, INPUT_PULLUP);

  pinMode(relay1_pin, OUTPUT);
  pinMode(relay4_pin, OUTPUT);

  // During startup, all Relays should be turned OFF
  digitalWrite(relay1_pin, LOW);
  digitalWrite(relay4_pin, HIGH);
  Serial.begin(9600);
  Blynk.begin(auth, ssid, pass);

  dht.begin(); //dht starts
  ledcSetup(0, 1000, 8); 
  lcd.init();
  lcd.backlight();
  lcd.begin(20, 4);
  lcd.setCursor(3, 0);
  lcd.print("Home Automation");

  //THINGSPEAK
  ThingSpeak.begin(client);

  timer.setInterval(15000L, updateThingSpeak);
}

void loop() {
  Blynk.run();
  timer.run();

  // Read temperature and humidity values from DHT sensor
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  // Check if reading is successful
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Display temperature and humidity on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TEMP:");
  lcd.setCursor(8, 0);
  lcd.print(temperature);
  lcd.setCursor(0, 2);
  lcd.print("HUMID:");
  lcd.setCursor(8, 2);
  lcd.print(humidity);
  Blynk.virtualWrite(V6, humidity);     // Send humidity value to Blynk App
  Blynk.virtualWrite(V5, temperature);  // Send temperature value to Blynk App

  // Listen for button presses
  listenPushButtons();
  if (digitalRead(relay1_pin) == HIGH) {
    Serial.println("Button1 is pressed. Checking for motion...");
    notifyOnTheft();
  }
  else if (digitalRead(relay1_pin) == LOW) {
    Blynk.virtualWrite(button1_vpin, 0);
  }

  //fire detection part:
  int sensorValue = analogRead(ldrPin);


  if ((sensorValue < 100) && (temperature > 50)) {
    tone(buzzerPin, 1000);
    Serial.println(" fire detected. ");
    Blynk.virtualWrite(V2, 1);

  } else {
    noTone(buzzerPin);
    Blynk.virtualWrite(V2, 0);
  }
}

// Function to handle button presses
void listenPushButtons() {
  if (digitalRead(button1_pin) == LOW) {
    controlRelay(1);
  }
  else if (digitalRead(button4_pin) == LOW) {
    controlRelay(4);
    Blynk.virtualWrite(button4_vpin, relay4_state); // Update button state
  }
}

// Function to control relays based on button presses
void controlRelay(int relay) {
  if (relay == 1) {
    relay1_state = !relay1_state;
    digitalWrite(relay1_pin, relay1_state);
    Serial.print("Relay1 State = ");
    Serial.println(relay1_state);
  }
  else if (relay == 4) {
    relay4_state = !relay4_state;
    digitalWrite(relay4_pin, relay4_state);
    Serial.print("Relay4 State = ");
    Serial.println(relay4_state);
  }
}



// Function to handle theft alerts
void notifyOnTheft()
{
  Serial.println("Checking button state...");

  // Check if button1 is pressed (relay1 is on)
  if (digitalRead(button1_pin) == HIGH) {
    Serial.println("Button1 is pressed. Checking for motion...");

    // Check for motion alert
    int isTheftAlert = digitalRead(PIR_SENSOR);
    Serial.println("Motion sensor state: " + String(isTheftAlert));

    if (isTheftAlert == HIGH) {
      Serial.println("Theft Alert in Home");
      Blynk.virtualWrite(button1_vpin, 1);
    }
    else {
      Blynk.virtualWrite(button1_vpin, 0);
    }
  }

}
void updateThingSpeak() {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  if (digitalRead(PIR_SENSOR) == HIGH && digitalRead(button1_pin) == HIGH) {
    ThingSpeak.setField(3,1);
  }
  else{
    ThingSpeak.setField(3,0);
  }

  int sensorValue = analogRead(ldrPin);

  if (sensorValue < 50) {
    ThingSpeak.setField(5,1);
  } else {
    ThingSpeak.setField(5,0);
  }

  ThingSpeak.setField(1,temperature);
  ThingSpeak.setField(2,humidity);
  ThingSpeak.setField(4,relay4_state);

  int x = ThingSpeak.writeFields(myChannelNumber,myApiKey);
  if(x == 200){
    Serial.println("Data pushed successfully");
  }else{
    Serial.println("Push error" + String(x));
  }
  Serial.println("---");
}