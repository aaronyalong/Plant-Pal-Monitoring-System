#include <Arduino.h>

// Wifi/Cloud 
#include <HttpClient.h>
#include <WiFi.h>
#include <inttypes.h>
#include <stdio.h>
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <inttypes.h>

// Temperature and Humidity Sensor
#include "DHT20.h"

// LCD
#include <TFT_eSPI.h>

using namespace std;

// Constructors
DHT20 DHT(&Wire1);              // Temperature and Humidity Sensors
TFT_eSPI tft=TFT_eSPI();        // LCD Display

// Pins
const int sensorPin = 27;     // UV sensor Pin
const int ledPin = 26;        // LED for Normal
const int ledPin2 = 25;       // LED for Alert
const int soilSensorPin = 33;    // Soil moisture sensor pin
const int soilPower = 32;   // Soil sensor power pin

// Soil moisture thresholds
int thresholdUp = 400;
int thresholdDown = 250;

// Time Variables
const int delayTime = 500;
const int calibrationTime = 10000;
int start = millis();

// Light Variables
int lightInit;      // Initial light value
int lightVal;       // Current light reading
int lightMax = 0;   // Maximum light value 
int lightMin = 0;   // Minimum light value

// Predictive Analysis Variables 
float prevHumidity = 0; 
unsigned long lastWateringTime = 0; 

// Cloud Variables
char ssid[50] = "Megan's (2)";     
char pass[50] = "Megatron2002";          
// Name of the server we want to connect to 
// const char kHostname[] = "worldtimeapi.org";
const char kHostname[] = "PLANTPAL.org";
// Path to download (this is a bit after the hostname in the URL that you want to download)
// const char kPath[] = "/api/timezone/Europe/London.txt";
const char kPath[] = "PLANTPAL.txt";
// Numbder of milliseconds to wait without receiving any data before we give up 
const int kNetworkTimeout = 30 * 1000;
// Number of milliseconds to wait if no data is available before trying again
const int kNetworkDelay = 1000;

void nvs_access(){
  // Initialize NVS
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {

        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  // Open 
  Serial.printf("\n");
  Serial.printf("Opening Non-Volatile Storage (NVS) handle...");
  nvs_handle_t my_handle;

  err = nvs_open("storage", NVS_READWRITE, &my_handle);

  // Error Check 
  if (err != ESP_OK){
    Serial.printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  } else {
    Serial.printf("Done\n");
    Serial.printf("Retrieving SSID/PASSWD\n");
    size_t ssid_len;
    size_t pass_len;
    err = nvs_set_str(my_handle, "ssid", ssid);
    err |= nvs_set_str(my_handle, "pass", pass);
    switch (err) {
      case ESP_OK:
        Serial.printf("Done\n");
        // Serial.printf("SSID = %s\n", ssid);
        // Serial.printf("PASSWD = %s\n", pass);
        break;
      case ESP_ERR_NVS_NOT_FOUND:
        Serial.printf("The value is not initialized yet!\n");
        break;
      default:
        Serial.printf("Error (%s) reading! \n", esp_err_to_name(err));
    }
  }

  // Close
  nvs_close(my_handle);
}

void LCD_init() {
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_SKYBLUE);             // Clear screen
  tft.setTextSize(1);                     // Set text size
}

void Temp_Humid_init() {
  Serial.println(__FILE__);
  Serial.print("DHT20 LIBRARY VERSION: ");
  Serial.println(DHT20_LIB_VERSION);
  Serial.println();
  Wire1.begin(21, 22);  // PIN CONNECTIONS
  delay(2000);
  Serial.println("Type,\tStatus,\tHumidity (%),\tTemperature (C)");
}


void LightExposure_Init() {
    // Pins and initial Reading 
  pinMode(ledPin, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(sensorPin, INPUT);
  lightInit = analogRead(sensorPin);

  // 10 second Calibration Time
  while (millis() - start < calibrationTime) {
  tft.drawString("Light Calibration (10 seconds)", 0, 20, 2);

    // Reead light
    lightVal = analogRead(sensorPin); 

    // Update Min and Max Light values
    lightMin = min(lightVal, lightMin);
    lightMax = max(lightVal, lightMax);

    // Blink Light
    digitalWrite(ledPin2, !digitalRead(ledPin2));
    delay(delayTime);
  }
  tft.fillScreen(TFT_SKYBLUE);
}

void Soil_Moisture_init() {
  pinMode(soilPower, OUTPUT);
  digitalWrite(soilPower, LOW);
}

void setup() {
  Serial.begin(9600);
  delay(1000);

  // Retrieve SSID/PASSWD from flash before anything else
  nvs_access();

  // Connecting to a WiFi network
  delay(1000);
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address:");
  Serial.println(WiFi.localIP());
  Serial.println("MAC address: ");
  Serial.println(WiFi.macAddress());

  // LCD Display 
  LCD_init();

  // Soil Moisture 
  Soil_Moisture_init();

  // Temperature & Humidity
  Temp_Humid_init();

  // Light Exposure
  LightExposure_Init();

}

// Helper Functions
int readSoil() {
  digitalWrite(soilPower, HIGH);
  delay(10);
  int val = analogRead(soilSensorPin);
  digitalWrite(soilPower, LOW);
  return val;
}

// Function to determine if watering is needed
bool shouldWater(float temp, float humid, float light, float prevHumid, unsigned long lastTime) {
  // Check if conditions are met to water the plant
  unsigned long timeSinceWatered = millis() - lastTime;
  if ((temp > 25 && light > 500 && humid < 30 && (millis() - lastTime) > 86400000) || // Hot, bright, dry, and it's been a day
      (humid < prevHumid - 5 && (millis() - lastTime) > 43200000)) { // Significant drop in humidity and it's been half a day
    return true;
  }
  return false;
}

void loop() {
  int err = 0;
  WiFiClient c;
  HttpClient http(c);

  //err = http.get(kHostname, kPath);
  std::string URL = "/?var=50&humidity="+std::to_string(DHT.getHumidity())+"&temp="+std::to_string(DHT.getTemperature());
  /*
  std::string URL = "/?var=50"
                + "&humidity=" + std::to_string(DHT.getHumidity())
                + "&temp=" + std::to_string(DHT.getTemperature())
                + "&light=" + std::to_string(lightVal)
                + "&soilMoisture=" + std::to_string(readSoil());
              */

const char* URL_formatted = URL.c_str();

  const char* URL_formatted = URL.c_str();
  err = http.get("18.118.132.82", 5000, URL_formatted, NULL);
  if (err == 0) {
    Serial.println("startedRequest ok");

    err = http.responseStatusCode();
    if (err >= 0) {
      Serial.print("Got status code: ");
      Serial.println(err);

      // Usually you'd check that the response code is 200 ir a similar
      // "success code (200-299) before carrying on, but we'll print out 
      // whatever response we get

      err = http.skipResponseHeaders();
      if (err >= 0) {
        int bodyLen = http.contentLength();
        Serial.print("Content length is: ");
        Serial.println(bodyLen);
        Serial.println();
        Serial.println("Body returned follows: ");

        // Now we've got to the body, so we can print it out
        unsigned long timeoutStart = millis();
        char c;

        // Whilst we haven't timed out & haven't reached the end of the body
        while ((http.connected() || http.available()) &&
              ((millis() - timeoutStart) < kNetworkTimeout)) {
            if (http.available()) {
              c = http.read();

              // Print out this character
              Serial.print(c);

              bodyLen--;

              // we read something, reset the timeout counter
              timeoutStart = millis();
            } else {
              // We haven't got any data, so let's pause to allow some to arrive
              delay(kNetworkDelay);
            }
          }        
      } else {
        Serial.print("Failed to skip response headers: ");
        Serial.println(err);
      }     
    } else {
      Serial.print("Getting response failed: ");
      Serial.println(err);
    }
  } else {
    Serial.print("Connect failed: ");
    Serial.println(err);
  }
  http.stop();

  // UV reading 
  lightVal = analogRead(sensorPin);  
  // Display UV index on the LCD
  tft.drawString("Light Exposure: " + String(lightVal), 0, 40, 2);

  // Adjust LED status based on UV levels
  if (lightVal < lightMin + (lightMax - lightMin) / 3) {
    digitalWrite(ledPin, HIGH); 
    tft.drawString("ALERT: LOW LIGHT EXPOSURE", 0, 60, 2);
  } else if (lightVal > lightMin + 2 * (lightMax - lightMin) / 3) {
    digitalWrite(ledPin2, HIGH);
    tft.drawString("ALERT: HIGH LIGHT EXPOSURE", 0, 60, 2);
  } else {
    digitalWrite(ledPin, LOW);  
    digitalWrite(ledPin2, LOW);
  }

  // TEMP & HUMIDITY Reading
  Serial.print("DHT20, \t");
  int status = DHT.read();
  switch (status)
  {
  case DHT20_OK:
    Serial.print("OK,\t");
    break;
  case DHT20_ERROR_CHECKSUM:
    Serial.print("Checksum error,\t");
    break;
  case DHT20_ERROR_CONNECT:
    Serial.print("Connect error,\t");
    break;
  case DHT20_MISSING_BYTES:
    Serial.print("Missing bytes,\t");
    break;
  case DHT20_ERROR_BYTES_ALL_ZERO:
    Serial.print("All bytes read zero");
    break;
  case DHT20_ERROR_READ_TIMEOUT:
    Serial.print("Read time out");
    break;
  case DHT20_ERROR_LASTREAD:
    Serial.print("Error read too fast");
    break;
  default:
    Serial.print("Unknown error,\t");
    break;
  }

  // Read Data 
  float humidity = DHT.getHumidity();
  float temperature = DHT.getTemperature();

  // Display temperature and humidity on the LCD
  tft.drawString("Temp: " + String(temperature, 1) + " C", 0, 20, 2);
  tft.drawString("Humidity: " + String(humidity, 1) + "%", 0, 0, 2);

  // SOIL MOISTURE Reading 
  int soilMoisture = readSoil();
  tft.drawString("Soil Moisture: " + String(soilMoisture), 0, 100, 2);

  //Display soil moisture status based on thresholds
  String soilStatus;
  if (soilMoisture <= thresholdDown) {
    soilStatus = "Dry, Water it!";
  } else if (soilMoisture >= thresholdUp) {
    soilStatus = "Wet, Leave it!";
  } else {
    soilStatus = "Normal";
  }
  tft.drawString(soilStatus, 0, 120, 2);
    // Predictive Analysis for watering

  if (shouldWater(temperature, humidity, lightVal, prevHumidity, lastWateringTime)) {
    digitalWrite(ledPin2, HIGH); // Alert to water the plant
    tft.drawString("ALERT: Water Plant!", 0, 80, 2);
    lastWateringTime = millis(); // Reset last watering time
  } else {
    digitalWrite(ledPin2, LOW); // No need to water
  }
  prevHumidity = humidity; // Update previous humidity
}


