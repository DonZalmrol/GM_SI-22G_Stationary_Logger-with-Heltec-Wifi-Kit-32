#include <heltec.h>
#include <WiFi.h>
#include <ArduinoHttpClient.h>
#include <movingAvg.h>
#include <esp_task_wdt.h>
#include <DHTesp.h>
#include <ESPPerfectTime.h>
#include "arduino_secrets.h"

#define WDT_TIMEOUT 75 //75 seconds watchdog timer

int increaseSecCount = 1;

float temperature = 0.0, humidity = 0.0, pressure = 0.0, tubeVoltage = 0.0;

unsigned long cps_1 = 0, cps_2 = 0;
unsigned long actual_cps_1 = 0, actual_cps_2 = 0;
unsigned long cpm = 0, cpm_temp = 0;
unsigned long sensorMovingAvg = 0;
unsigned long previousMillis_1 = 0, previousMillis_2 = 0;

// Set sensor to 120 data points for moving average (2x 60 second data entry points)
movingAvg cps_sensor(120);

// Set DHT
DHTesp dht;

boolean event1 = false, event2 = false;
boolean buttonShowLCD = false;

time_t epoch = 0;

// Tube dead time in seconds
// SI-22G = 0.000xxx seconds
// SBM-20 = 0.000190 seconds
#define tubeDeadTime 0.000190

// Tube conversion value
// SI-22G = 0.001714
// SBM-20 = 0.006315
#define tubeConversionFactor 0.001714

// WiFi credentials
const char *ssid = SECRET_SSID;
const char *password = SECRET_PASS;
const long timeoutTime = 5000;

// RADMON credentials
const char *UserName = SECRET_USER_NAME;
const char *DataSendingPassWord = SECRET_USER_PASS_01;

// URadMonitoring credentials
const char *USER_ID = SECRET_USER_ID;
const char *USER_KEY = SECRET_USER_KEY;
const char *DEVICE_ID = SECRET_DEVICE_ID;

// Set NTP server
const char *ntpServer = "europe.pool.ntp.org";

// Create a task handle
TaskHandle_t uploadTask;

void setup()
{
  // Initialize watchdog
  Serial.println(F("\n"));
  Serial.println(F("Initializing WatchDogTimer (WDT)"));
  Serial.println(F("\n"));
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  
  // True = Display, False = LoRa, True = Serial
  Heltec.begin(true, false, true);
  Heltec.display->flipScreenVertically();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->clear();
  Heltec.display->drawString(0, 0, "GM SI-22G");
  Heltec.display->drawString(0, 10, "Stationary Logger");
  Heltec.display->drawString(0, 20, "V6");
  Heltec.display->drawString(0, 40, "By Don Zalmrol");
  Heltec.display->display();
  delay(2500);
  Heltec.display->clear();

  // Initialize variables
  cps_1 = 0, cps_2 = 0;
  actual_cps_1 = 0, actual_cps_2 = 0;
  sensorMovingAvg = 0;
  increaseSecCount = 1;

  // Initialize sensor
  cps_sensor.begin();

  // Initialize Arduino pins
  pinMode(0, INPUT_PULLUP);   // Set pin0 (GPIO0) input for button (PCB and external)
  pinMode(25, OUTPUT);        // Set pin25 (GPIO25) as output for LED upload = OK
  pinMode(34, INPUT_PULLUP);  // Set pin12 (GPIO12) input for capturing GM Tube events
  pinMode(35, INPUT_PULLUP);  // Set pin14 (GPIO14) input for capturing GM Tube events
  pinMode(21, OUTPUT);        // Set pin21 (GPIO21) as output for OLED
  pinMode(26, OUTPUT);        // Set pin04 (GPIO26) as output for LED
  
  // Allow external interrupts on INT0 and INT1
  attachInterrupt(digitalPinToInterrupt(34), tube_impulse1, FALLING); //define external interrupts
  attachInterrupt(digitalPinToInterrupt(35), tube_impulse2, FALLING); //define external interrupts

  // Set pin13 (GPIO13) as input for DHT sensor
  dht.setup(13, DHTesp::DHT11);

  Serial.println(F("--- Start program ---"));
  Serial.println(F("\n"));
  
  // Initialize UART on 115200 baud rate for ethernet shield
  Serial.begin(115200);
  delay(1000);

  // Connect to the WiFi
  WiFiSetup();

  // Configure SNTP Client
  pftime::configTzTime(PSTR("CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00"), ntpServer);
}

void loop()
{
  // if button is pressed turn on oled
  if((digitalRead(0)== LOW) && (buttonShowLCD == false))
  {
    buttonShowLCD = true;
    //Serial.println("button pushed = " + String(digitalRead(0)));
  }

  // turn off oled by setting contrast to 0
  if(buttonShowLCD == false)
  {
    Heltec.display->clear();
    Heltec.display->setContrast(0);
    Heltec.display->display();
    Heltec.display->clear();
  }

  if ((event1 == true) || (event2 == true))
  {
    // 1 blink for 50 ms
    blinkLed(1, 50);
    
    event1 = false, event2 = false;
  }

  // If reached 01 seconds with a total of 60 seconds
  if (((millis() - previousMillis_1) >= 1000UL) && (increaseSecCount <= 60))
  {
    // Recalute with tube dead time accounted for actual CPS of both tubes
    actual_cps_1 = (cps_1 / (1- (cps_1 * tubeDeadTime)));
    actual_cps_2 = (cps_2 / (1- (cps_2 * tubeDeadTime)));

    // Add a new datapoint entry for both tubes to the moving average array
    sensorMovingAvg = cps_sensor.reading(actual_cps_1);
    sensorMovingAvg = cps_sensor.reading(actual_cps_2);

    Serial.println(F("--- 01 sec ---"));
    Serial.println("| Tube 1 " + String(increaseSecCount) + " sec current count (cps_1) = " + String(actual_cps_1));
    Serial.println("| Tube 2 " + String(increaseSecCount) + " sec current count (cps_2) = " + String(actual_cps_2));
    Serial.println("| Tubes current total mavg " + String(increaseSecCount) + " sec current count = " + String(sensorMovingAvg));
    Serial.println(F("--- 01 sec ---"));

    // Print to LCD
    if(buttonShowLCD)
    {
      Heltec.display->clear();
      Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
      Heltec.display->drawString(0, 0, "Current sec. = " + String(increaseSecCount) + " of 60");
      Heltec.display->drawString(0, 10, "CPS 1 = " + String(actual_cps_1) + " | CPS 2 = " + String(actual_cps_2));
      Heltec.display->drawString(0, 20, "MA-CPS = " + String(sensorMovingAvg));
      Heltec.display->drawString(0, 30, "Current uSv = " + String(outputSieverts(sensorMovingAvg)));
      Heltec.display->drawString(0, 40, "Tubes voltage = " + String(displayTubeVoltage()) + " V");
      Heltec.display->display();
      Heltec.display->clear();
    }
    
    previousMillis_1 = millis();
    increaseSecCount++;
  }
  
  // If 61 seconds have been reached do upload and reset vars
  if (((millis() - previousMillis_2) >= 61000UL) && (increaseSecCount >= 61))
  {
    // Fetch values
    cpm = cps_sensor.getAvg();
    cpm_temp = cpm;
    
    temperature = dht.getTemperature();
    humidity = dht.getHumidity();
    pressure = 0.0;
    tubeVoltage = displayTubeVoltage();

    // Get current time as UNIX time
    epoch = pftime::time(nullptr);

    // Print new line
    Serial.println(F("\n"));

    Serial.println(F("--- 61 sec ---"));
    Serial.println(F("| Commence uploading"));
    Serial.println(F("| Resetting vars"));
    Serial.println(F("--- 61 sec ---"));

    // Print new line
    Serial.println(F("\n"));

    // Initialize Task
    xTaskCreatePinnedToCore
    (
      uploadTaskFunction, /* Function to implement the task */
      "uploadTask", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &uploadTask,  /* Task handle. */
      1 /* Core where the task should run */
    );

    // Reset variables
    cps_1 = 0, cps_2 = 0;
    actual_cps_1 = 0, actual_cps_2 = 0;
    cpm = 0;
    sensorMovingAvg = 0;
    
    increaseSecCount = 1;
    
    cps_sensor.reset();
    buttonShowLCD = false;

    // Reset WatchDogTimer
    esp_task_wdt_reset();

    previousMillis_1 = millis();
    previousMillis_2 = millis();
  }
}

//------------------------//
//--- START functions ---//
//----------------------//
void WiFiSetup()
{
  Serial.println(F("\n"));
  Serial.println("Connecting to = " + String(ssid));
  
  WiFi.begin(ssid, password);
  
  // Set up Wifi connection
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  if(WiFi.status() == WL_CONNECTED)
  {
    Serial.println(F("\n"));
    Serial.println(F("WiFi connected"));
    Serial.println("IP address = " +  String(WiFi.localIP().toString()));
    Serial.println("Gateway = " + String(WiFi.gatewayIP().toString()));
    Serial.println("SubnetMask = " + String(WiFi.subnetMask().toString()));
    Serial.println("DNS 1 = " + String(WiFi.dnsIP(0).toString()));
    Serial.println("DNS 2 = " + String(WiFi.dnsIP(1).toString()));
    Serial.println("MAC = " + String(WiFi.macAddress()));
    Serial.println(F("\n"));
  }

  // Wifi is not connected
  else
  {
      Serial.println(F("WiFi NOT connected"));
      Serial.println(F("\n"));
      
      // Restart ESP32
      ESP.restart();
  }
}

// GM Tube fastpulse 01 counting
void tube_impulse1()
{
  cps_1++;
  event1 = true;
}

// GM Tube fastpulse 02 counting
void tube_impulse2()
{
  cps_2++;
  event2 = true;
}

// Convert counts to uSV
float outputSieverts(float counts) 
{
  float uSV = counts * tubeConversionFactor;
  return uSV;
}

// Measure Tube voltage through A0
float displayTubeVoltage()
{
  // read the input on analog pin 27:
  float adcInput = analogRead(27);

  // Convert the analog reading (12bit resolution which goes from 0 - 4095) to a voltage (0 - 3.3V):
  float lowVoltage = ((adcInput * 3.3) / 4095.0);
  
  // ~190-195 conversion factor according to Alex - RH Electronics 2021-03-15
  // ~171 calculated on 2021-04-01 for Arduino
  // 123.15 calcultated on 2021-05-04 for ESP32
  float highVoltage = (lowVoltage * 123.15);

  // Print out voltage
  //Serial.println("| Tubes HV = " + String(highVoltage) + " Volts, A0 LV = " + String(lowVoltage));

  return highVoltage;
}

// Blink the LED (blinks, time)
void blinkLed(int blinks, int time)
{                        
    for (int i = 0; i < blinks; i++)
    {
      digitalWrite(26, HIGH);
      delay(time);
      digitalWrite(26, LOW);
      delay(time);
    }
}

// Blink the LED (blinks, time)
void blinkLedUpload(int blinks, int time)
{                        
    for (int i = 0; i < blinks; i++)
    {
      digitalWrite(25, HIGH);
      delay(time);
      digitalWrite(25, LOW);
      delay(time);
    }
}

void uploadTaskFunction( void * parameter)
{
  esp_task_wdt_feed();

  // Connect to RadMon.org
  connecToRadMonLogger();
  
  // Connect to Uradmonitor
  connecToURadMonLogger();

  // Reset variables
  epoch = 0;
  cpm_temp = 0;
  temperature = 0.0, humidity = 0.0, pressure = 0.0, tubeVoltage = 0.0;

  esp_task_wdt_delete(NULL);

  vTaskDelete(uploadTask);
}

// Upload data to the RadMon.org server
void connecToRadMonLogger()
{
  esp_task_wdt_feed();
  
  WiFiClient wifi;
  HttpClient client = HttpClient(wifi, "radmon.org", 80);

  Serial.println(F("Connection to radmon monitoring platform succeeded!"));

  // Concat data for POST
  // Basic API URL (doesn't change)
  String ptr = "/radmon.php?function=submit";
  ptr += "&user=";
  ptr += UserName;
  ptr += "&password=";
  ptr += DataSendingPassWord;
  ptr += "&value=";
  ptr += cpm_temp;
  ptr += "&unit=CPM";

  // Test output
  Serial.println("Uploaded CPM = " + String(cpm_temp));
  Serial.println("created PTR = " + ptr);

  client.beginRequest();
  client.post(ptr);
  client.sendHeader("Content-Type", "application/x-www-form-urlencoded");
  client.sendHeader("Content-Length", ptr.length());
  client.beginBody();
  client.print("");
  client.endRequest();

  // Keep statuscode and response in comments during production, used for testing!
  // read the status code and body of the response
  //int statusCode = client.responseStatusCode();
  //String response = client.responseBody();
  //Serial.println("Status code = " + String(statusCode));
  //Serial.println("Response = " + String(response));
  
  // 5 blinks for 50 ms
  blinkLedUpload(5, 50);

  // Give the client some time to stop
  yield();

  esp_task_wdt_delete(NULL);
  
  Serial.println(F("Connection to radmon monitoring platform Disconnected."));
  Serial.println(F(""));
}

// Upload data to the uradmonitor.com server
void connecToURadMonLogger()
{
  esp_task_wdt_feed();
  
  WiFiClient wifi;
  HttpClient client = HttpClient(wifi, "data.uradmonitor.com", 80);

  Serial.println(F("Connection to uradmonitoring platform succeeded!"));
  
  // Concat data for POST
  // Basic API URL (doesn't change)
  String ptr = "/api/v1/upload/exp";
  ptr += "/01/";                // compulsory: local time in seconds
  ptr += epoch;                 // time epoch value
  ptr += "/02/";                // 02 = optional: temperature in degrees celsius
  ptr += temperature;           // temperature value 
  ptr += "/03/";                // 03 = optional: barometric pressure in pascals
  ptr += pressure;              // barometric value
  ptr += "/04/";                // 04 = optional: humidity as relative humidity in percentage %
  ptr += humidity;              // humidity value
  ptr += "/0B/";                // 0B = optional: radiation measured on geiger tube in cpm
  ptr += cpm_temp;              // a-cpm value
  ptr += "/0C/";                // 0C = optional: high voltage geiger tube inverter voltage in volts
  ptr += tubeVoltage;           // tube voltage value
  ptr += "/0E/106/0F/123";      // 0F : 123 = ver_sw : value | 0E : 106 = ver_hw : value
  ptr += "/10/6";               // 10 = Tube ID | 6 (0x6) = GEIGER_TUBE_SI22G
  
  // Test output
  Serial.println("Uploaded CPM = " + String(cpm_temp));
  Serial.println("created EXP code = " + ptr);

  client.beginRequest();
  client.post(ptr);
  client.sendHeader("X-User-id", USER_ID);
  client.sendHeader("X-User-hash", USER_KEY);
  client.sendHeader("X-Device-id", DEVICE_ID);
  client.sendHeader("Content-Type", "application/x-www-form-urlencoded");
  client.sendHeader("Content-Length", ptr.length());
  client.beginBody();
  client.print("");
  client.endRequest();

  // Keep statuscode and response in comments during production, used for testing!
  // read the status code and body of the response
  //int statusCode = client.responseStatusCode();
  //String response = client.responseBody();
  //Serial.println("Status code = " + String(statusCode));
  //Serial.println("Response = " + String(response));

  // 5 blinks for 50 ms
  blinkLedUpload(5, 50);

  // Give the client some time to stop
  yield();

  esp_task_wdt_delete(NULL);
  
  Serial.println(F("Connection to uradmonitoring platform Disconnected."));
  Serial.println(F(""));
}
