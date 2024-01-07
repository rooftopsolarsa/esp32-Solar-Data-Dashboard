/*
  ESP32 Servo Retro Style dashBoard using WS2812B LED Strips, NODERED or HOMEASSIST MQTT and OTA Firmware Updateable - For Sunsynk5.5
  Based on original idea by: [HedgeSlammer](https://powerforum.co.za/profile/27886-hedgeslammer/)
  
    Copyright (C) 2024  RoofTopSolarSA

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>
*/
#include <ArduinoJson.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);
#include <PubSubClient.h>

#define DEBUG 1    // SET TO 0 TO Disable serial debugging info or 1 to show serial debugging info
#if DEBUG
#define D_SerialBegin(...) Serial.begin(__VA_ARGS__);
#define D_print(...)       Serial.print(__VA_ARGS__);
#define D_write(...)       Serial.write(__VA_ARGS__);
#define D_println(...)     Serial.println(__VA_ARGS__);
#else
#define D_SerialBegin(bauds);
#define D_print(...);
#define D_write(...);
#define D_println(...);
#endif

#include <FastLED.h>
FASTLED_USING_NAMESPACE
#define NUM_LEDS 300 // add number of LEDs of your RGB LED strip
#define PIN_LED 23 // digital output PIN that is connected to DIN of the RGB LED strip
#define COLOR_ORDER GRB
#define LED_COLOR CRGB::Red // see https://github.com/FastLED/FastLED/wiki/Pixel-reference for a full list, e.g. CRGB::AliceBlue, CRGB::Amethyst, CRGB::AntiqueWhite...
CRGB rgb_led[NUM_LEDS]; // color array of the LED RGB strip
#define FRAMES_PER_SECOND 60
#define Brightness_Pin A3  // LED's Brightness_Pin
int BRIGHTNESS = 1;
#define FlowSensorSpeed_Pin A0  // FlowSensorSpeed_Pin
int flowSpeed = 50;
TBlendType    currentBlending;

// SERVO CALIBRATIONS - Be sure to calibrate your servo's before running this code!!

#define SERVOMIN0  106 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX0  600 // this is the 'maximum' pulse length count (out of 4096)
#define SERVOMIN1  96 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX1  605 // this is the 'maximum' pulse length count (out of 4096)
#define SERVOMIN2  96 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX2  605 // this is the 'maximum' pulse length count (out of 4096)
#define SERVOMIN3  96 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX3  605 // this is the 'maximum' pulse length count (out of 4096)
#define SERVOMIN4  96 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX4  605 // this is the 'maximum' pulse length count (out of 4096)
#define SERVOMIN5  96 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX5  605 // this is the 'maximum' pulse length count (out of 4096)
#define SERVOMIN6  96 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX6  605 // this is the 'maximum' pulse length count (out of 4096)
#define SERVOMIN7  96 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX7  605 // this is the 'maximum' pulse length count (out of 4096)
#define SERVOMIN8  96 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX8  605 // this is the 'maximum' pulse length count (out of 4096)
#define SERVOMIN9  96 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX9  605 // this is the 'maximum' pulse length count (out of 4096)
#define SERVOMIN10  96 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX10  605 // this is the 'maximum' pulse length count (out of 4096)
// our servo # counter
int servoNumber = 0;
int invVpulse = 0;
int GridWattspulse = 0;
int InvertWattspulse = 0;
int NonELoadsWattspulse = 0;
int ELoadsWattspulse = 0;
float BatVoltpulse = 0;
float BatWattspulse = 0;
float BatAmpspulse = 0;
int BatSOCpulse = 0;
int Mppt1pulse = 0;
int Mppt2pulse = 0;
int PVTotalpulse = 0;
int buzzerPin = 14;

bool stateSolar = false;
bool stateBatAmps = false;
bool stateToEssentialLoads = false;
bool stateGridToinverter = false;
bool stateGridToNonEssentials = false;
bool stateInverterToNonEssentials = false;

char message_buff[16];
char message_buffe[16];
String pubString;
String pubStringe;
int iTotalDelay;
const char* versionNumber = "Online-Ver a1008";
// ------------------------------------------------------------
// Start Editing here
const char* ssid = "YOURWIFINETWORKNAME";
const char* password = "YOURWIFIPASSWORD";
const char* mqtt_username = "YOURMQTTUSERNAME";
const char* mqtt_password = "YOURMQTTPASSWORD";
const char* mqtt_server = "YOURMQTTIPORHOSTNAME";
const char* host = "esp32-retrostats";
const char* noderedtopic = "YOURMQTTDATATOPIC"; // Subscribe to SOLAR DATA MQTT Topic From Nodered or HA
int batteryLowWarning = 30; // Percentage at which to beep for low battery warning - default 30 %
// Stop Editing here
// Making changes below this line could cause unexpected results
// -------------------------------------------------------------
//
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(512)
char msg[MSG_BUFFER_SIZE];
int value = 0;
volatile  bool TopicArrived = false;
const int mqttpayloadSize = 512;
char mqttpayload [mqttpayloadSize] = {'\0'};
String mqtttopic;

void callback(char* topic, byte* payload, unsigned int length)
{
  if ( !TopicArrived )
  {
    memset( mqttpayload, '\0', mqttpayloadSize ); // clear payload char buffer
    mqtttopic = ""; //clear topic string buffer
    mqtttopic = topic; //store new topic
    memcpy( mqttpayload, payload, length );
    TopicArrived = true;
  }
}

// Wifi Client
WiFiClient wifiClient;
//

//
// Starts WIFI connection
//
void startWIFI()
{
  // If we are not connected
  if (WiFi.status() != WL_CONNECTED)
  {
    int iTries;
    iTries = 0;
    Serial.println(" ");
    Serial.println("Starting WIFI connection");
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.begin(ssid, password);

    // If not WiFi connected, retry every 2 seconds for 15 minutes
    while (WiFi.status() != WL_CONNECTED)
    {
      iTries++;
      Serial.print(".");
      delay(2000);

      // If can't get to Wifi for 15 minutes, reboot ESP Default 450
      if (iTries > 50)
      {
        Serial.println("TOO MANY WIFI ATTEMPTS, REBOOTING!");
        ESP.restart();  /*ESP restart function*/
      }
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println(WiFi.localIP());
    // Let network have a chance to start up
    delay(1500);
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect to mqtt
    if (client.connect("esp32RetroStatsDisplay", mqtt_username, mqtt_password)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("retro/RetroStatsDisplay", versionNumber);
      // ... and resubscribe
      client.subscribe(noderedtopic); //Get DataSet From NodeRed
    }
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  mqtttopic.reserve(100);
  //  setup_wifi();
  startWIFI();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  pinMode(BUILTIN_LED, OUTPUT);
  FastLED.addLeds<WS2812B, PIN_LED, COLOR_ORDER>(rgb_led, NUM_LEDS);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 500);
  currentBlending = NOBLEND; //LINEARBLEND
  FastLED.clear();
  FastLED.show();

  Wire.begin();
  board1.begin();
  board1.setPWMFreq(60);
}

void loop(void) {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  int FlowSensorValue = analogRead(FlowSensorSpeed_Pin); // A0 pin 36
  flowSpeed = map(FlowSensorValue, 0, 1023, 1, 150);
  int LED_BsensorValue = analogRead(Brightness_Pin); // A3 pin 39
  BRIGHTNESS = map(LED_BsensorValue, 0, 1023, 1, 50);

  D_println();
  String readableTime;
  getReadableTime(readableTime);
  D_print("DashBoard Running For: ");
  D_println(readableTime);
  //D_println(" ago");
  
  D_print("LED Brightness: ");
  D_println(BRIGHTNESS);
  
  if ( TopicArrived )
  {
    //parse topic
    D_print("RetroDisplayData: ");
    D_println( mqttpayload );
    TopicArrived = false;
    //size_t inputLength;
    StaticJsonDocument<384> doc;
    DeserializationError error = deserializeJson(doc, mqttpayload);
    if (error) {
      D_print("deserializeJson() failed: ");
      D_println(error.c_str());
      return;
    }
    /* json raw data sample from MQTT
      {
      "BatterySOC": 93,
      "InvV": 2319,
      "GridW": 49,
      "InvW": 632,
      "Non Essential Loads Power": 5,
      "Essential Loads Power": 676,
      "BatV": 4971,
      "batW": 607,
      "BatAmps": 1223, 111,
      "mppt1": 76,
      "mppt2": 59,
      "PV Total Power": 135,
      "LoadsheddingLocal":1,
      "LSStage":3
      }
    */
    float BatSOC = doc["BatterySOC"]; // 0 to 100 %
    int InvertVolt = doc["InvV"];
    float InverterVoltage = InvertVolt / 10; // 200 to 260 AC Volts
    int GridWatts = doc["GridW"]; // 0 to 6000 watts
    int InvertWatts = doc["InvW"]; // 0 to 6000 watts
    int NonELoadsWatts = doc["Non Essential Loads Power"]; // 0 to 6000 watts
    int ELoadsWatts = doc["Essential Loads Power"]; // 0 to 6000 watts
    float BatVolt = doc["BatV"];
    float BatVoltX = BatVolt / 100; // 40 to 60 Volts
    float BatWatts = doc["batW"];
    float BatWattsX = BatWatts * -1; // -5000 to 5000 watts
    float BatAmps = doc["BatAmps"];
    float BatAmpsX = (BatAmps / 100) * -1; // -100 to 50 amps
    int Mppt1 = doc["mppt1"]; // 0 to 2500 watts
    int Mppt2 = doc["mppt2"]; // 0 to 2500 watts
    int PVTotal = doc["PV Total Power"]; // 0 to 6000 watts
    int LSLocalOnOff = doc["LoadsheddingLocal"]; // yes or no 1 or 0
    int LSStages = doc["LSStage"]; // None to 8 0 to 8

    // Battery SOC % - SERVO 0
    D_print("Battery SOC %: ");
    D_print(BatSOC, 2);
    if (BatSOC <= batteryLowWarning) {
      beepBeep();
    }
    BatSOCpulse = map(BatSOC, 0, 100, 180, 0);
    int SBatSOCpulse = map(BatSOCpulse, 0, 180, SERVOMIN0, SERVOMAX0);
    board1.setPWM(0, 0, SBatSOCpulse);
    D_print(" Battery SOC % Servo0 Angle: ");
    D_print(BatSOCpulse);
    D_print(" Battery SOC % Servo0 Pulse: ");
    D_println(SBatSOCpulse);

    // Inverter Voltage - SERVO 1
    D_print("Inverter Voltage: ");
    D_print(InverterVoltage, 0);
    invVpulse = map(InverterVoltage, 200, 260, 180, 0);
    int SinvVpulse = map(invVpulse, 0, 180, SERVOMIN1, SERVOMAX1);
    board1.setPWM(1, 0, SinvVpulse);
    D_print(" Inverter Voltage Servo1 Angle: ");
    D_print(invVpulse);
    D_print(" Inverter Voltage Servo1 Pulse: ");
    D_println(SinvVpulse);

    // Grid Watts - Eskom - SERVO 2
    D_print("Grid Watts: ");
    D_print(GridWatts, 0);
    GridWattspulse = map(GridWatts, 0, 6000, 180, 0);
    int SGridWattspulse = map(GridWattspulse, 0, 180, SERVOMIN2, SERVOMAX2);
    board1.setPWM(2, 0, SGridWattspulse);
    D_print(" Grid Watts Servo2 Angle: ");
    D_print(GridWattspulse);
    D_print(" Grid Watts Servo2 Pulse: ");
    D_println(SGridWattspulse);

    // Inverter Watts - SERVO 3
    D_print("Inverter Watts: ");
    D_print(InvertWatts, 0);
    InvertWattspulse = map(InvertWatts, 0, 6000, 180, 0);
    int SInvertWattspulse = map(InvertWattspulse, 0, 180, SERVOMIN3, SERVOMAX3);
    board1.setPWM(3, 0, SInvertWattspulse);
    D_print(" Inverter Watts Servo3 Angle: ");
    D_print(InvertWattspulse);
    D_print(" Inverter Watts Servo3 Pulse: ");
    D_println(SInvertWattspulse);

    // Non Essentials Watts - SERVO 4
    D_print("Non Essentials Watts: ");
    D_print(NonELoadsWatts, 0);
    NonELoadsWattspulse = map(NonELoadsWatts, 0, 6000, 180, 0);
    int SNonELoadsWattspulse = map(NonELoadsWattspulse, 0, 180, SERVOMIN4, SERVOMAX4);
    board1.setPWM(4, 0, SNonELoadsWattspulse);
    D_print(" Non Essentials Watts Servo4 Angle: ");
    D_print(NonELoadsWattspulse);
    D_print(" Non Essentials Watts Servo4 Pulse: ");
    D_println(SNonELoadsWattspulse);

    // Essentials Watts - SERVO 5
    D_print("Essentials Watts: ");
    D_print(ELoadsWatts, 0);
    if (ELoadsWatts <= 0) {
      stateToEssentialLoads = false;
    }
    else if (ELoadsWatts > 0) {
      stateToEssentialLoads = true;
    }
    else;
    ELoadsWattspulse = map(ELoadsWatts, 0, 6000, 180, 0);
    int SELoadsWattspulse = map(ELoadsWattspulse, 0, 180, SERVOMIN5, SERVOMAX5);
    board1.setPWM(5, 0, SELoadsWattspulse);
    D_print(" Essentials Watts Servo5 Angle: ");
    D_print(ELoadsWattspulse);
    D_print(" Essentials Watts Servo5 Pulse: ");
    D_println(SELoadsWattspulse);

    // Battery Voltage - SERVO 6
    D_print("Battery Voltage: ");
    D_print(BatVoltX, 2);
    BatVoltpulse = map(BatVoltX, 48, 54, 180, 0);
    int SBatVoltpulse = map(BatVoltpulse, 0, 180, SERVOMIN6, SERVOMAX6);
    board1.setPWM(6, 0, SBatVoltpulse);
    D_print(" Battery Voltage Servo6 Angle: ");
    D_print(BatVoltpulse);
    D_print(" Battery Voltage Servo6 Pulse: ");
    D_println(SBatVoltpulse);

    // Battery Charge/Discharge Amps - SERVO 7
    D_print("Battery Amps: ");
    D_print(BatAmpsX, 2);
    if (BatAmpsX < 0) {
      stateBatAmps = false;
    }
    else if (BatAmpsX > 0) {
      stateBatAmps = true;
    }
    else;
    BatAmpspulse = map(BatAmpsX, -100, 50, 0, 180);
    int SBatAmpspulse = map(BatAmpspulse, 0, 180, SERVOMIN7, SERVOMAX7);
    board1.setPWM(7, 0, SBatAmpspulse);
    D_print(" Battery Amps Servo7 Angle: ");
    D_print(BatAmpspulse);
    D_print(" Battery Amps Servo7 Pulse: ");
    D_println(SBatAmpspulse);

    // MPPT 1 Watts - SERVO 8
    D_print("MPPT 1 Watts: ");
    D_print(Mppt1, 0);
    Mppt1pulse = map(Mppt1, 0, 3000, 180, 0);
    int SMppt1pulse = map(Mppt1pulse, 0, 180, SERVOMIN8, SERVOMAX8);
    board1.setPWM(8, 0, SMppt1pulse);
    D_print(" MPPT 1 Watts Servo8 Angle: ");
    D_print(Mppt1pulse);
    D_print(" MPPT 1 Servo8 Pulse: ");
    D_println(SMppt1pulse);

    // MPPT 2 Watts - SERVO 9
    D_print("MPPT 2 Watts: ");
    D_print(Mppt2, 0);
    Mppt2pulse = map(Mppt2, 0, 3000, 180, 0);
    int SMppt2pulse = map(Mppt2pulse, 0, 180, SERVOMIN9, SERVOMAX9);
    board1.setPWM(9, 0, SMppt2pulse);
    D_print(" MPPT 2 Watts Servo9 Angle: ");
    D_print(Mppt2pulse);
    D_print(" MPPT 2 Servo9 Pulse: ");
    D_println(SMppt2pulse);
    
    // Total PV Watts - SERVO 10
    D_print("Total PV Watts: ");
    D_print(PVTotal, 0);
    if (PVTotal > 5) {
      stateSolar = true;
    }
    else stateSolar = false;
    PVTotalpulse = map(PVTotal, 0, 6000, 180, 0);
    int SPVTotalpulse = map(PVTotalpulse, 0, 180, SERVOMIN10, SERVOMAX10);
    board1.setPWM(10, 0, SPVTotalpulse);
    D_print(" Total PV Watts Servo10 Angle: ");
    D_print(PVTotalpulse);
    D_print(" Total PV Servo10 Pulse: ");
    D_println(SPVTotalpulse);
    
    D_println();
    D_print("Loadshedding National State: ");
    D_print(LSStages);
    LoadshedIndicate_National(LSStages);
    D_println(" bigger then 0 = Yes - Show National Loadshedding LED Status");
    D_print("Loadshedding Local State: ");
    D_print(LSLocalOnOff);
    LoadshedIndicate_Local(LSLocalOnOff);
    D_println(" bigger then 0 = Yes - Show Local Loadshedding LED Status");
    D_print("Loadshedding Stage: ");
    D_print(LSStages);
    D_println(" (LEDs 1 to 8)");
    StagesLeds(LSStages);
    green();
    orange();
    red();
    D_println();
  }
  
  if (stateToEssentialLoads )
  { ToEssentialLoads_led();
    D_println("Inverter to Essential Loads - LED's Green Direction to Essential Loads (LEDs 37 to 47)");
  }
  if (stateSolar )
  { solar_in_led();
    D_println("Solar In - LED's Green Direction to Inverter (LEDs 15 to 25)");
  }
  else {
    solar_out_led();
    D_println("Solar Off - LED's Black (LEDs 15 to 25)");
  }
  if (stateBatAmps )
  { BatAmps_Ch_led();
    D_println("Battery Charging - LED's Green Direction to Inverter (LEDs 26 to 36)");
  }
  else {
    BatAmps_DiCh_led();
    D_println("Battery dis-Charging - LED's Red Direction to Battery(LEDs 26 to 36)");
  }

} // end main loop

void beepBeep() {
  D_println(" ");
  D_println("BEEP BEEP BEEP BEEP BEEP BEEP");
  static unsigned long previousMillis1 = 0;
  unsigned long currentMillis1 = millis();
  static int state = 0;  // Store the current state
  static int beepDuration = 250;  // Store the beep duration
  if (currentMillis1 - previousMillis1 >= beepDuration) {
    previousMillis1 = currentMillis1;
    state++; if (state >= 10) state = 0;
    if (state == 1 || state == 3)
      digitalWrite(buzzerPin, HIGH);
    else digitalWrite(buzzerPin, LOW);
  }
}

void StagesLeds(int tLSST) {
  // Loadshedding Stage received from MQTT Data
  int theLoadshedStages = tLSST;
  D_print("Loadshed Number LED bars lit: ");
  D_println(theLoadshedStages);
  //Number LEDs mapped to loadshed stage
  int num_LS_leds_switchedon = map(theLoadshedStages, 0, 8, 0, 8);
  //Light up the LEDs
  for (int i = 0; i < num_LS_leds_switchedon; ++i) {
    rgb_led[i] = LED_COLOR;
  }
  // LEDs are switched off
  for (int i = num_LS_leds_switchedon; i < 8; ++i) {
    rgb_led[i] = CRGB::Black;
  }
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.show();
}

void LoadshedIndicate_National(int tLSLonOff) {
  for (int tLSLonOffi = 297; tLSLonOffi < 300; tLSLonOffi++) {
    if (tLSLonOff > 0) {
      rgb_led[tLSLonOffi] = CRGB::Red;
    }
    else {
      rgb_led[tLSLonOffi] = CRGB::Blue;
    }
    FastLED.setBrightness(BRIGHTNESS);
    FastLED.show();
  }
}

void LoadshedIndicate_Local(int tLSNonOff) {
  for (int tLSNonOffi = 292; tLSNonOffi < 295; tLSNonOffi++) {
    if (tLSNonOff > 0) {
      rgb_led[tLSNonOffi] = CRGB::Red;
    }
    else {
      rgb_led[tLSNonOffi] = CRGB::Blue;
    }
    FastLED.setBrightness(BRIGHTNESS);
    FastLED.show();
  }
}

void green() {
  for (int i = 8; i < 10; i++) {
    rgb_led[i] = CRGB::Green;
    FastLED.setBrightness(BRIGHTNESS);
    FastLED.show();
  }
}

void orange() {
  for (int i = 10; i < 12; i++) {
    rgb_led[i] = CRGB::Orange;
    FastLED.setBrightness(BRIGHTNESS);
    FastLED.show();
  }
}

void red() {
  for (int i = 12; i < 15; i++) {
    rgb_led[i] = CRGB::Red;
    FastLED.setBrightness(BRIGHTNESS);
    FastLED.show();
  }
}

void fadeall() {
  for (int i = 0; i < NUM_LEDS; i++) {
    rgb_led[i].nscale8(250);
  }
}

void solar_in_led() {
  // Move a single white led
  for (int SolarInLed = 15; SolarInLed < 25; SolarInLed = SolarInLed + 1) {
    rgb_led[SolarInLed] = CRGB::Green;
    FastLED.setBrightness(BRIGHTNESS);
    FastLED.show();
    delay(flowSpeed);
    rgb_led[SolarInLed] = CRGB::Black;
  }
}

void solar_out_led() {
  for (int SolarOutLed = 25; SolarOutLed >= 15; SolarOutLed = SolarOutLed - 1) {
    rgb_led[SolarOutLed] = CRGB::Black;
    FastLED.setBrightness(BRIGHTNESS);
    FastLED.show();
    delay(flowSpeed);
    rgb_led[SolarOutLed] = CRGB::Black;
  }
}

void BatAmps_Ch_led() {
  for (int BatAmps_Ch = 26; BatAmps_Ch < 36; BatAmps_Ch = BatAmps_Ch + 1) {
    rgb_led[BatAmps_Ch] = CRGB::Green;
    FastLED.setBrightness(BRIGHTNESS);
    FastLED.show();
    delay(flowSpeed);
    rgb_led[BatAmps_Ch] = CRGB::Black;
  }
}

void BatAmps_DiCh_led() {
  for (int BatAmps_DiCh = 36; BatAmps_DiCh >= 26; BatAmps_DiCh = BatAmps_DiCh - 1) {
    rgb_led[BatAmps_DiCh] = CRGB::Red;
    FastLED.setBrightness(BRIGHTNESS);
    FastLED.show();
    delay(flowSpeed);
    rgb_led[BatAmps_DiCh] = CRGB::Black;
  }
}

void ToEssentialLoads_led() {
  for (int ToEssentialLoads = 38; ToEssentialLoads < 48; ToEssentialLoads = ToEssentialLoads + 1) {
    rgb_led[ToEssentialLoads] = CRGB::Green;
    FastLED.setBrightness(BRIGHTNESS);
    FastLED.show();
    delay(flowSpeed);
    rgb_led[ToEssentialLoads] = CRGB::Black;
  }
}

void getReadableTime(String &readableTime) {
  unsigned long currentMillis;
  unsigned long seconds;
  unsigned long minutes;
  unsigned long hours;
  unsigned long days;
  currentMillis = millis();
  seconds = currentMillis / 1000;
  minutes = seconds / 60;
  hours = minutes / 60;
  days = hours / 24;
  currentMillis %= 1000;
  seconds %= 60;
  minutes %= 60;
  hours %= 24;
  if (days > 0) {
    readableTime = String(days) + " days ";
  }
  if (hours > 0) {
    readableTime += String(hours) + " hours ";
  }
  if (minutes < 10) {
    readableTime += "0";
  }
  readableTime += String(minutes) + " minutes and ";
  if (seconds < 10) {
    readableTime += "0";
  }
  readableTime += String(seconds) + " seconds";
}
