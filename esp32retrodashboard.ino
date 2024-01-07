/*
  ESP32 Servo Retro Style dashBoard using WS2812B LED Strips, NODERED or HOMEASSIST MQTT and OTA Firmware Updateable - For Sunsynk5.5
  
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

// SERVO CALIBRATIONS
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

  int FlowSensorValue = 200; // analogRead(FlowSensorSpeed_Pin); // A0 pin 36
  flowSpeed = map(FlowSensorValue, 0, 1023, 1, 150);
  int LED_BsensorValue = 1; // analogRead(Brightness_Pin); // A3 pin 39
  BRIGHTNESS = map(LED_BsensorValue, 0, 1023, 1, 50);

  Serial.println();
  String readableTime;
  getReadableTime(readableTime);
  Serial.print("DashBoard Running For: ");
  Serial.println(readableTime);
  //Serial.println(" ago");
  
  Serial.print("LED Brightness: ");
  Serial.println(BRIGHTNESS);
  
  if ( TopicArrived )
  {
    //parse topic
    Serial.print("RetroDisplayData: ");
    Serial.println( mqttpayload );
    TopicArrived = false;
    //size_t inputLength;
    StaticJsonDocument<384> doc;
    DeserializationError error = deserializeJson(doc, mqttpayload);
    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
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
    Serial.print("Battery SOC %: ");
    Serial.print(BatSOC, 2);
    if (BatSOC <= batteryLowWarning) {
      beepBeep();
    }
    BatSOCpulse = map(BatSOC, 0, 100, 180, 0);
    int SBatSOCpulse = map(BatSOCpulse, 0, 180, SERVOMIN0, SERVOMAX0);
    board1.setPWM(0, 0, SBatSOCpulse);
    Serial.print(" Battery SOC % Servo0 Angle: ");
    Serial.print(BatSOCpulse);
    Serial.print(" Battery SOC % Servo0 Pulse: ");
    Serial.println(SBatSOCpulse);

    // Inverter Voltage - SERVO 1
    Serial.print("Inverter Voltage: ");
    Serial.print(InverterVoltage, 0);
    invVpulse = map(InverterVoltage, 200, 260, 180, 0);
    int SinvVpulse = map(invVpulse, 0, 180, SERVOMIN1, SERVOMAX1);
    board1.setPWM(1, 0, SinvVpulse);
    Serial.print(" Inverter Voltage Servo1 Angle: ");
    Serial.print(invVpulse);
    Serial.print(" Inverter Voltage Servo1 Pulse: ");
    Serial.println(SinvVpulse);

    // Grid Watts - Eskom - SERVO 2
    Serial.print("Grid Watts: ");
    Serial.print(GridWatts, 0);
    GridWattspulse = map(GridWatts, 0, 6000, 180, 0);
    int SGridWattspulse = map(GridWattspulse, 0, 180, SERVOMIN2, SERVOMAX2);
    board1.setPWM(2, 0, SGridWattspulse);
    Serial.print(" Grid Watts Servo2 Angle: ");
    Serial.print(GridWattspulse);
    Serial.print(" Grid Watts Servo2 Pulse: ");
    Serial.println(SGridWattspulse);

    // Inverter Watts - SERVO 3
    Serial.print("Inverter Watts: ");
    Serial.print(InvertWatts, 0);
    InvertWattspulse = map(InvertWatts, 0, 6000, 180, 0);
    int SInvertWattspulse = map(InvertWattspulse, 0, 180, SERVOMIN3, SERVOMAX3);
    board1.setPWM(3, 0, SInvertWattspulse);
    Serial.print(" Inverter Watts Servo3 Angle: ");
    Serial.print(InvertWattspulse);
    Serial.print(" Inverter Watts Servo3 Pulse: ");
    Serial.println(SInvertWattspulse);

    // Non Essentials Watts - SERVO 4
    Serial.print("Non Essentials Watts: ");
    Serial.print(NonELoadsWatts, 0);
    NonELoadsWattspulse = map(NonELoadsWatts, 0, 6000, 180, 0);
    int SNonELoadsWattspulse = map(NonELoadsWattspulse, 0, 180, SERVOMIN4, SERVOMAX4);
    board1.setPWM(4, 0, SNonELoadsWattspulse);
    Serial.print(" Non Essentials Watts Servo4 Angle: ");
    Serial.print(NonELoadsWattspulse);
    Serial.print(" Non Essentials Watts Servo4 Pulse: ");
    Serial.println(SNonELoadsWattspulse);

    // Essentials Watts - SERVO 5
    Serial.print("Essentials Watts: ");
    Serial.print(ELoadsWatts, 0);
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
    Serial.print(" Essentials Watts Servo5 Angle: ");
    Serial.print(ELoadsWattspulse);
    Serial.print(" Essentials Watts Servo5 Pulse: ");
    Serial.println(SELoadsWattspulse);

    // Battery Voltage - SERVO 6
    Serial.print("Battery Voltage: ");
    Serial.print(BatVoltX, 2);
    BatVoltpulse = map(BatVoltX, 48, 54, 180, 0);
    int SBatVoltpulse = map(BatVoltpulse, 0, 180, SERVOMIN6, SERVOMAX6);
    board1.setPWM(6, 0, SBatVoltpulse);
    Serial.print(" Battery Voltage Servo6 Angle: ");
    Serial.print(BatVoltpulse);
    Serial.print(" Battery Voltage Servo6 Pulse: ");
    Serial.println(SBatVoltpulse);

    // Battery Charge/Discharge Amps - SERVO 7
    Serial.print("Battery Amps: ");
    Serial.print(BatAmpsX, 2);
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
    Serial.print(" Battery Amps Servo7 Angle: ");
    Serial.print(BatAmpspulse);
    Serial.print(" Battery Amps Servo7 Pulse: ");
    Serial.println(SBatAmpspulse);

    // MPPT 1 Watts - SERVO 8
    Serial.print("MPPT 1 Watts: ");
    Serial.print(Mppt1, 0);
    Mppt1pulse = map(Mppt1, 0, 3000, 180, 0);
    int SMppt1pulse = map(Mppt1pulse, 0, 180, SERVOMIN8, SERVOMAX8);
    board1.setPWM(8, 0, SMppt1pulse);
    Serial.print(" MPPT 1 Watts Servo8 Angle: ");
    Serial.print(Mppt1pulse);
    Serial.print(" MPPT 1 Servo8 Pulse: ");
    Serial.println(SMppt1pulse);

    // MPPT 2 Watts - SERVO 9
    Serial.print("MPPT 2 Watts: ");
    Serial.print(Mppt2, 0);
    Mppt2pulse = map(Mppt2, 0, 3000, 180, 0);
    int SMppt2pulse = map(Mppt2pulse, 0, 180, SERVOMIN9, SERVOMAX9);
    board1.setPWM(9, 0, SMppt2pulse);
    Serial.print(" MPPT 2 Watts Servo9 Angle: ");
    Serial.print(Mppt2pulse);
    Serial.print(" MPPT 2 Servo9 Pulse: ");
    Serial.println(SMppt2pulse);
    
    // Total PV Watts - SERVO 10
    Serial.print("Total PV Watts: ");
    Serial.print(PVTotal, 0);
    if (PVTotal > 5) {
      stateSolar = true;
    }
    else stateSolar = false;
    PVTotalpulse = map(PVTotal, 0, 6000, 180, 0);
    int SPVTotalpulse = map(PVTotalpulse, 0, 180, SERVOMIN10, SERVOMAX10);
    board1.setPWM(10, 0, SPVTotalpulse);
    Serial.print(" Total PV Watts Servo10 Angle: ");
    Serial.print(PVTotalpulse);
    Serial.print(" Total PV Servo10 Pulse: ");
    Serial.println(SPVTotalpulse);
    
    Serial.println();
    Serial.print("Loadshedding National State: ");
    Serial.print(LSStages);
    LoadshedIndicate_National(LSStages);
    Serial.println(" bigger then 0 = Yes - Show National Loadshedding LED Status");
    Serial.print("Loadshedding Local State: ");
    Serial.print(LSLocalOnOff);
    LoadshedIndicate_Local(LSLocalOnOff);
    Serial.println(" bigger then 0 = Yes - Show Local Loadshedding LED Status");
    Serial.print("Loadshedding Stage: ");
    Serial.print(LSStages);
    Serial.println(" (LEDs 1 to 8)");
    StagesLeds(LSStages);
    green();
    orange();
    red();
    Serial.println();
  }
  
  if (stateToEssentialLoads )
  { ToEssentialLoads_led();
    Serial.println("Inverter to Essential Loads - LED's Green Direction to Essential Loads (LEDs 37 to 47)");
  }
  if (stateSolar )
  { solar_in_led();
    Serial.println("Solar In - LED's Green Direction to Inverter (LEDs 15 to 25)");
  }
  else {
    solar_out_led();
    Serial.println("Solar Off - LED's Black (LEDs 15 to 25)");
  }
  if (stateBatAmps )
  { BatAmps_Ch_led();
    Serial.println("Battery Charging - LED's Green Direction to Inverter (LEDs 26 to 36)");
  }
  else {
    BatAmps_DiCh_led();
    Serial.println("Battery dis-Charging - LED's Red Direction to Battery(LEDs 26 to 36)");
  }

} // end main loop

void beepBeep() {
  Serial.println(" ");
  Serial.println("BEEP BEEP BEEP BEEP BEEP BEEP");
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
  Serial.print("Loadshed Number LED bars lit: ");
  Serial.println(theLoadshedStages);
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
      rgb_led[tLSLonOffi] = CRGB::RED;
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
      rgb_led[tLSNonOffi] = CRGB::RED;
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
