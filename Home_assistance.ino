#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

BlynkTimer timer;

int buttonState_1 = 1;
int buttonState_2 = 1;

int internetState = 1;
int smoke = 0;
int Gate_Sensor = 0;

int buzz_state = 0;
int freq = 100;
unsigned long prev =0,curr=0;


#define AUTH "w3jhMt0X635tHqUkdLPrCuL4Ox-SKYe0"  // Auth Token of the Blynk App
#define WIFI_SSID "DESKTOP-U9VR8ID 5694"                 // Enter Wifi Name
#define WIFI_PASS "awkps5929c"                     // Enter wifi Password

// Pins of ESP32
#define RELAY_PIN_1 26  //D26
#define RELAY_PIN_2 27  //D27

#define PUSH_BUTTON_1 33  //D32
#define PUSH_BUTTON_2 32  //D33

#define WIFI_LED 2  //Turning blue light in esp32 if internet is connected

#define Gate_Sensor_Pin 4 // Gate Sensor ESP32
#define Buzzer_Pin 5
#define Smoke_Pin 34

// Pins of Blynk
#define VPIN_BUTTON_1 V1  // Button1
#define VPIN_BUTTON_2 V2  // Button2
#define VPIN_DOOR_4 V4  // for door
#define VPIN_SMOKE_5 V5  // for smoke

void toggleRelay(int relay) {

  switch (relay) {
    case 1:
      if (buttonState_1 == 0) {
        digitalWrite(RELAY_PIN_1, HIGH);  // turn on relay 1
        buttonState_1 = 1;
      } else {
        digitalWrite(RELAY_PIN_1, LOW);  // turn off relay 1
        buttonState_1 = 0;
      }
      delay(200);
      break;
    case 2:
      if (buttonState_2 == 0) {
        digitalWrite(RELAY_PIN_2, HIGH);  // turn on relay 2
        buttonState_2 = 1;
      } else {
        digitalWrite(RELAY_PIN_2, LOW);  // turn off relay 2
        buttonState_2 = 0;
      }
      delay(200);
      break;
    default: break;
  }
}

BLYNK_CONNECTED() {
  // Request the latest state from the server
  Blynk.syncVirtual(VPIN_BUTTON_1);
  Blynk.syncVirtual(VPIN_BUTTON_2);
}

// When App button is pushed - switch the state
BLYNK_WRITE(VPIN_BUTTON_1) {
  buttonState_1 = param.asInt();
  digitalWrite(RELAY_PIN_1, buttonState_1);
}

BLYNK_WRITE(VPIN_BUTTON_2) {
  buttonState_2 = param.asInt();
  digitalWrite(RELAY_PIN_2, buttonState_2);
}

void with_internet() {
  if (digitalRead(PUSH_BUTTON_1) == LOW) {
    toggleRelay(1);
    // Update Button Widget
    Blynk.virtualWrite(VPIN_BUTTON_1, buttonState_1);
  }
  if (digitalRead(PUSH_BUTTON_2) == LOW) {
    toggleRelay(2);
    // Update Button Widget
    Blynk.virtualWrite(VPIN_BUTTON_2, buttonState_2);
  }
}
void without_internet() {
  if (digitalRead(PUSH_BUTTON_1) == LOW) {
    toggleRelay(1);
  }
  if (digitalRead(PUSH_BUTTON_2) == LOW) {
    toggleRelay(2);
  }
}

void BlynkStatus() {  // called every 2 seconds by SimpleTimer

  bool isconnected = Blynk.connected();
  if (isconnected == false) {
    internetState = 0;
    digitalWrite(WIFI_LED, LOW);
    Serial.println("No internet!");
    if (WiFi.status() != WL_CONNECTED){
      WiFi.begin(WIFI_SSID,WIFI_PASS);
      Serial.printf("Trying to connect \"%s\" WI-FI.....\n",WIFI_SSID);
    }
  }
  else if (isconnected == true) {
    internetState = 1;
    digitalWrite(WIFI_LED, HIGH);
    Serial.println("Blynk Server Reachable.............");
    Serial.println(String(smoke)+"% Smoke Detected!");
    if (Gate_Sensor == 1) {
      Blynk.virtualWrite(VPIN_DOOR_4, "Door Close.");
      Serial.println("Door Close.");
    }
    else if(Gate_Sensor == 0)
    {
      Blynk.logEvent("door", "Door Open!");
      Blynk.virtualWrite(VPIN_DOOR_4, "Door Open!");
      Serial.println("Door Open!");
    }
             
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(Gate_Sensor_Pin, INPUT_PULLDOWN);
  pinMode(Buzzer_Pin, OUTPUT);
  Serial.print("DOOR   CONFIGURED.");
  pinMode(Smoke_Pin,INPUT);

  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(PUSH_BUTTON_1, INPUT_PULLUP);
  digitalWrite(RELAY_PIN_1, buttonState_1);

  pinMode(RELAY_PIN_2, OUTPUT);
  pinMode(PUSH_BUTTON_2, INPUT_PULLUP);
  digitalWrite(RELAY_PIN_2, buttonState_2);

  pinMode(WIFI_LED, OUTPUT);
  digitalWrite(WIFI_LED, LOW);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  timer.setInterval(2000L, BlynkStatus);  // check if Blynk server is connected every 2 seconds
  Blynk.config(AUTH);

  tone(Buzzer_Pin, 2500);
  delay(100);
  noTone(Buzzer_Pin);
}


void loop() {
  smoke = map(analogRead(Smoke_Pin),0,4095,0,100);
  if(smoke){
    if(smoke>=50 && buzz_state == 0){
      tone(Buzzer_Pin,2500);
      buzz_state = 1;
      if(internetState = 1) Blynk.logEvent("smoke","Smoke Detected above threshold. Take Action!");
    }
    else if(buzz_state == 1 && smoke<30){
      noTone(Buzzer_Pin);
      buzz_state = 0;
    }
  }

  Gate_Sensor = digitalRead(Gate_Sensor_Pin);
  // Serial.println("\nGate_Sensor: " + Gate_Sensor);

  Blynk.run();
  timer.run();  //initiates SimpleTimer
  if (internetState == 1){
    with_internet();
    if(millis() - prev >= 100){
      prev = millis();
      Blynk.virtualWrite(VPIN_SMOKE_5,smoke);
      }
  }
  else
    without_internet();
}
