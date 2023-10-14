#include <MPU6050_tockn.h>
#include <Wire.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
// #include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <BlynkSimpleEsp8266.h> 

BlynkTimer Btimer;


MPU6050 mpu6050(Wire);
const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
boolean fall = false; // stores if a fall has occurred
boolean trigger1 = false;
boolean trigger2 = false;
boolean trigger3 = false;
byte trigger1count = 0;
byte trigger2count = 0;
byte trigger3count = 0;
int angleChange = 0;
long timer = 0;
int steps = 0;
float distanceinonestep = 71;
float distance;
const int oneWireBus = 0;
unsigned long prev =0,curr=0;

OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

int internetState = 1;

//const char *ssid = "Alsan";
//const char *pass = "01234567890";

#define AUTH "w3jhMt0X635tHqUkdLPrCuL4Ox-SKYe0"  // Auth Token of the Blynk App
// #define WIFI_SSID "DESKTOP-U9VR8ID 5694"                 // Enter Wifi Name
// #define WIFI_PASS "awkps5929c"                     // Enter wifi Password

const char* WIFI_SSID     = "DESKTOP-U9VR8ID 5694";
const char* WIFI_PASS = "awkps5929c";

#define VPIN_FALL_6 V6  // for fall
#define VPIN_WALK_7 V7  // for walk
#define VPIN_TEMP_8 V8  // for temperature
#define WIFI_LED 2  //Turning blue light in esp32 if internet is connected


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
    Serial.println(steps);
    Blynk.virtualWrite(VPIN_WALK_7,steps);
  }
}

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  pinMode(WIFI_LED, OUTPUT);
  digitalWrite(WIFI_LED, LOW);
//    Serial.println("Wrote to IMU");
//    mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  sensors.begin();
  // WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Btimer.setInterval(2000L, BlynkStatus);  // check if Blynk server is connected every 2 seconds
  Blynk.config(AUTH);

}

void loop()
{
    mpu_read();
    ax = (AcX - 2050) / 16384.00;
    ay = (AcY - 77) / 16384.00;
    az = (AcZ - 1947) / 16384.00;
    gx = (GyX + 270) / 131.07;
    gy = (GyY - 351) / 131.07;
    gz = (GyZ + 136) / 131.07;
    float Raw_Amp = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5);
    int Amp = Raw_Amp * 10;
    // Serial.println(Amp);
    if (Amp <= 2 && trigger2 == false)
    {
        trigger1 = true;
        Serial.println("TRIGGER 1 ACTIVATED");
    }
    if (trigger1 == true)
    {
        trigger1count++;
        if (Amp >= 12)
        { // if AM breaks upper threshold (3g)
            trigger2 = true;
            Serial.println("TRIGGER 2 ACTIVATED");
            trigger1 = false;
            trigger1count = 0;
        }
    }
    if (trigger2 == true)
    {
        trigger2count++;
        angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
        Serial.println(angleChange);
        if (angleChange >= 30 && angleChange <= 400)
        {
            trigger3 = true;
            trigger2 = false;
            trigger2count = 0;
            Serial.println(angleChange);
            Serial.println("TRIGGER 3 ACTIVATED");
        }
    }
    if (trigger3 == true)
    {
        trigger3count++;
        if (trigger3count >= 10)
        {
            angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
            Serial.println(angleChange);
            if ((angleChange >= 0) && (angleChange <= 10))
            {
                fall = true;
                trigger3 = false;
                trigger3count = 0;
                Serial.println(angleChange);
            }
            else
            {
                trigger3 = false;
                trigger3count = 0;
                Serial.println("TRIGGER 3 DEACTIVATED");
            }
        }
    }
    if (fall == true)
    {
        Serial.println("\n\n\nFall Detected\n\n\n");
        if(internetState == 1) Blynk.logEvent("fall", "Fall detected!");
        fall = false;
    }
    if (trigger2count >= 6)
    { // allow 0.5s for orientation change
        trigger2 = false;
        trigger2count = 0;
        Serial.println("TRIGGER 2 DECACTIVATED");
    }
    if (trigger1count >= 6)
    { // allow 0.5s for AM to break upper threshold
        trigger1 = false;
        trigger1count = 0;
        Serial.println("TRIGGER 1 DECACTIVATED");
    }
    mpu6050.update();
    if (mpu6050.getAccY() > 1)
    {
        steps += 1;
        delay(150);
    }

    Serial.print("steps: ");
    Serial.println(steps);
    Serial.print("metres: ");
    Serial.println(steps * distanceinonestep / 100);
    sensors.requestTemperatures(); 
    float temperatureF = sensors.getTempFByIndex(0);
    Serial.print(temperatureF);
    Serial.println("ºF");
    delay(50);

  Blynk.run();
  Btimer.run();  //initiates SimpleTimer
  if (internetState == 1){
    // with_internet();
    if(millis() - prev >= 100){
      prev = millis();
      Blynk.virtualWrite(VPIN_TEMP_8,temperatureF);
      }
  }
}
void mpu_read()
{
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
    AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}