#include <ESP8266WiFi.h>
#include <ThingsBoard.h>
#include <FirebaseArduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <DHT.h>

#define FIREBASE_HOST "coalminers-47-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "SlSg9LDxoCR0gSfhz4j6ohO2QcnVW2AyzS00ptis"
#define WIFI_SSID "Redmi"
#define WIFI_PASSWORD "skamalesh"
#define WIFI_AP "Redmi"
#define WIFI_PASSWORD "skamalesh"
#define TOKEN "37J7ap4gFOvPcQIp0qRR"
char thingsboardServer[] = "demo.thingsboard.io";

WiFiClient wifiClient;
ThingsBoard tb(wifiClient);

int status = WL_IDLE_STATUS;
unsigned long lastSend;

void InitWiFi()
{
  Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network

  WiFi.begin(WIFI_AP, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}


void reconnect() {
  // Loop until we're reconnected
  while (!tb.connected()) {
    status = WiFi.status();
    if ( status != WL_CONNECTED) {
      WiFi.begin(WIFI_AP, WIFI_PASSWORD);
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      Serial.println("Connected to ");
      Serial.print(WIFI_AP);
    }
    Serial.print("Connecting to ThingsBoard node ...");
    if ( tb.connect(thingsboardServer, TOKEN) ) {
      Serial.println( "[DONE]" );
    } else {
      Serial.print( "[FAILED]" );
      Serial.println( " : retrying in 5 seconds]" );
      // Wait 5 seconds before retrying
      delay( 5000 );
    }
  }
}

int n;

int sensor_value;
int readD2;
int Pin_D0 = 16;
int Pin_D4 = 2;


Adafruit_MPU6050 mpu; //MPU6050

#define minval -5
#define maxval 3
int Buzzer1 = 15;

int Buzzer2 = 12;   //Heartsensor
int Signal;
int Threshold = 550;

int Buzzer = 13;
const int sensor = A0; //MQ06 sensor and Heart sensor bcause only one analog pin
int sensor_limit = 2000;
int D2;
#define DHTPIN   D2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

int analogRead1() {
  digitalWrite(Pin_D0, HIGH); // Turn D1 On
  digitalWrite(Pin_D4, LOW); // Turn D2 Off
  return analogRead(0);
}

int analogRead2() {
  digitalWrite(Pin_D0, LOW); //  Turn D1 On
  digitalWrite(Pin_D4, HIGH); // Turn D2 Off
  return analogRead(0);
}

void setup()
{
  Serial.begin(9600);        //Temperature sensor
  dht.begin();


  pinMode(Buzzer2, OUTPUT);     //heartbeat!
  pinMode(Pin_D4, OUTPUT);
  pinMode(Pin_D0, OUTPUT);
  delay(1000);

  pinMode(Buzzer, OUTPUT); //MQ06 sensor
  pinMode(sensor, INPUT);

  pinMode(Buzzer1, OUTPUT);  //MPU6050

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to ");
  Serial.print(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
  Serial.print(".");
  InitWiFi();
  lastSend = 0;  
    delay(1000);
  }

  Serial.println();
  Serial.print("Connected\t");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());                               //prints local IP address
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);             // connect to the firebase

  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  Serial.println("Adafruit MPU6050 test!");
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }

  Serial.println("");
  delay(100);

}

void loop()
{
  
  f ( !tb.connected() ) {
    reconnect();
  }

  if ( millis() - lastSend > 1000 ) { // Update and send only after 1 seconds
   temp();
   heart();
   gas();
   quake();
    lastSend = millis();
  }

  tb.loop();

  if (Firebase.failed())
  {

    Serial.print("pushing /logs failed:");
    Serial.println(Firebase.error());
    return;
  }
}

void temp() {
  float h = dht.readHumidity();                                 // Read Humidity
  float t = dht.readTemperature();                              // Read temperature

  if (isnan(h) || isnan(t))                                     // Checking sensor working
  {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  Serial.print("Humidity: ");
  Serial.print(h);
  String fireHumid = String(h) + String("%");                   //Humidity integer to string conversion

  Serial.print("%  Temperature: ");
  Serial.print(t);
  Serial.println("°C ");
  String fireTemp = String(t) + String("°C");                  //Temperature integer to string conversion
  delay(1000);
  tb.sendTelemetryFloat("Temperature", t);
  tb.sendTelemetryFloat("Humidity", h);
  for (int i = 0; i < n; ++i) {
    String k = String(i);
    Firebase.setString("/DHT11/Humidity/" + k, fireHumid);
  }
  for (int i = 0; i < n; ++i) {
    String k = String(i);
    Firebase.setString("/DHT11/Humidity/" + k, fireTemp);
  }
}

void heart() {
  Signal = analogRead2();  // Read the PulseSensor's value

  Serial.println(Signal);                    // Send the Signal value to Serial Plotter.

  if (Signal > Threshold) {                        // If the signal is above "550", then "turn-on" Arduino's on-Board LED.
    digitalWrite(Buzzer2, HIGH);
  } else {
    digitalWrite(Buzzer2, LOW);               //  Else, the sigal must be below "550", so "turn-off" this LED.
  }
  delay(1000);
  tb.sendTelemetryInt("Heartbeat",Signal);

  String firesignal = String(Signal);
  Firebase.pushString("/heartsensor", firesignal);
}

void gas() {
  sensor_value = analogRead1();
  delay(200);
  Serial.print("Gas Level: ");
  Serial.println(sensor_value);
  if (sensor_value > sensor_limit)  // Checks if it has reached the threshold value
  {
    digitalWrite(Buzzer, HIGH);
    delay(300);
    digitalWrite(Buzzer, LOW);
  }
  else
  {
    digitalWrite(Buzzer, LOW);
  }
  delay(100);
  tb.sendTelemetryFloat("Gas Level", sensor_value);
  String firePPM = String(sensor_value) + String("ppm");
  Firebase.pushString("/MQ06/gas level", firePPM);
}

void quake() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if (g.gyro.x > maxval || g.gyro.x < minval && g.gyro.y > maxval || g.gyro.y  < minval && g.gyro.z > maxval || g.gyro.z < minval)

  { digitalWrite(Buzzer1, HIGH);
    delay(300);
    digitalWrite(Buzzer1, HIGH);
    delay(300);
    Serial.print("***EarthQuake***");
    delay (1000);
  }

  else {
    digitalWrite(Buzzer1, LOW);
    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print("\n Y: ");
    Serial.print(a.acceleration.y);
    Serial.print("\n Z: ");
    Serial.print(a.acceleration.z);
    Serial.println(" m/s^2");

    Serial.print("Rotation X: ");
    Serial.print(g.gyro.x);
    Serial.print("\n Y: ");
    Serial.print(g.gyro.y);
    Serial.print("\n Z: ");
    Serial.print(g.gyro.z);
    Serial.println(" rad/s");

    Serial.print("\nTemperature: ");
    Serial.print(temp.temperature);
    Serial.println("°C");
  }

  tb.sendTelemetryFloat("BodyTemperature", temp.temperature);
  tb.sendTelemetryFloat("Motion",a.acceleration.x,a.acceleration.y,a.acceleration.z);

  String fireBodytemp = String(temp.temperature) + String("°C");
  Firebase.pushString("/MPU/Temp", fireBodytemp);

  Serial.println("");
  delay(1000);
}
