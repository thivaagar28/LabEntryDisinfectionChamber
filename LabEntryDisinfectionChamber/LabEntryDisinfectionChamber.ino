#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include avr power.h
#endif
#define PIN 46 // pin for build in RGB LED on the board
Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIN, NEO_GRB + NEO_KHZ800);

const char* ssid = "";    //add wifi ssid
const char* password = "";    //add wifi password
const char *MQTT_SERVER = ""; // your VM instance public IP address (external ip from GCP)
const int MQTT_PORT = 1883;
const char *MQTT_TOPIC = "iot"; // MQTT topic

//Used Pins
const int motionSensor = 4;     //Left side Maker Port
const int dht11Pin = 42;         //Right side Maker Port
const int button_door= 47;
const int MQ2pin = A2;          //center Maker Port
const int servoPin = 21;
const int led_uv= 14;
const int fan = 38;

//Temperature input sensor
#define DHTTYPE DHT11
DHT dht(dht11Pin, DHTTYPE);
//Wifi Initialization
WiFiClient espClient;
PubSubClient client(espClient);

int status=0;

int angle = 0;
int baselineTemp = 0;

Servo Myservo;

void setup_wifi(){
  delay(10);
  WiFi.begin(ssid, password);
  Serial.println("\nConnecting");

  while(WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(100);
  }
  Serial.println("\nConnected");
  Serial.print("\Maker Feather AIoT S3 IP address:");
  Serial.println(WiFi.localIP());
}

void setup() {
  pinMode(motionSensor, INPUT);
  pinMode(button_door, INPUT);
  pinMode(led_uv, OUTPUT);
  dht.begin();
  Myservo.attach(servoPin);
  pinMode(fan, OUTPUT);
  Myservo.write(0);
  Serial.begin(9600);
  delay(1000);

  strip.begin();
  strip.setBrightness(50);
  strip.show(); // Initialize all pixels to 'off'
  setup_wifi();
  client.setServer(MQTT_SERVER, MQTT_PORT);
}

void reconnect()
{
 while (!client.connected())
 {
 Serial.println("Attempting MQTT connection...");
 if (client.connect("ESP32Client"))
 {
 Serial.println("Connected to MQTT server");
 }
 else
 {
 Serial.print("Failed, rc=");
 Serial.print(client.state());
 Serial.println(" Retrying in 5 seconds...");
 delay(5000);
 }
 }
}

bool ext_factor(float gasSensorVal, int tmpSensorVal){
  bool ret = false;
  baselineTemp = 50;
  if(gasSensorVal >= 2000){
    ret = true;
  }
  //Publish the Gas Sensor Value and Temperature Sensor Value
  char payload[50];
  sprintf(payload, "%.2f, %d", gasSensorVal, tmpSensorVal);
  client.publish(MQTT_TOPIC, payload);
  
  if (tmpSensorVal > baselineTemp) {
    ret = true;
  }
  return ret;
}

void loop() {
  Serial.println("loop starting");
  if (!client.connected()){
    reconnect();
  }
  client.loop();
  delay(5000);
  analogWrite(fan, 255);
  Myservo.write(angle);
  if(!ext_factor(analogRead(MQ2pin), dht.readTemperature())){
    colorWipe(strip.Color(0, 255, 0), 50); // Green
    status = digitalRead(button_door);
    if(status == HIGH){
      Serial.println("Button pressed");
      //open door
      Myservo.write(angle+90);
      delay(5000);
      //close door
      Myservo.write(angle);
      delay(3000);
      //check for motion inside chmaber
      if(digitalRead(motionSensor) == HIGH){
        //uv light on
        digitalWrite(led_uv,1);
        //fan on
        analogWrite(fan, 0);
        delay(7000);
        Serial.println("Disinfection Done");
        //fan off
        analogWrite(fan, 255);
        delay(1000);
        //uv light off
        digitalWrite(led_uv,0);
        delay(3000);
        //open door
        Myservo.write(angle+90);
        delay(5000);
        //close door
        Myservo.write(angle);
      	delay(5000);
      }
    }
  }
  else{
    colorWipe(strip.Color(255, 0, 0), 50); // Red
    Serial.println("SMOKE OR HIGH TEMPERATURE DETECTED");
    delay(3000);
  }
}

void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}
