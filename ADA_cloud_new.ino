//include keyword is used to import libraries

#include <WiFi.h>                  //WiFi library will be able to answer all HTTP request
#include <Adafruit_BMP085.h>       //Adafruit BMP085 is used to connect 
                                   //BMP180 sensor with the Adafruit dashboard.

//Adafruit supports a protocol called MQTT, or 
//message queue telemetry transport, for communication with devices.
//MQTT is a standard messaging protocol for the Internet of Things (IoT)

#include "Adafruit_MQTT.h"         //Adafruit_MQTT header file tells about 
                                   //sending and receiving packets.
#include "Adafruit_MQTT_Client.h"  //Adafruit_MQTT_Client allows you to publish to a feed

/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "DEADLYVIRUS_102_2.4GHz"
#define WLAN_PASS       "sarkar@889293"

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                // use 8883 for SSL (Secure Sockets Layer)
#define AIO_USERNAME    "meenagarai"
#define AIO_KEY         "aio_Opxg35bh7SyA5mfccenG6RtiTcIG"

WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

//Publish - push data from device to server for example Sensor Data
Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");

Adafruit_MQTT_Publish pressure = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressure");

//Subscribe - push data from server to device for example LED Control
Adafruit_MQTT_Subscribe ac = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/ac");

Adafruit_MQTT_Subscribe lamp = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/lamp");

/*************************** Sketch Code ************************************/

void MQTT_connect();    //establish a connection of ESp32 with cloud server

const int led1 = 18;
const int led2 = 19;

float p;
float q;

String stringOne, stringTwo;

Adafruit_BMP085 bmp;

void setup() {
  Serial.begin(9600);    //data exchange speed
  delay(10);

//PinMode() configures the specified pin to behave either as an input or an output.
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);

//digitalWrite() function helps to change the state of LED
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);

//F() tells the compiler to keep the string inside of PROGMEM 
//and not allow it to consume RAM.
  Serial.println(F("Adafruit MQTT demo"));  //print the statement

//connect to wifi access point
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); 
  Serial.println(WiFi.localIP());

// Setup MQTT subscription for ac feed.
  mqtt.subscribe(&ac);

// Setup MQTT subscription for lamp feed.
  mqtt.subscribe(&lamp);

  if (!bmp.begin()) {                //start the process.
    Serial.println("BMP180 Sensor not found ! ! !");
    while (1) {}
  }
}

uint32_t x = 0;     //unsigned uint32_t data type works on 32-bit numbers.

void loop() {
  p = bmp.readPressure();         //read the pressure around us
  q = bmp.readTemperature();      //read the temperature around us
  Serial.println(p);
  Serial.println(q);
  delay(100);

  MQTT_connect();                  //start connecting to the server.

//So after the connection check, the server waits for subscriptions to come in. 
//Now it's time to send data from server to LED’s
  Adafruit_MQTT_Subscribe *subscription;
  
  //wait up to 5 seconds for a subscription message.
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &ac) {           //Compare the latest feed
      stringOne = (char *)ac.lastread;   //read the last message
      Serial.print(F("stringOne: "));
      Serial.println(stringOne);         //prints out the received data

      if (stringOne == "ON") {
        digitalWrite(led1, HIGH);
      }
      if (stringOne == "OFF") {
        digitalWrite(led1, LOW);
      }
    }

    if (subscription == &lamp) {
      stringTwo = (char *)lamp.lastread;
      Serial.print(F("stringTwo: "));
      Serial.println(stringTwo);

      if (stringTwo == "ON") {
        digitalWrite(led2, HIGH);
      }
      if (stringTwo == "OFF") {
        digitalWrite(led2, LOW);
      }
    }
  }
  
  if (! temperature.publish(q)) {
    //Serial.println(F("Temp Failed"));
  } else {
    //Serial.println(F("Temp OK!"));
  }

  if (! pressure.publish(p)) {
    //Serial.println(F("pressure Failed"));
  } else {
    //Serial.println(F("pressure OK!"));
  }
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;
  
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
  
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { 
    // connect will return 0 for connected
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      while (1);
    }
  }

}
