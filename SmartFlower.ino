//
// MQTT / WIFI
//

#include <PubSubClient.h>
#include <WiFi.h>

const char* ssid = "";
const char* password = "";
const char* mqtt_server = "";
const char *mqtt_username = "";
const char *mqtt_password = "";

WiFiClient espClient;
PubSubClient client(espClient);

#define TEMPERATURE_TOPIC ""
#define HUMIDITY_TOPIC ""
#define SOIL_TOPIC ""

long lastMsg = 0;
char msg[20];

//
// RGB LED
//

#include "Freenove_WS2812_Lib_for_ESP32.h"

#define LEDS_COUNT  1
#define LEDS_PIN	8
#define CHANNEL		0

Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, CHANNEL, TYPE_GRB);

//
//  DHT11
//

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 2     // Digital pin connected to the DHT sensor 
#define ANALOGPIN 3
#define DHTTYPE    DHT11     // DHT 11

DHT_Unified dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  dht.begin();
  // put your setup code here, to run once:
  wifiSetup();
  /* configure the MQTT server with IPaddress and port */
  client.setServer(mqtt_server, 1883);
  /* this receivedCallback function will be invoked
  when client received subscribed topic */
  client.setCallback(receivedCallback);
 	strip.begin();
	strip.setBrightness(255);	
}

bool ledFlag = false;
int temperature, humidity, soil;
uint32_t getDataTimer;
uint32_t restartDhtTimer;
void loop() 
{
  mqttLogic();

  if (millis() - getDataTimer >= 3000)
  {
    getDataTimer = millis();
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (!isnan(event.temperature)) 
    {
      temperature = event.temperature;
      Serial.print("TEMP: ");
      Serial.print(temperature);
    }
    else
    {
      Serial.print("TEMP: ");
      Serial.print("???");
    }

    // Get humidity event and print its value.
    dht.humidity().getEvent(&event);
    if (!isnan(event.relative_humidity)) 
    {
      humidity = event.relative_humidity;
      Serial.print(" HUM: ");
      Serial.print(humidity);
    }
    else
    {
      Serial.print(" HUM: ");
      Serial.print("???");
    }

    float value = analogRead(ANALOGPIN);
    soil = 100 - (value / 4095 * 100);

    if (soil <= 10)
    {
      if (ledFlag)
      {
        strip.setLedColorData(0, 255, 0, 0);
        strip.show();
      }
      else
      {
        strip.setLedColorData(0, 0, 0, 0);
        strip.show();
      }
      ledFlag = !ledFlag;
    }
    else
    {
      strip.setLedColorData(0, 0, 0, 0);
			strip.show();
    }

    Serial.print(" SOIL: ");
    Serial.print(soil);
    Serial.print(" VALUE: ");
    Serial.println(value);
  }
}


void mqttconnect() {

  /* Loop until reconnected */
  while (!client.connected()) {
    //Serial.print("MQTT connecting ...");
    /* client ID */
    String client_id = "ESP32";
    /* connect now */
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      //Serial.println("connected");
      /* subscribe topic with default QoS 0*/
      // client.subscribe(TEMP_TOPIC);
    } else {
      Serial.print("failed, status code =");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");
      /* Wait 5 seconds before retrying */
      delay(5000);
    }
  }
}

void receivedCallback(char* topic, byte* payload, unsigned int length) {
  //Serial.print("Message received: ");
  //Serial.println(topic);

 // Serial.print("payload: ");
  for (int i = 0; i < length; i++) {
    //Serial.print((char)payload[i]);
  }
  //Serial.println();
}

void wifiSetup()
{
  WiFi.begin(ssid, password);
  Serial.println("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    strip.setLedColorData(0, 255, 255, 255);
    strip.show();
    ledFlag = !ledFlag;
  }
  for (int i = 0; i < 10; i++)
  {
    strip.setLedColorData(0, 255, 255, 255);
    strip.show();
    delay(50);
    strip.setLedColorData(0, 0, 0, 0);
    strip.show();
    delay(50);
  }
  strip.setLedColorData(0, 0, 0, 0);
  strip.show();
  ledFlag = false;
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void mqttLogic()
{
  /* if client was disconnected then try to reconnect again */
  if (!client.connected()) {
    mqttconnect();
  }
  /* this function will listen for incomming
  subscribed topic-process-invoke receivedCallback */
  client.loop();
  /* we measure temperature every 3 secs
  we count until 3 secs reached to avoid blocking program if using delay()*/
  long now = millis();
  if (now - lastMsg > 3000) {
    lastMsg = now;
    if (!isnan(temperature))
    {
      snprintf(msg, 5, "%i", temperature);
      client.publish(TEMPERATURE_TOPIC, msg);
    }

    if (!isnan(humidity))
    {
      snprintf(msg, 5, "%i", humidity);
      client.publish(HUMIDITY_TOPIC, msg);
    }

    if (!isnan(soil))
    {
      snprintf(msg, 5, "%i", soil);
      client.publish(SOIL_TOPIC, msg);
    }
  }
}

