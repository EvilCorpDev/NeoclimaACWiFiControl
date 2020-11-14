#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>
#include <Esp.h>

#define RXD2 15 //RX1 pin ac orange
#define TXD2 4 //TX1 pin ac brown
#define RXD1 16 //RXX2 pin stick brown cable
#define TXD1 17 //TX2 pin orange cable

#define BAUD_RATE 9600
#define DELIMITER ' '

#define MQTT_CLIENT_ID "esp32_sniffer"
#define MQTT_TOPIC_COMMAND_RESULT "myhome/Conditioner/command/result"
#define MQTT_TOPIC_COMMAND_SEND "myhome/Conditioner/command/send"
#define MQTT_TOPIC_CONNECTION_IP "myhome/Conditioner/connection/ip"
#define MQTT_TOPIC_LISTENER_SERIAL_1 "myhome/Conditioner/listener/serial1"
#define MQTT_TOPIC_LISTENER_SERIAL_2 "myhome/Conditioner/listener/serial2"

#define MQTT_USER "user"
#define MQTT_PASSWORD "pass"

WiFiClient espClient;
PubSubClient mqttClient(espClient);

const char* mqtt_server = "192.168.1.128";

const char* ssid = "WiFi ssid";
const char* password = "wifi pass";

std::vector<int> parseCommandPayload(uint8_t* payloadBytes, unsigned int length) {
    payloadBytes[length] = '\0';
    String payload = String((char*)payloadBytes);

    std::vector<int> bytesToSend;
    std::vector<char> buffer;

    for (int i = 0; i < length; ++i) {
        char value = payload[i];

        if(value != DELIMITER) {
            buffer.push_back(value);
        }

        if(value == DELIMITER || i == length - 1) {
            buffer.push_back('\0');
            int commandByte = String(buffer.data()).toInt();
            bytesToSend.push_back(commandByte);
            buffer.clear();
        }
    }

    return bytesToSend;
}

void mqttCallback(char* topic, uint8_t* payloadBytes, unsigned int length) {
    for(int data: parseCommandPayload(payloadBytes, length)) {
        Serial2.write(data);
        Serial.print(data);
        Serial.print(' ');
    }

    mqttClient.publish(MQTT_TOPIC_COMMAND_RESULT, String(millis()).c_str());
}

void setup() {
    Serial.begin(BAUD_RATE);
    Serial1.begin(BAUD_RATE, SERIAL_8N1, RXD1, TXD1);
    Serial2.begin(BAUD_RATE, SERIAL_8N1, RXD2, TXD2);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("Connection Failed! Rebooting...");
        delay(30000);
        ESP.restart();
    }

    ArduinoOTA.begin();

    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    mqttClient.setServer(mqtt_server, 1883);
    mqttClient.setCallback(mqttCallback);
}

void reconnect() {
    while (!mqttClient.connected()) {
        if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
            mqttClient.publish(MQTT_TOPIC_CONNECTION_IP, WiFi.localIP().toString().c_str());
            mqttClient.publish(MQTT_TOPIC_COMMAND_SEND, "Put here your command");
            mqttClient.publish(MQTT_TOPIC_COMMAND_RESULT, "Await command result here");
            mqttClient.subscribe(MQTT_TOPIC_COMMAND_SEND);
        } else {
            delay(500);
        }
    }
}

void listenSerial(HardwareSerial& serial, const char* topic) {
    if(serial.available()) {
        int byteArray = serial.read();
        mqttClient.publish(topic, String(byteArray, DEC).c_str());
    }
}

void loop() {
    ArduinoOTA.handle();

    if(!mqttClient.connected()) {
        reconnect();
    }

    listenSerial(Serial1, MQTT_TOPIC_LISTENER_SERIAL_1);
    listenSerial(Serial2, MQTT_TOPIC_LISTENER_SERIAL_2);

    mqttClient.loop();
}