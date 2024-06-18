//#pragma once
#include <string>
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// MQTT settings
const String ID = "1";
const String BROKER = "192.168.1.102";
const String CLIENT_NAME = "esp32_"+ID;
const String TOPIC = "80";

String in_txt;
bool callback = false;

WiFiClient espClient;
PubSubClient client(espClient); // Setup MQTT client

// ---- MQTT


// Handle incomming messages from the broker
void clientCallback(char* topic, byte* payload, unsigned int length) {
  //String response;

  for (int i = 0; i < length; i++) {
    in_txt += (char)payload[i];
    //response += (char)payload[i];
  }
  //Serial.print("Message arrived\n");
  // Serial.print(TOPIC.c_str());
  // Serial.print("] ");
  // Serial.println(response); 
  //in_txt = response;
  callback = true;

  /*
  // Obtén la longitud de la cadena original
  size_t len = response.length();

  // Reserva memoria para una cadena de caracteres (char[]) con el tamaño adecuado
  char *response_char = (char *)malloc(len + 1);  // +1 para el carácter nulo '\0'

  // Copia la cadena original en la cadena convertida
  response.toCharArray(response_char, len + 1);

  //return response;
  */
}

void reconnectMQTTClient() {
  while (!client.connected()) {
    Serial.println(F("Attempting MQTT connection..."));
    //client.setKeepAlive(7200);
    if (client.connect(CLIENT_NAME.c_str())) {
      Serial.print(F("connected to Broker: "));
      Serial.println(BROKER.c_str());
      // Topic(s) subscription
      client.subscribe(TOPIC.c_str());
      Serial.print(F("Subscribed to Topic: "));
      Serial.println(TOPIC.c_str());
      client.setCallback(clientCallback);
      
    }
    else {
      Serial.print(F("Retying in 5 seconds - failed, rc="));
      Serial.println(client.state());
      delay(5000);
    }
  }
}

void MQTTloop(){
  client.loop();
}

void createMQTTClient() {
  client.setServer(BROKER.c_str(), 1883);
  //client.setCallback(clientCallback);
  reconnectMQTTClient();
}