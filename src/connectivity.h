#include "WiFi.h" // ESP32 WiFi include
#include "esp_sntp.h"
#include "WiFiConfig.h" // My WiFi configuration.
#include "mqtt.h"
//#include "esp_wifi.h"

IPAddress ip(192, 168, 1, 200);
IPAddress mask(255, 255, 255, 0);


void ledBlink(int repe) {

  for (int i = 0; i < repe; i++) {
    digitalWrite(ledPin, LOW);
    delay(250);
    digitalWrite(ledPin, HIGH);
    delay(250);
    
  }
}


void printTime(){
  
  struct tm time;
   
  if(!getLocalTime(&time)){
    Serial.println("Could not obtain time info");
    return;
  }
 
  Serial.println("\n---------TIME----------");
  Serial.println(&time, "%A, %B %d %Y %H:%M:%S");
  Serial.println("");
   
//   Serial.print("Number of years since 1900: ");
//   Serial.println(time.tm_year);
 
//   Serial.print("month, from 0 to 11: ");
//   Serial.println(time.tm_mon);
 
//   Serial.print("day, from 1 to 31: "); 
//   Serial.println(time.tm_mday);
 
//   Serial.print("hour, from 0 to 23: ");
//   Serial.println(time.tm_hour);
 
//   Serial.print("minute, from 0 to 59: ");
//   Serial.println(time.tm_min);
   
//   Serial.print("second, from 0 to 59: ");
//   Serial.println(time.tm_sec);

}

void setTimezone(String timezone){
  setenv("TZ",timezone.c_str(),1);  //  Now adjust the TZ.  Clock settings are adjusted to show the new local time
  tzset();
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.print("************ Free Memory (disconnected): ");
  Serial.println(esp_get_free_heap_size());
  //Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.wifi_sta_disconnected.reason);
  //Serial.println("Trying to Reconnect");
  WiFi.disconnect(true);
  //WiFi.config(ip, INADDR_NONE, mask);
  WiFi.begin(SSID, WiFiPassword);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED){
        Serial.print('*');
        delay(500);

        if ((++i % 16) == 0)
        {
          //Serial.println(F(" still trying to connect"));
          WiFi.disconnect(true);
          WiFi.begin(SSID, WiFiPassword);
        }
  }
  createMQTTClient();
}

void ConnectToWiFi()
{
  WiFi.mode(WIFI_STA);
  //WiFi.config(ip, INADDR_NONE, mask);
  WiFi.begin(SSID, WiFiPassword);
  Serial.print(F("Connecting to ")); Serial.println(SSID);
 
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print('.');
    delay(500);
 
    if ((++i % 16) == 0)
    {
      Serial.println(F(" still trying to connect"));
      WiFi.disconnect(true);
      WiFi.begin(SSID, WiFiPassword);
    }
  }
 
  Serial.print(F("Connected. My IP address is: "));
  Serial.println(WiFi.localIP());
  //WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  ledBlink(3);

  delay(1000);

  // Configurar el servicio SNTP
  configTime(0, 0, ntpServer1, ntpServer2, ntpServer3); // -18000 es para UTC -5 (-5*60*60)
  setTimezone("<-05>5");  // Ajusta la hora a UTC-5 Hora en Bogotá https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv

  printTime();
  //delay(1000); 
}

void WiFiReconnect(){
  WiFi.disconnect();
  WiFi.reconnect();
}

struct tm get_current_time() {

    struct tm time;
   
    if(!getLocalTime(&time)){
        Serial.println("Could not obtain time info");
        return time;
    }

    return time;
}