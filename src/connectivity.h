#include "WiFi.h" // ESP32 WiFi include
#include "esp_sntp.h"
#include "WiFiConfig.h" // My WiFi configuration.

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

void ConnectToWiFi()
{
 
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, WiFiPassword);
  Serial.print("Connecting to "); Serial.println(SSID);
 
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print('.');
    delay(500);
 
    if ((++i % 16) == 0)
    {
      Serial.println(F(" still trying to connect"));
    }
  }
 
  Serial.print(F("Connected. My IP address is: "));
  Serial.println(WiFi.localIP());

  delay(1000);

  // Configurar el servicio SNTP
  configTime(0, 0, ntpServer1, ntpServer2, ntpServer3); // -18000 es para UTC -5 (-5*60*60)
  setTimezone("<-05>5");  // Ajusta la hora a UTC-5 Hora en Bogotá https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv

  printTime();
  //delay(1000); 
}

struct tm get_current_time() {

    struct tm time;
   
    if(!getLocalTime(&time)){
        Serial.println("Could not obtain time info");
        return time;
    }

    return time;
}