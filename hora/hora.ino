
#include "RTClib1.h"
//librerias complementarias
#include <Wire.h>
#include <SPI.h>

RTC_DS1307 rtc;
int sec_pasados = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire1.begin(); // Inicia el puerto I2C
  rtc.begin();
  rtc.adjust(DateTime(2016,05,03,10,58,05));
  DateTime pasa = rtc.now();
  sec_pasados = pasa.unixtime();

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(rtc.now().year());
  Serial.print("/");
  Serial.print(rtc.now().month());
  Serial.print("/");
  Serial.print(rtc.now().day());
  Serial.print("........");
  Serial.print(rtc.now().hour());
  Serial.print(":");
  Serial.print(rtc.now().minute());
  Serial.print(":");
  Serial.println(rtc.now().second());


  
}
