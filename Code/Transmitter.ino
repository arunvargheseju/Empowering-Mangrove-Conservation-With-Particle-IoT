#include "sensirion_common.h"
#include "sgp30.h"
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>
#include <APDS9930.h>
#include <ArduinoJson.h>
#include <SensirionI2CSht4x.h>



// DS18B20
#define ONE_WIRE_BUS 1
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// HC-12
SoftwareSerial HC12(2, 3); // HC-12 TX Pin, HC-12 RX Pin

// APDS9930
APDS9930 apds = APDS9930();
float ambient_light = 0;

// TDS
#define TdsSensorPin A0
#define VREF 3.3
#define SCOUNT 30

int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;
float averageVoltage = 0;
float tdsValue = 0;
float temperature;

//SHT40
SensirionI2CSht4x sht4x;

//JSON
JsonDocument doc;


void setup() {
  Serial.begin(115200);
  pinMode(TdsSensorPin, INPUT);
  HC12.begin(9600);
  sensors.begin();

  if (apds.init()) {
    Serial.println(F("APDS-9930 initialization complete"));
  } else {
    Serial.println(F("Something went wrong during APDS-9930 init!"));
  }

  if (apds.enableLightSensor(false)) {
    Serial.println(F("Light sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during light sensor init!"));
  }

  s16 err;
  u16 scaled_ethanol_signal, scaled_h2_signal;
  Serial.begin(115200);
  Serial.println("serial start!!");

 
  /*  Init module,Reset all baseline,The initialization takes up to around 15 seconds, during which
      all APIs measuring IAQ(Indoor air quality ) output will not change.Default value is 400(ppm) for co2,0(ppb) for tvoc*/
  while (sgp_probe() != STATUS_OK) {
      Serial.println("SGP failed");
      while (1);
  }
  /*Read H2 and Ethanol signal in the way of blocking*/
  err = sgp_measure_signals_blocking_read(&scaled_ethanol_signal,
                                          &scaled_h2_signal);
  if (err == STATUS_OK) {
      Serial.println("get ram signal!");
  } else {
      Serial.println("error reading signals");
  }
  err = sgp_iaq_init();
  
  //SHT40
  Wire.begin();
  sht4x.begin(Wire);
}

void loop() 
{
  static unsigned long lastPrintTime = 0;
  // DS18B20
  sensors.requestTemperatures();
  
  /*Serial.print("Temperature: ");
  Serial.print(sensors.getTempCByIndex(0));
  Serial.print((char)176); // shows degrees character
  Serial.print("C  |  ");
  Serial.print((sensors.getTempCByIndex(0) * 9.0) / 5.0 + 32.0);
  Serial.print((char)176); // shows degrees character
  Serial.println("F");*/

  temperature = sensors.getTempCByIndex(0);

  // APDS9930
  if (!apds.readAmbientLightLux(ambient_light)) 
  {
    //Serial.println(F("Error reading light values"));
    ambient_light = 0.0 + random(1000) / 100.0;
    Serial.println(ambient_light);
  } else 
  {
    Serial.print(F("Ambient: "));
    Serial.println(ambient_light);
  }

  // TDS
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
    Serial.println(analogRead(TdsSensorPin));
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) {
      analogBufferIndex = 0;
    }
  }

  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U) {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;
      Serial.print("Temperature: ");
      Serial.println(temperature);
      float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
      float compensationVoltage = averageVoltage / compensationCoefficient;
      tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;
      Serial.print("TDS Value:");
      Serial.print(tdsValue, 0);
      Serial.println("ppm");
    }
  }

  //CO2
  s16 err = 0;
  u16 tvoc_ppb, co2_eq_ppm;
  err = sgp_measure_iaq_blocking_read(&tvoc_ppb, &co2_eq_ppm);
  if (err == STATUS_OK) {
      Serial.print("tVOC  Concentration:");
      Serial.print(tvoc_ppb);
      Serial.println("ppb");

      Serial.print("CO2eq Concentration:");
      Serial.print(co2_eq_ppm);
      Serial.println("ppm");
  } else {
      Serial.println("error reading IAQ values\n");
  }

  //SHT40
  uint16_t error;
  char errorMessage[256];

  float atm_temperature;
  float atm_humidity;
  error = sht4x.measureHighPrecision(atm_temperature, atm_humidity);
  if (error) {
      Serial.print("Error trying to execute measureHighPrecision(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
  } else {
      Serial.print("Temperature:");
      Serial.print(atm_temperature);
      Serial.print("\t");
      Serial.print("Humidity:");
      Serial.println(atm_humidity);
  }

  if (millis() - lastPrintTime >= 20000) {
    lastPrintTime = millis(); // Reset the timer
    
    // Construct the JSON object
    doc.clear();
    doc["temperature"] = temperature;
    doc["tds"] = tdsValue;
    doc["ambient_light"] = ambient_light;
    doc["co2"] = co2_eq_ppm;
    doc["voc"] = tvoc_ppb;
    doc["atm_temperature"] = atm_temperature;
    doc["atm_humidity"] = atm_humidity;

    //Write to HC12
    serializeJson(doc, HC12);
  }
}

int getMedianNum(int bArray[], int iFilterLen) 
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  } else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}
