#ifndef RTC_MANAGER_H
#define RTC_MANAGER_H

#include <Arduino.h>
#include <RTClib.h>
#include "GPSDataTypes.h"

class RTCManager {
private:
  RTC_DS3231* rtc;
  GPSData* gpsData;
  unsigned long lastRTCRead = 0;
  static const unsigned long RTC_READ_INTERVAL = 500;    // 500ms
  static const unsigned long GPS_SYNC_INTERVAL = 600000; // 10 minuti
  
  // Funzioni private
  bool isValidDateTime(const DateTime& dt);
  void setFallbackValues();

public:
  // Costruttore
  RTCManager(RTC_DS3231* rtcDevice, GPSData* data);
  
  // Inizializzazione
  bool begin();
  
  // Aggiornamento dati RTC
  void updateRTCData();
  
  // Sincronizzazione con GPS
  void syncRTCWithGPS();
  
  // Funzioni di utilità
  bool isRTCAvailable() const;
  DateTime getCurrentTime();
  float getTemperature();
  
  // Controllo periodico (da chiamare nel loop principale)
  void performMaintenance();
};

#endif