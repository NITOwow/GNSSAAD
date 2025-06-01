#ifndef GPS_MANAGER_H
#define GPS_MANAGER_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <RTClib.h>
#include "GPSDataTypes.h"
#include "NMEAParser.h"
#include "UBXParser.h"
#include "RTCManager.h"

class GPSManager {
private:
  // Hardware
  SoftwareSerial* gpsSerial;
  RTC_DS3231* rtc;
  
  // Dati e parser
  GPSData gpsData;
  SatellitePool satellitePool;
  NMEAParser* nmeaParser;
  UBXParser* ubxParser;
  RTCManager* rtcManager;
  
  // Buffer e stato
  String nmeaBuffer = "";
  uint8_t ubxBuffer[256];
  int ubxBufferIndex = 0;
  bool ubxReceiving = false;
  
  // Timing
  unsigned long lastDataReceived = 0;
  unsigned long lastStatusUpdate = 0;
  static const unsigned long DATA_TIMEOUT = 5000;      // 5 secondi
  static const unsigned long STATUS_UPDATE_INTERVAL = 1000; // 1 secondo
  
  // Funzioni private
  void processSerialData();
  void processNMEALine(String line);
  void processUBXData();
  void checkDataTimeout();
  void updateConnectionStatus();
  
public:
  // Costruttore
  GPSManager(SoftwareSerial* serial, RTC_DS3231* rtcDevice = nullptr);
  
  // Distruttore
  ~GPSManager();
  
  // Inizializzazione
  bool begin(long baudRate = 38400);
  
  // Funzione principale (da chiamare nel loop)
  void update();
  
  // Accesso ai dati
  const GPSData& getData() const;
  const SatellitePool& getSatellitePool() const;
  
  // Controllo stato
  bool hasValidFix() const;
  bool hasTimeReference() const;
  bool isConnected() const;
  
  // Configurazione
  void enableRTC(bool enable);
  void setDebugMode(bool debug);
  
  // Utilità
  void printStatus();
  void printSatelliteInfo();
  void printRFInfo();
};

#endif

