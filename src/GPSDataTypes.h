#ifndef GPS_DATA_TYPES_H
#define GPS_DATA_TYPES_H

#include <Arduino.h>
#include <RTClib.h>

// Struttura per informazioni satellite singolo
struct SatelliteInfo {
  String constellation = "";
  int prn = 0;
  int elevation = 0;
  int azimuth = 0;
  int snr = 0;
  bool inUse = false;
};

// Struttura per pool satelliti (gestione completa)
struct SatellitePool {
  SatelliteInfo satellites[32];
  int count = 0;
  unsigned long lastUpdate = 0;
};

// Struttura per blocchi RF (UBX-MON-RF)
struct RFBlock {
  uint8_t blockId = 0;
  uint8_t jammingState = 0;
  uint8_t antStatus = 0;
  uint8_t antPower = 0;
  uint16_t noisePerMS = 0;
  uint16_t agcCnt = 0;
  uint8_t cwSuppression = 0;
  int8_t ofsI = 0;
  uint8_t magI = 0;
  int8_t ofsQ = 0;
  uint8_t magQ = 0;
  uint8_t rfBlockGnssBand = 0;
};

// Struttura dati GPS principale
struct GPSData {
  // Fix e stato
  String fixStatus = "NO CARRIER";
  String listenStatus = "OFFLINE";
  bool hasValidFix = false;
  bool hasTimeOnly = false;
  
  // Posizione
  String latitude = "";
  String longitude = "";
  float altitude = 0.0;
  float geoidHeight = 0.0;
  
  // Tempo
  String utcTime = "";
  String localTime = "";
  String date = "";
  String timeAndDateFix = "";
  
  // Validità e precisione
  String validity = "V";
  String warning = "V";
  int satellites = 0;
  float precision = 0.0;
  float hdop = 0.0;
  
  // Velocità e direzione
  float speed = 0.0;
  float bearing = 0.0;
  float magneticVariation = 0.0;
  
  // Satelliti (4 visualizzati + SBAS)
  SatelliteInfo satellites_info[4];
  SatelliteInfo sbasSat;
  float signalStrength = 0.0;
  
  // RF Data (UBX)
  bool rfDataAvailable = false;
  uint8_t numRFBlocks = 0;
  RFBlock rfBlocks[2];
  
  // RTC Data
  bool rtcAvailable = false;
  String rtcDate = "";
  String rtcTime = "";
  float rtcTemperature = 298.15;
  
  // GPS Time Sync
  bool hasValidGPSTime = false;
  DateTime lastValidGPSTime;
  unsigned long lastGPSSync = 0;
  unsigned long lastRTCUpdate = 0;
};

// Variabili globali per satelliti
extern int satellitesInUse[20];
extern int satellitesInUseCount;
extern int displayedSatelliteIndices[4];
extern unsigned long lastSatelliteRotation;
extern const unsigned long satelliteRotationInterval;

#endif