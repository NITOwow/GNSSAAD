#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

#include <Arduino.h>
#include "GPSDataTypes.h"

class NMEAParser {
private:
  GPSData* gpsData;
  SatellitePool* satellitePool;
  
  // Funzioni di parsing private
  void parseGGA(String sentence);
  void parseGSA(String sentence);
  void parseGSV(String sentence);
  void parseRMC(String sentence);
  void parseVTG(String sentence);
  
  // Funzioni di supporto private
  String getConstellation(String sentence);
  String convertToDMS(String coord, String direction);
  void clearPositionData();
  void clearAllGPSData();
  bool isSatelliteInUse(int prn);
  String getSatelliteSymbol(const SatelliteInfo& sat);
  
  // Gestione satelliti
  void addSatelliteToPool(String constellation, int prn, int elevation, int azimuth, int snr);
  void updateDisplayedSatellites();
  void cleanupOldSatellites();
  void calculateAverageSST();

public:
  // Costruttore
  NMEAParser(GPSData* data, SatellitePool* pool);
  
  // Funzione principale di parsing
  void processNMEA(String nmea);
  
  // Funzioni di manutenzione
  void performMaintenance();
};

#endif