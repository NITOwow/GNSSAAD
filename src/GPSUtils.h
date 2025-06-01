#ifndef GPS_UTILS_H
#define GPS_UTILS_H

#include <Arduino.h>
#include "GPSDataTypes.h"

class GPSUtils {
public:
  // Conversioni coordinate
  static String convertToDMS(String coord, String direction);
  static String convertToDecimal(String dmsCoord);
  static double dmsToDecimal(String dms);
  
  // Calcoli geografici
  static double calculateDistance(double lat1, double lon1, double lat2, double lon2);
  static double calculateBearing(double lat1, double lon1, double lat2, double lon2);
  
  // Validazione dati
  static bool isValidLatitude(String lat);
  static bool isValidLongitude(String lon);
  static bool isValidNMEAChecksum(String sentence);
  
  // Formattazione
  static String formatTime(String utcTime, int timezoneOffset = 1);
  static String formatDate(String dateStr);
  static String formatCoordinate(String coord, int precision = 4);
  
  // Analisi qualità segnale
  static String getSignalQualityDescription(float snr);
  static String getFixQualityDescription(int quality);
  static String getConstellationName(String prefix);
  
  // Utilità satelliti
  static String getSatelliteSymbol(const SatelliteInfo& sat, bool inUse);
  static int countVisibleSatellites(const SatelliteInfo satellites[], int count);
  static int countSatellitesInUse(const SatelliteInfo satellites[], int count);
  static float calculateAverageSNR(const SatelliteInfo satellites[], int count);
  
  // Validazione NMEA
  static uint8_t calculateNMEAChecksum(String sentence);
  static bool validateNMEAChecksum(String sentence);
  
  // Parsing campi NMEA
  static String extractField(String sentence, int fieldIndex);
  static int countFields(String sentence);
  
  // Conversioni tempo
  static DateTime parseNMEADateTime(String timeStr, String dateStr);
  static String formatDateTime(const DateTime& dt, bool includeDate = true);
  
  // Debug e diagnostica
  static void printGPSStatus(const GPSData& data);
  static void printSatelliteTable(const SatelliteInfo satellites[], int count);
  static void printRFStatus(const RFBlock blocks[], int count);
  
  // Costanti utili
  static const double EARTH_RADIUS_KM;
  static const double KNOTS_TO_KMH;
  static const double DEGREES_TO_RADIANS;
  static const double RADIANS_TO_DEGREES;
};

#endif