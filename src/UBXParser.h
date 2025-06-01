#ifndef UBX_PARSER_H
#define UBX_PARSER_H

#include <Arduino.h>
#include "GPSDataTypes.h"

class UBXParser {
private:
  GPSData* gpsData;
  
  // Funzioni di parsing private
  void parseMonRFVersion0(uint8_t* payload, uint16_t length, uint8_t nBlocks);
  void parseMonRFVersion1(uint8_t* payload, uint16_t length, uint8_t nBlocks);
  uint8_t estimateJammingState(uint8_t cwSuppression);

public:
  // Costruttore
  UBXParser(GPSData* data);
  
  // Funzioni di parsing UBX
  void parseUBXMonRF(uint8_t* payload, uint16_t length);
  
  // Funzioni di utilità
  bool validateUBXChecksum(uint8_t* data, uint16_t length);
  uint16_t calculateUBXChecksum(uint8_t* data, uint16_t length);
};

#endif