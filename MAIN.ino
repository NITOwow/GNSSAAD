#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <HardwareSerial.h>
#include <TimeLib.h>
#include <RTClib.h>

// Configurazione hardware
#define BUTTON_PIN 2
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define LCD_ADDRESS 0x27
#define LCD_COLS 20
#define LCD_ROWS 4

// Configurazione hardware - AGGIUNGERE DOPO LE DEFINIZIONI ESISTENTI
#define BUTTON_PIN 2
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define LCD_ADDRESS 0x27
#define LCD_COLS 20
#define LCD_ROWS 4

// Definizioni PIN LED
#define LED_ACT_PIN 4      // Lampeggia 5 volte al secondo quando sistema in funzione
#define LED_CLOCK_PIN 5    // Lampeggia ogni secondo dell'RTC
#define LED_PPS_PIN 18     // Lampeggia ogni 5 sec dopo fix GPS
#define LED_INTRFC_PIN 19  // Lampeggia 5 volte ogni 10 sec
#define LED_FAULT_PIN 12   // Acceso fisso se BITE ha rilevato fault

struct GPSData {
  struct Satellite {
    String constellation;
    int prn;
    int elevation;
    int azimuth;
    int snr;
    bool inUse;
    
    
    Satellite() : constellation(""), prn(0), elevation(0), azimuth(0), snr(0), inUse(false) {}
  };
  
  struct RFBlock {
    uint8_t blockId;
    uint8_t jammingState;
    uint8_t antStatus;
    uint8_t antPower;
    uint16_t noisePerMS;
    uint16_t agcCnt;
    uint8_t cwSuppression;
    int8_t ofsI;
    uint8_t magI;
    int8_t ofsQ;
    uint8_t magQ;
    uint8_t rfBlockGnssBand;
  };
  
  // Dati GPS principali
  String errorRange;
  String fixStatus;
  String listenStatus;
  bool hasValidFix;
  int satellites;
  float precision;
  float altitude;
  float geoidHeight;
  String latitude;
  String longitude;
  String validity;
  String warning;
  String date;
  String utcTime;
  String localTime;
  String timeAndDateFix;
  float speed;
  float bearing;
  float hdop;
  float magneticVariation;
  float signalStrength;
  bool hasTimeOnly;  
  
  // Array di satelliti per display
  Satellite satellites_info[4];
  Satellite sbasSat;
  
  // Dati RF
  bool rfDataAvailable;
  int numRFBlocks;
  RFBlock rfBlocks[2];
  
  // Dati RTC
  bool rtcAvailable;
  String rtcDate;
  String rtcTime;
  float rtcTemperature;
  DateTime lastValidGPSTime;
  bool hasValidGPSTime;
  unsigned long lastGPSSync;
  unsigned long lastRTCUpdate;
  
  GPSData() : fixStatus("NO CARRIER"), listenStatus("OFFLINE"), hasValidFix(false),
              satellites(0), precision(0), altitude(0), geoidHeight(0),
              latitude(""), longitude(""), validity("V"), warning("V"),
              date(""), utcTime(""), localTime(""), timeAndDateFix(""),
              speed(0), bearing(0), hdop(0), magneticVariation(0), signalStrength(0),
              rfDataAvailable(false), numRFBlocks(0), rtcAvailable(false),
              rtcDate(""), rtcTime(""), rtcTemperature(298.15),
              hasValidGPSTime(false), lastGPSSync(0), lastRTCUpdate(0) {}
};

// Variabili globali per la gestione dei satelliti - da aggiungere dopo le strutture

// Array per tracciare i satelliti in uso (da GSA)
int satellitesInUse[20];
int satellitesInUseCount = 0;

// Gestione rotazione display satelliti
int displayedSatelliteIndices[4] = {0, 1, 2, 3};
unsigned long lastSatelliteRotation = 0;
const unsigned long satelliteRotationInterval = 5000; // 5 secondi


// Variabili globali
GPSData gpsData;


// AGGIUNGERE QUESTE VARIABILI DOPO LE VARIABILI GLOBALI ESISTENTI
// Variabili per il controllo dei LED
unsigned long lastActBlink = 0;
const unsigned long actBlinkInterval = 100;  // 100ms = 10 volte al secondo (5 ON + 5 OFF)
bool actLedState = false;

unsigned long lastClockBlink = 0;
const unsigned long clockBlinkInterval = 1000; // 1 secondo
bool clockLedState = false;

unsigned long lastPpsBlink = 0;
const unsigned long ppsBlinkInterval = 5000; // 5 secondi
bool ppsLedState = false;
bool ppsEnabled = false; // Abilitato solo dopo fix GPS

unsigned long lastIntrfcBlink = 0;
const unsigned long intrfcBlinkInterval = 2000; // 2 secondi (5 volte in 10 sec)
bool intrfcLedState = false;
int intrfcBlinkCount = 0;
unsigned long intrfcCycleStart = 0;

bool faultLedState = false; // Stato del LED FAULT

// Inizializzazione componenti
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);
HardwareSerial gpsSerial(2);
RTC_DS3231 rtc;

// Variabili globali
int currentPage = 1;
const int totalPages = 11;
bool buttonPressed = false;
unsigned long lastButtonPress = 0;
const unsigned long debounceTime = 300;
unsigned long lastDisplayUpdate = 0;
const unsigned long displayUpdateInterval = 1000;

// Variabili globali per il PRN
String currentPRN = "";
unsigned long lastPrnGeneration = 0;
int prnScrollPosition = 0;
unsigned long lastPrnScroll = 0;


// Aggiungere questa variabile globale insieme alle altre variabili globali:
unsigned long lastCursorBlink = 0;
const unsigned long cursorBlinkInterval = 500;
bool cursorVisible = true;

// Variabili per BITE test
bool biteTestCompleted = false;
bool rfCoreOK = false;
bool rtcOK = false;
bool ramOK = false;
unsigned long biteStartTime = 0;
int biteStep = 0;

// CORREZIONE 2: Aggiungere queste variabili globali per la gestione satelliti
// (Aggiungere dopo le altre variabili globali esistenti)
struct SatellitePool {
  GPSData::Satellite satellites[32]; // Pool più grande per tutti i satelliti
  int count = 0;
  unsigned long lastUpdate = 0;
} satellitePool;






// Aggiungere dopo le altre variabili globali:
// Struttura per i dati UBX-NAV-SAT
struct UBXNavSatData {
  struct Satellite {
    uint8_t gnssId = 0;
    uint8_t svId = 0;
    uint8_t cno = 0;
    int8_t elev = 0;
    int16_t azim = 0;
    uint8_t qualityInd = 0;
    uint8_t health = 0;
    uint8_t orbitSource = 0;
    bool svUsed = false;
  } satellites[12]; // Massimo 12 satelliti per pagina
  uint8_t numSvs = 0;
  uint32_t iTOW = 0;
  bool dataAvailable = false;
} ubxNavSatData;

int currentSatPage = 0; // Per navigare tra le pagine di satelliti
unsigned long lastUBXRequest = 0;





String nmeaBuffer = "";
bool ubxRequestSent = false;

void setup() {

  gpsData.hasTimeOnly = false;
  Serial.begin(115200);
  gpsSerial.begin(38400, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  
  // Inizializzazione LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("NAVCORE V5 StrServal");
  lcd.setCursor(0, 1);
  lcd.print("ArgonAeroDynamics #");
  lcd.setCursor(0, 2);
  lcd.print("====================");
  lcd.setCursor(0, 3);
  lcd.print("$INIT/BITE EXEC");
  
  // Configurazione pulsante
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Configurazione PIN LED
  pinMode(LED_ACT_PIN, OUTPUT);
  pinMode(LED_CLOCK_PIN, OUTPUT);
  pinMode(LED_PPS_PIN, OUTPUT);
  pinMode(LED_INTRFC_PIN, OUTPUT);
  pinMode(LED_FAULT_PIN, OUTPUT);
  
  // Spegni tutti i LED all'avvio
  digitalWrite(LED_ACT_PIN, LOW);
  digitalWrite(LED_CLOCK_PIN, LOW);
  digitalWrite(LED_PPS_PIN, LOW);
  digitalWrite(LED_INTRFC_PIN, LOW);
  digitalWrite(LED_FAULT_PIN, LOW);

  // Inizializzazione RTC
  if (!rtc.begin()) {
    Serial.println("RTC non trovato!");
    gpsData.rtcAvailable = false;
    // Imposta valori di default se RTC non trovato
    gpsData.rtcDate = "01/01/2024";
    gpsData.rtcTime = "00:00:00";
    gpsData.rtcTemperature = 298.15; // 25°C in Kelvin
  } else {
    Serial.println("RTC DS3231 init");
    gpsData.rtcAvailable = true;
    
    // Se RTC perso l'ora, impostala con un valore di default
    if (rtc.lostPower()) {
      Serial.println("RTC DATA LOSS! using backup values");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    
    // Inizializza i dati RTC
    updateRTCData();
  }

  initializeGPSData();

  delay(2000);
  lcd.clear();
  
  Serial.println("PSTRS RDY");

  // Abilita UBX-NAV-SAT all'avvio
  enableUBXNavSat();
  delay(100);

  // Abilita UBX-MON-RF all'avvio
  enableUBXMonRF();
  delay(500);
    
  delay(3000);
  
  // Avvia BITE test
  biteStartTime = millis();
  biteStep = 0;
  biteTestCompleted = false;
  
  Serial.println("NAVCORE MODULE V1");
  Serial.println("Built In Test in pProgress...");
}
// AGGIUNGERE questa funzione per permettere la ri-verifica dell'RTC:

// Also add this function to initialize the warning field properly
void initializeGPSData() {
  // Initialize all GPS data fields to default values
  gpsData.fixStatus = "NO CARRIER";
  gpsData.satellites = 0;
  gpsData.signalStrength = 0.0;
  gpsData.precision = 0.0;
  gpsData.bearing = 0.0;
  gpsData.altitude = 0.0;
  gpsData.speed = 0.0;
  gpsData.geoidHeight = 0.0;
  gpsData.latitude = "";
  gpsData.longitude = "";
  gpsData.date = "";
  gpsData.utcTime = "";
  gpsData.localTime = "";
  gpsData.timeAndDateFix = "";
  gpsData.magneticVariation = 0.0;
  gpsData.validity = "V";  // Invalid by default
  gpsData.warning = "V";   // Warning by default
  gpsData.listenStatus = "OFFLINE";
  gpsData.hasValidFix = false;
  gpsData.rtcAvailable = false;
  gpsData.hdop = 99.99;
  gpsData.hasValidGPSTime = false;
  gpsData.rfDataAvailable = false;
  gpsData.numRFBlocks = 0;
}


void recheckRTC() {
  bool previousRtcOK = rtcOK;
  rtcOK = rtc.begin();
  
  if (rtcOK != previousRtcOK) {
    if (rtcOK) {
      Serial.println("# RTC now OK - fault cleared");
      gpsData.rtcAvailable = true;
      updateRTCData();
    } else {
      Serial.println("# RTC fault detected");
      gpsData.rtcAvailable = false;
    }
  }
}

// MODIFICARE il loop() principale aggiungendo la ri-verifica periodica dell'RTC:

void loop() {
  // Se BITE test non completato, eseguilo
  if (!biteTestCompleted) {
    performBITETest();
    return;
  }
  
  // Controllo LED (solo se BITE completato)
  updateLEDs();
  
  // Ri-verifica periodica dell'RTC ogni 30 secondi
  static unsigned long lastRTCCheck = 0;
  if (millis() - lastRTCCheck > 30000) {
    recheckRTC();
    lastRTCCheck = millis();
  }

  // Gestione pulsante
  handleButton();
  
  // Lettura dati GPS con output seriale ordinato
  if (gpsSerial.available()) {
    String nmea = gpsSerial.readStringUntil('\n');
    nmea.trim();
    
    // Invia NMEA su Serial Monitor solo se è una frase valida
    if (nmea.startsWith("$") && nmea.length() > 10) {
      Serial.println(nmea);
    }
    
    // Processa NMEA
    processNMEA(nmea);
  }
  
  // Lettura messaggi UBX
  static uint8_t ubxBuffer[512];
  static int ubxIndex = 0;
  static bool inUBXMessage = false;
  static uint16_t expectedLength = 0;
  static uint8_t ubxClass = 0, ubxId = 0;
  
  while (gpsSerial.available()) {
    uint8_t byte = gpsSerial.read();
    
    if (!inUBXMessage) {
      if (ubxIndex == 0 && byte == 0xB5) {
        ubxBuffer[ubxIndex++] = byte;
      } else if (ubxIndex == 1 && byte == 0x62) {
        ubxBuffer[ubxIndex++] = byte;
        inUBXMessage = true;
      } else {
        ubxIndex = 0;
      }
    } else {
      ubxBuffer[ubxIndex++] = byte;
      
      if (ubxIndex == 4) {
        ubxClass = ubxBuffer[2];
        ubxId = ubxBuffer[3];
      } else if (ubxIndex == 6) {
        expectedLength = ubxBuffer[4] | (ubxBuffer[5] << 8);
      } else if (ubxIndex >= 8 && ubxIndex >= (8 + expectedLength)) {
        // Messaggio completo ricevuto
        if (ubxClass == 0x01 && ubxId == 0x35) { // NAV-SAT
          parseUBXNavSat(&ubxBuffer[6], expectedLength);
          Serial.println("# UBX-NAV-SAT received");
        } else if (ubxClass == 0x0A && ubxId == 0x38) { // MON-RF
          parseUBXMonRF(&ubxBuffer[6], expectedLength);
          Serial.println("# UBX-MON-RF received");
        }
        
        // Reset per il prossimo messaggio
        ubxIndex = 0;
        inUBXMessage = false;
        expectedLength = 0;
        break;
      }
      
      if (ubxIndex >= sizeof(ubxBuffer)) {
        ubxIndex = 0;
        inUBXMessage = false;
      }
    }
  }
  
  // Richiedi periodicamente UBX-NAV-SAT e MON-RF se non arrivano dati
  if (millis() - lastUBXRequest > 15000) {
    enableUBXNavSat();
    delay(100);
    enableUBXMonRF();
    lastUBXRequest = millis();
    Serial.println("# Requesting UBX data");
  }

  // Aggiorna display solo ogni secondo
  if (millis() - lastDisplayUpdate >= displayUpdateInterval) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }
  
  // Aggiorna dati RTC
  updateRTCData();
  
  cleanupOldSatellites();


  // Sincronizza RTC con GPS se necessario
  syncRTCWithGPS();
  
  delay(50);
}
// Sostituire la funzione requestUBXJammingInfo() con questa versione:

void performBITETest() {
  unsigned long currentTime = millis();
  
  switch (biteStep) {
    case 0:
      // Mostra schermata BITE TEST
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("BITE TEST:");
      lcd.setCursor(0, 1);
      lcd.print(">RF-CORE: TESTING...");
      lcd.setCursor(0, 2);
      lcd.print(">RTC: TESTING...");
      lcd.setCursor(0, 3);
      lcd.print(">RAM: TESTING...");
      biteStep = 1;
      biteStartTime = currentTime;
      break;
      
    case 1:
      // Test RF-CORE (modulo GPS)
      if (currentTime - biteStartTime > 2000) {
        // Svuota il buffer seriale prima del test
        while (gpsSerial.available()) {
          gpsSerial.read();
        }
        
        // Aspetta 2 secondi e controlla se arrivano dati spontanei dal GPS
        delay(2000);
        
        // Controlla se ci sono dati in arrivo dalla porta GPS
        rfCoreOK = gpsSerial.available() > 0;
        
        // Se ci sono dati, svuota il buffer per non interferire con il normale funzionamento
        if (rfCoreOK) {
          while (gpsSerial.available()) {
            gpsSerial.read();
          }
        }
        
        lcd.setCursor(0, 1);
        if (rfCoreOK) {
          lcd.print(">RF-CORE: OK        ");
          Serial.println("# BITE: RF-CORE OK");
        } else {
          lcd.print(">RF-CORE: FAULT     ");
          Serial.println("# BITE: RF-CORE FAULT");
        }
        
        biteStep = 2;
        biteStartTime = currentTime;
      }
      break;
      
    case 2:
      // Test RTC
      if (currentTime - biteStartTime > 1000) {
        rtcOK = rtc.begin();
        
        lcd.setCursor(0, 2);
        if (rtcOK) {
          lcd.print(">RTC: OK            ");
          Serial.println("# BITE: RTC OK");
          gpsData.rtcAvailable = true;
          updateRTCData();
        } else {
          lcd.print(">RTC: FAULT         ");
          Serial.println("# BITE: RTC FAULT");
          gpsData.rtcAvailable = false;
        }
        
        biteStep = 3;
        biteStartTime = currentTime;
      }
      break;
      
    case 3:
      // Test RAM
      if (currentTime - biteStartTime > 1000) {
        // Test semplice di lettura/scrittura RAM
        volatile uint32_t testValue = 0xAA55AA55;
        volatile uint32_t readValue = testValue;
        ramOK = (readValue == 0xAA55AA55);
        
        // Test aggiuntivo con array
        static uint8_t testArray[100];
        for (int i = 0; i < 100; i++) {
          testArray[i] = i;
        }
        for (int i = 0; i < 100 && ramOK; i++) {
          if (testArray[i] != i) {
            ramOK = false;
          }
        }
        
        lcd.setCursor(0, 3);
        if (ramOK) {
          lcd.print(">RAM: OK            ");
          Serial.println("# BITE: RAM OK");
        } else {
          lcd.print(">RAM: FAULT         ");
          Serial.println("# BITE: RAM FAULT");
        }
        
        biteStep = 4;
        biteStartTime = currentTime;
      }
      break;
      
    case 4:
      // Valuta risultati BITE - RIMOSSO IL CONTROLLO LED QUI
      if (currentTime - biteStartTime > 1000) {
        // Solo log, non controllo LED
        bool anyFault = !rfCoreOK || !rtcOK || !ramOK;
        
        if (anyFault) {
          Serial.println("# BITE: FAULT DETECTED");
        } else {
          Serial.println("# BITE: ALL OK");
        }
        
        biteStep = 5;
        biteStartTime = currentTime;
      }
      break;
      
    case 5:
      // Mostra risultati per 2 secondi aggiuntivi
      if (currentTime - biteStartTime > 2000) {
        biteTestCompleted = true;
        
        // Inizializza GPS se RF-CORE è OK
        if (rfCoreOK) {
          enableUBXNavSat();
          delay(100);
          enableUBXMonRF();
          delay(500);
        }
        
        Serial.println("# BITE test completed");
        Serial.println("# System ready");
        
        // Clear e vai alla prima pagina
        lcd.clear();
        currentPage = 1;
        updateDisplay();
      }
      break;
  }
}

// SOSTITUIRE la funzione updateLEDs() con questa versione corretta:

void updateLEDs() {
  unsigned long currentTime = millis();
  
  // LED ACT - Lampeggia 5 volte al secondo (10 volte considerando ON/OFF)
  if (currentTime - lastActBlink >= actBlinkInterval) {
    actLedState = !actLedState;
    digitalWrite(LED_ACT_PIN, actLedState ? HIGH : LOW);
    lastActBlink = currentTime;
  }
  
  // LED CLOCK - Lampeggia ogni secondo solo se RTC è disponibile
  if (gpsData.rtcAvailable && (currentTime - lastClockBlink >= clockBlinkInterval)) {
    clockLedState = !clockLedState;
    digitalWrite(LED_CLOCK_PIN, clockLedState ? HIGH : LOW);
    lastClockBlink = currentTime;
  } else if (!gpsData.rtcAvailable) {
    // Se RTC non disponibile, spegni il LED
    digitalWrite(LED_CLOCK_PIN, LOW);
    clockLedState = false;
  }
  
  // LED PPS - Lampeggia ogni 5 secondi solo dopo fix GPS
  if (gpsData.hasValidFix && !ppsEnabled) {
    // Abilita PPS quando otteniamo il primo fix
    ppsEnabled = true;
    lastPpsBlink = currentTime;
    Serial.println("# GPS FIX - PPS LED enabled");
  } else if (!gpsData.hasValidFix && ppsEnabled) {
    // Disabilita PPS se perdiamo il fix
    ppsEnabled = false;
    digitalWrite(LED_PPS_PIN, LOW);
    ppsLedState = false;
    Serial.println("# GPS FIX LOST - PPS LED disabled");
  }
  
  if (ppsEnabled && (currentTime - lastPpsBlink >= ppsBlinkInterval)) {
    ppsLedState = !ppsLedState;
    digitalWrite(LED_PPS_PIN, ppsLedState ? HIGH : LOW);
    lastPpsBlink = currentTime;
  }
  
  // LED INTRFC - Lampeggia 5 volte ogni 10 secondi
  // Ciclo di 10 secondi: 5 lampeggi nei primi 5 secondi, poi pausa di 5 secondi
  if (currentTime - intrfcCycleStart >= 10000) {
    // Inizia nuovo ciclo
    intrfcCycleStart = currentTime;
    intrfcBlinkCount = 0;
    lastIntrfcBlink = currentTime;
  }
  
  unsigned long cycleTime = currentTime - intrfcCycleStart;
  
  if (cycleTime < 5000) { // Primi 5 secondi del ciclo - fai lampeggiare
    if (currentTime - lastIntrfcBlink >= intrfcBlinkInterval && intrfcBlinkCount < 10) { // 10 = 5 ON + 5 OFF
      intrfcLedState = !intrfcLedState;
      digitalWrite(LED_INTRFC_PIN, intrfcLedState ? HIGH : LOW);
      lastIntrfcBlink = currentTime;
      intrfcBlinkCount++;
    }
  } else { // Ultimi 5 secondi del ciclo - LED spento
    digitalWrite(LED_INTRFC_PIN, LOW);
    intrfcLedState = false;
  }
  
  // LED FAULT - CONTROLLO CONTINUO DEI FAULT
  // Controlla continuamente se ci sono fault
  bool anyFault = !rfCoreOK || !rtcOK || !ramOK;
  
  if (anyFault != faultLedState) {
    faultLedState = anyFault;
    digitalWrite(LED_FAULT_PIN, faultLedState ? HIGH : LOW);
    
    if (faultLedState) {
      Serial.println("# FAULT DETECTED - LED FAULT ON");
      Serial.print("# Faults: RF=");
      Serial.print(rfCoreOK ? "OK" : "FAULT");
      Serial.print(", RTC=");
      Serial.print(rtcOK ? "OK" : "FAULT");
      Serial.print(", RAM=");
      Serial.println(ramOK ? "OK" : "FAULT");
    } else {
      Serial.println("# ALL OK - LED FAULT OFF");
    }
  }
}


// Funzione per generare PRN a 16 cifre
void generatePRN() {
  String prn = "";
  
  // Leggi ADC da pin GPIO dell'ESP32
  uint16_t adc0 = analogRead(36); // GPIO36 (A0)
  uint16_t adc1 = analogRead(39); // GPIO39 (A3)
  uint16_t adc2 = analogRead(34); // GPIO34 (A6)
  uint16_t adc3 = analogRead(35); // GPIO35 (A7)
  
  // Leggi temperatura dal DS3231 RTC invece del sensore interno ESP32
  float rtcTemp = rtc.getTemperature();
  uint16_t temp = (uint16_t)(rtcTemp * 100); // Converti in centesimi per più precisione
  
  // Combina i valori per creare entropia
  uint32_t entropy1 = (adc0 << 16) | adc1;
  uint32_t entropy2 = (adc2 << 16) | adc3;
  uint32_t entropy3 = (temp << 16) | (millis() & 0xFFFF);
  
  // XOR delle entropie per maggiore casualità
  uint32_t finalEntropy = entropy1 ^ entropy2 ^ entropy3;
  
  // Usa il rumore dei bit meno significativi degli ADC
  uint16_t noise = (adc0 & 0xF) | ((adc1 & 0xF) << 4) | ((adc2 & 0xF) << 8) | ((adc3 & 0xF) << 12);
  
  // Aggiungi rumore dalla temperatura RTC
  noise ^= (temp & 0xFF);
  
  // Genera 16 cifre usando l'entropia combinata
  for (int i = 0; i < 16; i++) {
    // Combina entropia finale, rumore e posizione per ogni cifra
    uint32_t seed = finalEntropy + noise + i + micros();
    
    // Estrai una cifra usando operazioni bit
    uint8_t digit = ((seed >> (i * 2)) ^ (seed >> (8 + i))) & 0xFF;
    digit = (digit * 10) >> 8; // Converti in 0-9
    
    prn += String(digit);
    
    // Ruota l'entropia per la prossima cifra
    finalEntropy = (finalEntropy << 1) | (finalEntropy >> 31);
    noise = (noise << 1) | (noise >> 15);
  }
  
  currentPRN = prn;
  Serial.println("# PRN Generated: " + currentPRN);
}


// ==== FUNZIONE enableUBXMonRF CORRETTA ====
void enableUBXMonRF() {
  // Comando UBX-CFG-VALSET per abilitare MSGOUT-UBX_MON_RF_I2C
  // Header: B5 62 (sync chars) + 06 8A (class/id) + 09 00 (length)
  // Payload: 00 01 00 00 (version, layer, reserved) + 5A 03 91 20 (key) + 01 (value)
  uint8_t ubxCmd[] = {
    0xB5, 0x62,           // Sync chars
    0x06, 0x8A,           // Class/ID (CFG-VALSET)
    0x09, 0x00,           // Payload length (9 bytes)
    0x00,                 // Version (0)
    0x01,                 // Layer (RAM)
    0x00, 0x00,           // Reserved
    0x5A, 0x03, 0x91, 0x20, // Key CFG-MSGOUT-UBX_MON_RF_I2C
    0x01                  // Value (enable = 1)
  };
  
  // Calcola checksum (dalla posizione 2 in poi, esclusi sync chars)
  uint8_t ck_a = 0, ck_b = 0;
  for (int i = 2; i < sizeof(ubxCmd); i++) {
    ck_a += ubxCmd[i];
    ck_b += ck_a;
  }
  
  // Invia comando completo
  for (int i = 0; i < sizeof(ubxCmd); i++) {
    gpsSerial.write(ubxCmd[i]);
  }
  gpsSerial.write(ck_a);
  gpsSerial.write(ck_b);
  
  Serial.println("UBX-MON-RF abilitato");
}

void enableUBXNavSat() {
  // Comando UBX per abilitare NAV-SAT
  uint8_t ubxCmd[] = {
    0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 
    0x00, 0x01, 0x00, 0x00, 0x16, 0x00, 
    0x91, 0x20, 0x01, 0x62, 0x93
  };
  
  // Invia comando
  for (int i = 0; i < sizeof(ubxCmd); i++) {
    gpsSerial.write(ubxCmd[i]);
  }
  
  Serial.println("UBX-NAV-SAT abilitato con comando binario");
}

// 3. AGGIUNGERE anche alla fine della funzione parseUBXNavSat:
void parseUBXNavSat(uint8_t* payload, uint16_t length) {
  if (length < 8) return;
  
  ubxNavSatData.iTOW = (uint32_t)(payload[0] | (payload[1] << 8) | (payload[2] << 16) | (payload[3] << 24));
  uint8_t version = payload[4];
  ubxNavSatData.numSvs = payload[5];
  
  // Limita il numero di satelliti
  if (ubxNavSatData.numSvs > 12) ubxNavSatData.numSvs = 12;
  
  // Parsing dei dati satelliti (ogni satellite occupa 12 bytes)
  for (int i = 0; i < ubxNavSatData.numSvs && i < 12; i++) {
    int offset = 8 + (i * 12);
    if (offset + 11 >= length) break;
    
    ubxNavSatData.satellites[i].gnssId = payload[offset];
    ubxNavSatData.satellites[i].svId = payload[offset + 1];
    ubxNavSatData.satellites[i].cno = payload[offset + 2];
    ubxNavSatData.satellites[i].elev = (int8_t)payload[offset + 3];
    ubxNavSatData.satellites[i].azim = (int16_t)(payload[offset + 4] | (payload[offset + 5] << 8));
    
    uint32_t flags = (uint32_t)(payload[offset + 8] | (payload[offset + 9] << 8) | 
                                (payload[offset + 10] << 16) | (payload[offset + 11] << 24));
    
    ubxNavSatData.satellites[i].qualityInd = flags & 0x07;
    ubxNavSatData.satellites[i].svUsed = (flags & 0x08) != 0;
    ubxNavSatData.satellites[i].health = (flags >> 4) & 0x03;
    ubxNavSatData.satellites[i].orbitSource = (flags >> 8) & 0x07;
  }
  
  ubxNavSatData.dataAvailable = true;
  
  // AGGIUNGERE questa riga alla fine:
  calculateAverageSST();
}

String getConstellationLetter(uint8_t gnssId) {
  switch (gnssId) {
    case 0: return "G"; // GPS
    case 1: return "S"; // SBAS
    case 2: return "R"; // Galileo
    case 3: return "E"; // BeiDou
    case 4: return "I"; // IMES
    case 5: return "Q"; // QZSS
    case 6: return "L"; // GLONASS
    default: return "?";
  }
}

String getOrbitSourceString(uint8_t orbitSource) {
  switch (orbitSource) {
    case 0: return "OTH";
    case 1: return "EPH";
    case 2: return "ALM";
    case 3: 
    case 4:
    case 5:
    case 6:
    case 7: return "OTH";
    default: return "OTH";
  }
}



void handleButton() {
  static bool lastButtonState = HIGH;
  bool currentButtonState = digitalRead(BUTTON_PIN);
  
  if (lastButtonState == HIGH && currentButtonState == LOW && 
      (millis() - lastButtonPress) > debounceTime) {
    
    lastButtonPress = millis();
    currentPage++;
    if (currentPage > totalPages) {
      currentPage = 1;
    }
    
    // Clear immediato e aggiornamento forzato
    lcd.clear();
    delay(50);
    updateDisplay();
    lastDisplayUpdate = millis(); // Reset timer
  }
  
  lastButtonState = currentButtonState;
}

void processNMEA(String nmea) {
  nmea.trim();
  
  if (nmea.startsWith("$GPGGA") || nmea.startsWith("$GNGGA")) {
    parseGGA(nmea);
  } else if (nmea.startsWith("$GPGSA") || nmea.startsWith("$GNGSA")) {
    parseGSA(nmea);
  } else if (nmea.startsWith("$GPGSV") || nmea.startsWith("$GNGSV") || 
             nmea.startsWith("$GLGSV") || nmea.startsWith("$GAGSV") || 
             nmea.startsWith("$BDGSV") || nmea.startsWith("$QZGSV")) {
    parseGSV(nmea);
  } else if (nmea.startsWith("$GPRMC") || nmea.startsWith("$GNRMC")) {
    parseRMC(nmea);
  } else if (nmea.startsWith("$GPVTG") || nmea.startsWith("$GNVTG")) {
    parseVTG(nmea);
  }
}

// 1. AGGIUNGERE questa funzione per calcolare SST medio
void calculateAverageSST() {
  float totalSNR = 0.0;
  int validSatellites = 0;
  
  // Calcola media dai satelliti GSV con SNR > 0
  for (int i = 0; i < 4; i++) {
    if (gpsData.satellites_info[i].snr > 0) {
      totalSNR += gpsData.satellites_info[i].snr;
      validSatellites++;
    }
  }
  
  // Aggiungi SBAS se disponibile
  if (gpsData.sbasSat.snr > 0) {
    totalSNR += gpsData.sbasSat.snr;
    validSatellites++;
  }
  
  // Calcola media o imposta 0 se nessun satellite valido
  if (validSatellites > 0) {
    gpsData.signalStrength = totalSNR / validSatellites;
  } else {
    gpsData.signalStrength = 0.0;
  }
}

void parseGGA(String sentence) {
  int commaCount = 0;
  int startPos = 0;
  String fields[15];
  
  for (int i = 0; i < sentence.length(); i++) {
    if (sentence.charAt(i) == ',' || i == sentence.length() - 1) {
      fields[commaCount] = sentence.substring(startPos, i);
      startPos = i + 1;
      commaCount++;
      if (commaCount >= 15) break;
    }
  }
  
  // Parse GGA fields
  int fixQuality = fields[6].toInt();
  int satelliteCount = fields[7].toInt();
  float hdopValue = fields[8].toFloat();
  
  // Check if we have valid coordinate data
  bool hasValidCoordinates = (fields[2].length() > 0 && fields[3].length() > 0 && 
                              fields[4].length() > 0 && fields[5].length() > 0);
  
  // Check if we have valid time data
  bool hasValidTime = (fields[1].length() >= 6);
  
  // Determine fix status based on GGA quality indicator
  if (fixQuality == 0) {
    if (hasValidTime) {
      gpsData.fixStatus = "TIME REF ONLY";
      gpsData.hasValidFix = false;
      gpsData.hasTimeOnly = true;
    } else {
      gpsData.fixStatus = "NO CARRIER";
      gpsData.hasValidFix = false;
      gpsData.hasTimeOnly = false;
    }
  } else if (fixQuality == 6) {
    // Dead reckoning mode - time reference only
    gpsData.fixStatus = "TIME REF ONLY";
    gpsData.hasValidFix = false;
    gpsData.hasTimeOnly = true;
  } else {
    // We have some kind of position fix
    gpsData.hasValidFix = true;
    gpsData.hasTimeOnly = false;
    
    switch (fixQuality) {
      case 1: 
        gpsData.fixStatus = "2D FIX";
        break;
      case 2: 
        gpsData.fixStatus = "3D FIX";
        break;
      case 3:
        gpsData.fixStatus = "PPS FIX";
        break;
      case 4:
        gpsData.fixStatus = "RTK FIX";
        break;
      case 5:
        gpsData.fixStatus = "RTK FLOAT";
        break;
      case 7:
        gpsData.fixStatus = "MANUAL";
        break;
      case 8:
        gpsData.fixStatus = "SIMULATION";
        break;
      default: 
        gpsData.fixStatus = "UNKNOWN FIX";
        break;
    }
  }
  
  // Always update these values from GGA
  gpsData.satellites = satelliteCount;
  gpsData.precision = hdopValue;
  
  // Handle coordinates - only if we have a valid position fix
  if (gpsData.hasValidFix && hasValidCoordinates) {
    gpsData.latitude = convertToDMS(fields[2], fields[3]);
    gpsData.longitude = convertToDMS(fields[4], fields[5]);
    
    // Update altitude only with valid fix
    gpsData.altitude = fields[9].toFloat();
    gpsData.geoidHeight = fields[11].toFloat();
  } else {
    // Clear position data if no valid fix
    gpsData.latitude = ""; 
    gpsData.longitude = "";
    gpsData.altitude = 0.0;
    gpsData.geoidHeight = 0.0;
  }
  
  // Update listen status
  if (gpsData.hasValidFix) {
    gpsData.listenStatus = "ONLINE";
  } else if (gpsData.hasTimeOnly) {
    gpsData.listenStatus = "TIME ONLY";
  } else {
    gpsData.listenStatus = "OFFLINE";
  }
  
  // Parse time data (always attempt if present, regardless of fix)
  if (hasValidTime) {
    String timeStr = fields[1];
    if (timeStr.length() >= 6) {
      String hour = timeStr.substring(0, 2);
      String minute = timeStr.substring(2, 4);
      String second = timeStr.substring(4, 6);
      gpsData.utcTime = hour + ":" + minute + ":" + second + " UTC";
      
      // Local time (assuming UTC+1 for Italy)
      int localHour = hour.toInt() + 1;
      if (localHour >= 24) localHour -= 24;
      String localHourStr = (localHour < 10) ? "0" + String(localHour) : String(localHour);
      gpsData.localTime = localHourStr + ":" + minute + ":" + second + " CET";
    }
  }
  
  // Debug output
  Serial.print("# GGA: Quality=");
  Serial.print(fixQuality);
  Serial.print(", Sats=");
  Serial.print(satelliteCount);
  Serial.print(", HDOP=");
  Serial.print(hdopValue);
  Serial.print(", HasCoords=");
  Serial.print(hasValidCoordinates ? "YES" : "NO");
  Serial.print(", HasTime=");
  Serial.print(hasValidTime ? "YES" : "NO");
  Serial.print(", Status=");
  Serial.println(gpsData.fixStatus);
}


void parseGSA(String sentence) {
  int commaCount = 0;
  int startPos = 0;
  String fields[18];
  
  for (int i = 0; i < sentence.length(); i++) {
    if (sentence.charAt(i) == ',' || i == sentence.length() - 1) {
      fields[commaCount] = sentence.substring(startPos, i);
      startPos = i + 1;
      commaCount++;
      if (commaCount >= 18) break;
    }
  }
  
  // Fix type from GSA (more detailed than GGA)
  int fixType = fields[2].toInt();
  
  // Only update fix status if we already have some kind of fix from GGA
  // GSA provides more detail about 2D vs 3D
  if (gpsData.hasValidFix) {
    if (fixType == 2) {
      gpsData.fixStatus = "2D FIX";
    } else if (fixType == 3) {
      gpsData.fixStatus = "3D FIX";
    }
    // Don't change fix status if fixType is 1 (no fix) - let GGA handle it
  }
  
  // Update dilution of precision values
  gpsData.hdop = fields[16].length() > 0 ? fields[16].toFloat() : 0.0;
  
  // Track satellites in use (fields 3-14)
  satellitesInUseCount = 0;
  for (int i = 3; i < 15 && i < commaCount; i++) {
    if (fields[i].length() > 0 && satellitesInUseCount < 20) {
      satellitesInUse[satellitesInUseCount] = fields[i].toInt();
      satellitesInUseCount++;
    }
  }
  
  Serial.print("# GSA: FixType=");
  Serial.print(fixType);
  Serial.print(", SatsInUse=");
  Serial.print(satellitesInUseCount);
  Serial.print(", HDOP=");
  Serial.println(gpsData.hdop);
}

// Helper function to clear all position-related data
void clearPositionData() {
  gpsData.latitude = "";
  gpsData.longitude = "";
  gpsData.altitude = 0.0;
  gpsData.geoidHeight = 0.0;
  gpsData.speed = 0.0;
  gpsData.bearing = 0.0;
  gpsData.magneticVariation = 0.0;
}

// Helper function to clear all data when no signal
void clearAllGPSData() {
  clearPositionData();
  gpsData.utcTime = "";
  gpsData.localTime = "";
  gpsData.date = "";
  gpsData.timeAndDateFix = "";
  gpsData.validity = "V";
  gpsData.warning = "V";
}

// Modifica anche il parseRMC per essere più conservativo
void parseRMC(String sentence) {
  int commaCount = 0;
  int startPos = 0;
  String fields[13];
  
  // Parse all fields from RMC sentence
  for (int i = 0; i < sentence.length(); i++) {
    if (sentence.charAt(i) == ',' || i == sentence.length() - 1) {
      fields[commaCount] = sentence.substring(startPos, i);
      startPos = i + 1;
      commaCount++;
      if (commaCount >= 13) break;
    }
  }
  
  // Status (field 2) - A=Active (valid), V=Void (invalid)
  gpsData.validity = fields[2];
  
  // Warning field - opposite of validity for compatibility
  if (fields[2] == "A") {
    gpsData.warning = "A"; // All OK
  } else {
    gpsData.warning = "V"; // Warning/Invalid
  }
  
  // Solo aggiorna velocità e bearing se RMC è valido E abbiamo un fix dal GGA
  bool rmcValid = (fields[2] == "A");
  
  if (rmcValid && gpsData.hasValidFix) {
    // Speed in knots (field 7) - convert to km/h
    if (fields[7].length() > 0) {
      gpsData.speed = fields[7].toFloat() * 1.852; // knots to km/h
    }
    
    // Course over ground (field 8)
    if (fields[8].length() > 0) {
      gpsData.bearing = fields[8].toFloat();
    }
  } else {
    // Se RMC non valido o non abbiamo fix, azzera velocità e bearing
    gpsData.speed = 0.0;
    gpsData.bearing = 0.0;
  }
  
  // Date (field 9) - aggiorna sempre se presente
  if (fields[9].length() == 6) {
    String day = fields[9].substring(0, 2);
    String month = fields[9].substring(2, 4);
    String year = "20" + fields[9].substring(4, 6);
    gpsData.date = day + "/" + month + "/" + year;
  }
  
  // Time (field 1) - aggiorna sempre se presente
  if (fields[1].length() >= 6) {
    String hour = fields[1].substring(0, 2);
    String minute = fields[1].substring(2, 4);
    String second = fields[1].substring(4, 6);
    gpsData.utcTime = hour + ":" + minute + ":" + second + " UTC";
    
    // Local time (assuming UTC+1 for Italy)
    int localHour = hour.toInt() + 1;
    if (localHour >= 24) localHour -= 24;
    String localHourStr = (localHour < 10) ? "0" + String(localHour) : String(localHour);
    gpsData.localTime = localHourStr + ":" + minute + ":" + second + " CET";
    
    gpsData.timeAndDateFix = gpsData.utcTime + " - " + gpsData.date;
    
    // Save valid GPS time for RTC synchronization solo se RMC valido
    if (rmcValid && fields[9].length() == 6) {
      int gpsYear = 2000 + fields[9].substring(4, 6).toInt();
      int gpsMonth = fields[9].substring(2, 4).toInt();
      int gpsDay = fields[9].substring(0, 2).toInt();
      int gpsHour = hour.toInt();
      int gpsMinute = minute.toInt();
      int gpsSecond = second.toInt();
      
      gpsData.lastValidGPSTime = DateTime(gpsYear, gpsMonth, gpsDay, gpsHour, gpsMinute, gpsSecond);
      gpsData.hasValidGPSTime = true;
      gpsData.lastGPSSync = millis();
    }
  }
  
  // Magnetic variation - solo se RMC valido
  if (rmcValid && fields[10].length() > 0) {
    float magVar = fields[10].toFloat();
    String magDir = fields[11];
    
    if (magDir == "W") {
      magVar = -magVar;
    }
    
    gpsData.magneticVariation = magVar;
  } else {
    gpsData.magneticVariation = 0.0;
  }
  
  // Debug output
  Serial.print("# RMC: Valid=");
  Serial.print(rmcValid ? "YES" : "NO");
  Serial.print(", HasFix=");
  Serial.print(gpsData.hasValidFix ? "YES" : "NO");
  Serial.print(", Speed=");
  Serial.print(gpsData.speed);
  Serial.print(", Bearing=");
  Serial.println(gpsData.bearing);
}



// Aggiungere funzione per verificare se un satellite è in uso:
bool isSatelliteInUse(int prn) {
  for (int i = 0; i < satellitesInUseCount; i++) {
    if (satellitesInUse[i] == prn) {
      return true;
    }
  }
  return false;
}

// Aggiungere funzione per determinare il simbolo del satellite:
String getSatelliteSymbol(const GPSData::Satellite& sat) {
  if (sat.prn == 0 || sat.constellation.length() == 0) {
    return " "; // Satellite non presente
  }
  
  // Satellite presente ma senza altri dati (solo PRN)
  if (sat.snr == 0 && sat.azimuth == 0 && sat.elevation == 0) {
    return "?";
  }
  
  // Satellite non visibile ma con dati (SNR = 0 ma con azimuth/elevation)
  if (sat.snr == 0 && (sat.azimuth > 0 || sat.elevation > 0)) {
    return "*";
  }
  
  // Satellite visibile (SNR > 0)
  if (sat.snr > 0) {
    // Controlla se è in uso
    if (isSatelliteInUse(sat.prn)) {
      return "$"; // Visibile e in uso
    } else {
      return "#"; // Visibile ma non in uso
    }
  }
  
  return "?"; // Default per casi non coperti
}

void parseGSV(String sentence) {
  int commaCount = 0;
  int startPos = 0;
  String fields[20];
  
  for (int i = 0; i < sentence.length(); i++) {
    if (sentence.charAt(i) == ',' || i == sentence.length() - 1) {
      fields[commaCount] = sentence.substring(startPos, i);
      startPos = i + 1;
      commaCount++;
      if (commaCount >= 20) break;
    }
  }
  
  String constellation = getConstellation(sentence);
  
  // Processa fino a 4 satelliti per messaggio GSV
  int satCount = 0;
  for (int i = 4; i < commaCount && satCount < 4; i += 4, satCount++) {
    if (fields[i].length() > 0) {
      // Validazione e pulizia dei dati
      int prn = fields[i].toInt();
      int elevation = fields[i+1].length() > 0 ? fields[i+1].toInt() : 0;
      int azimuth = fields[i+2].length() > 0 ? fields[i+2].toInt() : 0;
      int snr = fields[i+3].length() > 0 ? fields[i+3].toInt() : 0;
      
      // Validazione ranges ragionevoli
      if (prn > 0 && prn <= 255 && elevation >= 0 && elevation <= 90 && 
          azimuth >= 0 && azimuth <= 359 && snr >= 0 && snr <= 99) {
        
        // Aggiungi al pool satelliti
        addSatelliteToPool(constellation, prn, elevation, azimuth, snr);
        
        // Check per SBAS con validazione
        if (prn >= 120 && prn <= 158) {
          gpsData.sbasSat.constellation = constellation;
          gpsData.sbasSat.prn = prn;
          gpsData.sbasSat.elevation = elevation;
          gpsData.sbasSat.azimuth = azimuth;
          gpsData.sbasSat.snr = snr;
        }
      }
    }
  }
  
  // Aggiorna i satelliti visualizzati per la pagina 3
  updateDisplayedSatellites();
  
  // Calcola SST medio
  calculateAverageSST();
}

void addSatelliteToPool(String constellation, int prn, int elevation, int azimuth, int snr) {
  // Cerca se il satellite esiste già nel pool
  int existingIndex = -1;
  for (int i = 0; i < satellitePool.count; i++) {
    if (satellitePool.satellites[i].prn == prn && 
        satellitePool.satellites[i].constellation == constellation) {
      existingIndex = i;
      break;
    }
  }
  
  if (existingIndex >= 0) {
    // Aggiorna satellite esistente
    satellitePool.satellites[existingIndex].elevation = elevation;
    satellitePool.satellites[existingIndex].azimuth = azimuth;
    satellitePool.satellites[existingIndex].snr = snr;
    satellitePool.satellites[existingIndex].inUse = isSatelliteInUse(prn);
  } else if (satellitePool.count < 32) {
    // Aggiungi nuovo satellite
    satellitePool.satellites[satellitePool.count].constellation = constellation;
    satellitePool.satellites[satellitePool.count].prn = prn;
    satellitePool.satellites[satellitePool.count].elevation = elevation;
    satellitePool.satellites[satellitePool.count].azimuth = azimuth;
    satellitePool.satellites[satellitePool.count].snr = snr;
    satellitePool.satellites[satellitePool.count].inUse = isSatelliteInUse(prn);
    satellitePool.count++;
  }
  
  satellitePool.lastUpdate = millis();
}

void updateDisplayedSatellites() {
  // Aggiorna lo stato "in uso" per tutti i satelliti nel pool
  for (int i = 0; i < satellitePool.count; i++) {
    satellitePool.satellites[i].inUse = isSatelliteInUse(satellitePool.satellites[i].prn);
  }
  
  // Ruota i satelliti visualizzati ogni 5 secondi
  if (millis() - lastSatelliteRotation >= satelliteRotationInterval && satellitePool.count > 4) {
    // Sposta gli indici avanti di 4 posizioni
    for (int i = 0; i < 4; i++) {
      displayedSatelliteIndices[i] += 4;
      if (displayedSatelliteIndices[i] >= satellitePool.count) {
        displayedSatelliteIndices[i] = i; // Ricomincia dal principio
      }
    }
    lastSatelliteRotation = millis();
    Serial.println("# Satellite display rotated");
  }
  
  // Copia i satelliti selezionati nell'array di visualizzazione
  for (int i = 0; i < 4; i++) {
    if (displayedSatelliteIndices[i] < satellitePool.count) {
      gpsData.satellites_info[i] = satellitePool.satellites[displayedSatelliteIndices[i]];
    } else {
      // Azzera slot se non ci sono abbastanza satelliti
      gpsData.satellites_info[i].constellation = "";
      gpsData.satellites_info[i].prn = 0;
      gpsData.satellites_info[i].elevation = 0;
      gpsData.satellites_info[i].azimuth = 0;
      gpsData.satellites_info[i].snr = 0;
      gpsData.satellites_info[i].inUse = false;
    }
  }
}

// CORREZIONE 5: Aggiungere questa funzione per pulire satelliti vecchi
void cleanupOldSatellites() {
  static unsigned long lastCleanup = 0;
  
  // Pulisci ogni 30 secondi
  if (millis() - lastCleanup < 30000) {
    return;
  }
  
  // Se non riceviamo dati satelliti da più di 1 minuto, azzera il pool
  if (millis() - satellitePool.lastUpdate > 60000) {
    satellitePool.count = 0;
    Serial.println("# Satellite pool cleared - no recent data");
    
    // Azzera anche i dati visualizzati
    for (int i = 0; i < 4; i++) {
      gpsData.satellites_info[i].constellation = "";
      gpsData.satellites_info[i].prn = 0;
      gpsData.satellites_info[i].elevation = 0;
      gpsData.satellites_info[i].azimuth = 0;
      gpsData.satellites_info[i].snr = 0;
      gpsData.satellites_info[i].inUse = false;
    }
  }
  
  lastCleanup = millis();
}




void parseVTG(String sentence) {
  int commaCount = 0;
  int startPos = 0;
  String fields[10];
  
  for (int i = 0; i < sentence.length(); i++) {
    if (sentence.charAt(i) == ',' || i == sentence.length() - 1) {
      fields[commaCount] = sentence.substring(startPos, i);
      startPos = i + 1;
      commaCount++;
      if (commaCount >= 10) break;
    }
  }
  
  gpsData.bearing = fields[1].toFloat();
  gpsData.speed = fields[7].toFloat(); // km/h
}

String getConstellation(String sentence) {
  if (sentence.startsWith("$GP")) return "$GPS";
  else if (sentence.startsWith("$GL")) return "$GLN";
  else if (sentence.startsWith("$GA")) return "$GAL";
  else if (sentence.startsWith("$BD")) return "$BDU";
  else if (sentence.startsWith("$QZ")) return "$QZS";
  else if (sentence.startsWith("$GN")) return "$GPS";
  return "$UNK";
}

String convertToDMS(String coord, String direction) {
  if (coord.length() < 4) return "";
  
  float decimal = coord.toFloat();
  int degrees = (int)(decimal / 100);
  float minutes = decimal - (degrees * 100);
  
  String result = String(degrees) + "^" + String(minutes, 4) + "'" + direction;
  return result;
}


// ==== FUNZIONE parseUBXMonRF CORRETTA ====
void parseUBXMonRF(uint8_t* payload, uint16_t length) {
  if (length < 4) return;
  
  uint8_t version = payload[0];
  uint8_t nBlocks = payload[1];
  // payload[2] e payload[3] sono reserved0
  
  // Limita il numero di blocchi RF
  if (nBlocks > 2) nBlocks = 2;
  gpsData.numRFBlocks = nBlocks;
  
  Serial.print("UBX-MON-RF: Version=");
  Serial.print(version);
  Serial.print(", Blocks=");
  Serial.println(nBlocks);
  
  if (version == 0x00) {
    // Versione 0 (DATA0) - ogni blocco è 24 bytes
    for (int i = 0; i < nBlocks && i < 2; i++) {
      int offset = 4 + (i * 24);
      if (offset + 23 >= length) break;
      
      gpsData.rfBlocks[i].blockId = payload[offset + 0];
      // flags a payload[offset + 1] (1 byte) - non utilizzato
      gpsData.rfBlocks[i].jammingState = payload[offset + 2];
      gpsData.rfBlocks[i].antStatus = payload[offset + 3];
      gpsData.rfBlocks[i].antPower = payload[offset + 4];
      // postStatus a payload[offset + 5] (4 bytes: offset+5,+6,+7,+8) - non utilizzato
      // reserved1 a payload[offset + 9] (1 byte) - non utilizzato
      gpsData.rfBlocks[i].noisePerMS = payload[offset + 10] | (payload[offset + 11] << 8);
      gpsData.rfBlocks[i].agcCnt = payload[offset + 12] | (payload[offset + 13] << 8);
      gpsData.rfBlocks[i].cwSuppression = payload[offset + 14];
      gpsData.rfBlocks[i].ofsI = (int8_t)payload[offset + 15];
      gpsData.rfBlocks[i].magI = payload[offset + 16];
      gpsData.rfBlocks[i].ofsQ = (int8_t)payload[offset + 17];
      gpsData.rfBlocks[i].magQ = payload[offset + 18];
      gpsData.rfBlocks[i].rfBlockGnssBand = payload[offset + 19];
      // reserved2 a payload[offset + 20] (4 bytes: +20,+21,+22,+23) - non utilizzato
      
      Serial.print("Block ");
      Serial.print(i);
      Serial.print(": JamState=");
      Serial.print(gpsData.rfBlocks[i].jammingState);
      Serial.print(", AntStatus=");
      Serial.print(gpsData.rfBlocks[i].antStatus);
      Serial.print(", CW=");
      Serial.print(gpsData.rfBlocks[i].cwSuppression);
      Serial.print(", AGC=");
      Serial.print(gpsData.rfBlocks[i].agcCnt);
      Serial.print(", Noise=");
      Serial.println(gpsData.rfBlocks[i].noisePerMS);
    }
  } else if (version == 0x01) {
    // Versione 1 (DATA1) - ogni blocco è 20 bytes
    for (int i = 0; i < nBlocks && i < 2; i++) {
      int offset = 4 + (i * 20);
      if (offset + 19 >= length) break;
      
      gpsData.rfBlocks[i].blockId = payload[offset + 0];
      gpsData.rfBlocks[i].antStatus = payload[offset + 1];
      gpsData.rfBlocks[i].antPower = payload[offset + 2];
      gpsData.rfBlocks[i].cwSuppression = payload[offset + 3];
      // postStatus a payload[offset + 4] (4 bytes: +4,+5,+6,+7) - non utilizzato
      // reserved1 a payload[offset + 8] (1 byte) - non utilizzato
      gpsData.rfBlocks[i].noisePerMS = payload[offset + 9] | (payload[offset + 10] << 8);
      gpsData.rfBlocks[i].agcCnt = payload[offset + 11] | (payload[offset + 12] << 8);
      gpsData.rfBlocks[i].ofsI = (int8_t)payload[offset + 13];
      gpsData.rfBlocks[i].magI = payload[offset + 14];
      gpsData.rfBlocks[i].ofsQ = (int8_t)payload[offset + 15];
      gpsData.rfBlocks[i].magQ = payload[offset + 16];
      // Nella versione 1 non c'è rfBlockGnssBand, quindi impostiamo 0 (unknown)
      gpsData.rfBlocks[i].rfBlockGnssBand = 0;
      // payload[offset + 17], [18], [19] sono reserved - non utilizzati
      
      // Nella versione 1, jammingState non è presente nel messaggio
      // Lo stimiamo basandoci su cwSuppression (metodo euristico)
      if (gpsData.rfBlocks[i].cwSuppression == 0) {
        gpsData.rfBlocks[i].jammingState = 1; // OK - nessun jamming CW
      } else if (gpsData.rfBlocks[i].cwSuppression < 64) {
        gpsData.rfBlocks[i].jammingState = 2; // Warning - jamming leggero
      } else {
        gpsData.rfBlocks[i].jammingState = 3; // Critical - jamming forte
      }
      
      Serial.print("Block ");
      Serial.print(i);
      Serial.print(": AntStatus=");
      Serial.print(gpsData.rfBlocks[i].antStatus);
      Serial.print(", CW=");
      Serial.print(gpsData.rfBlocks[i].cwSuppression);
      Serial.print(", AGC=");
      Serial.print(gpsData.rfBlocks[i].agcCnt);
      Serial.print(", Noise=");
      Serial.println(gpsData.rfBlocks[i].noisePerMS);
    }
  }
  
  gpsData.rfDataAvailable = true;
}


void updateRTCData() {
  static unsigned long lastRTCRead = 0;
  
  // Leggi RTC solo ogni 500ms per evitare sovraccarico I2C
  if (millis() - lastRTCRead < 500) {
    return;
  }
  lastRTCRead = millis();
  
  if (!gpsData.rtcAvailable) {
    // Se RTC non disponibile, usa valori di fallback
    gpsData.rtcDate = "RTC ERROR";
    gpsData.rtcTime = "NO RTC";
    gpsData.rtcTemperature = 298.15;
    return;
  }
  
  DateTime now = rtc.now();
  
  // Verifica che la data sia valida
  if (now.year() >= 2020 && now.year() <= 2050) {
    // Data RTC
    String day = (now.day() < 10) ? "0" + String(now.day()) : String(now.day());
    String month = (now.month() < 10) ? "0" + String(now.month()) : String(now.month());
    gpsData.rtcDate = day + "/" + month + "/" + String(now.year());
    
    // Ora RTC
    String hour = (now.hour() < 10) ? "0" + String(now.hour()) : String(now.hour());
    String minute = (now.minute() < 10) ? "0" + String(now.minute()) : String(now.minute());
    String second = (now.second() < 10) ? "0" + String(now.second()) : String(now.second());
    gpsData.rtcTime = hour + ":" + minute + ":" + second;
    
    // Temperatura RTC (in Celsius, convertita in Kelvin)
    gpsData.rtcTemperature = rtc.getTemperature() + 273.15;
  } else {
    // Se data non valida, usa valori di fallback
    gpsData.rtcDate = "01/01/2024";
    gpsData.rtcTime = "00:00:00";
    gpsData.rtcTemperature = 298.15;
  }
}


void syncRTCWithGPS() {
  // Sincronizza RTC con GPS ogni 10 minuti se abbiamo dati validi
  if (!gpsData.rtcAvailable || !gpsData.hasValidGPSTime) {
    return;
  }
  
  // Controlla se è passato abbastanza tempo dall'ultima sincronizzazione
  if (millis() - gpsData.lastGPSSync < 600000) { // 10 minuti
    return;
  }
  
  // Aggiorna RTC con l'ultimo tempo GPS valido
  rtc.adjust(gpsData.lastValidGPSTime);
  Serial.println("RTC sincronizzato con GPS");
  
  // Aggiorna timestamp
  gpsData.lastRTCUpdate = millis();
}

void updateDisplay() {
  switch (currentPage) {
    case 1: displayPage1(); break;
    case 2: displayPage2(); break;
    case 3: displayPage3(); break;
    case 4: displayPage4(); break;
    case 5: displayPage5(); break;
    case 6: displayPage6(); break;
    case 7: displayPage7(); break;
    case 8: displayPage8(); break;
    case 9: displayPage9(); break; 
    case 10: displayPage10(); break;
    case 11: displayPage11(); break;
    
  }
}

void displayPage1() {
  lcd.setCursor(0, 0);
  lcd.print("FIXST:");
  lcd.setCursor(8, 0);
  lcd.print(gpsData.fixStatus);
  
  lcd.setCursor(0, 1);
  lcd.print("SATELLITES: ");
  lcd.print(gpsData.satellites);
  
  lcd.setCursor(0, 2);
  lcd.print("SST:");
  lcd.setCursor(5, 2);
  lcd.print(String(gpsData.signalStrength, 1) + " dB");
  
  lcd.setCursor(0, 3);
  lcd.print("PRCS:");
  lcd.print(String(gpsData.precision, 1) + "m");
}

void displayPage2() {
  lcd.setCursor(0, 0);
  lcd.print("BRNG:");
  lcd.print(String(gpsData.bearing, 1));
  lcd.print("");
  
  lcd.setCursor(11, 0);
  lcd.print("ALTD:");
  
  lcd.setCursor(16, 0);
  lcd.print(String((int)gpsData.altitude));
  
  lcd.setCursor(0, 1);
  lcd.print("SPD:");
  lcd.print(String(gpsData.speed, 1));
  lcd.print("");
  
  lcd.setCursor(11, 1);
  lcd.print("GHEL:");
  
  lcd.setCursor(16, 1);
  lcd.print(String((int)gpsData.geoidHeight));
  
  lcd.setCursor(0, 2);
  lcd.print("LAT: ");
  lcd.print(gpsData.latitude.substring(0, 14));
  
  lcd.setCursor(0, 3);
  lcd.print("LON: ");
  lcd.print(gpsData.longitude.substring(0, 14));
}

// Modificare la funzione displayPage3:
void displayPage3() {
  bool hasSatellites = false;
  
  // Controlla se ci sono satelliti validi da visualizzare
  for (int i = 0; i < 4; i++) {
    if (gpsData.satellites_info[i].prn > 0 && 
        gpsData.satellites_info[i].constellation.length() > 0) {
      hasSatellites = true;
      break;
    }
  }
  
  if (hasSatellites) {
    // Se ci sono satelliti, visualizzali con formattazione corretta
    for (int i = 0; i < 4; i++) {
      lcd.setCursor(0, i);
      if (gpsData.satellites_info[i].prn > 0 && 
          gpsData.satellites_info[i].constellation.length() > 0) {
        
        // Ottieni il simbolo appropriato per il satellite
        String symbol = getSatelliteSymbol(gpsData.satellites_info[i]);
        
        // Formatta con padding per allineamento
        String line = symbol + gpsData.satellites_info[i].constellation.substring(1); // Rimuovi il $ originale
        line += " ";
        
        // PRN con padding a 3 cifre
        if (gpsData.satellites_info[i].prn < 10) line += "  ";
        else if (gpsData.satellites_info[i].prn < 100) line += " ";
        line += String(gpsData.satellites_info[i].prn);
        line += " ";
        
        // SNR con padding a 2 cifre
        if (gpsData.satellites_info[i].snr < 10) line += " ";
        line += String(gpsData.satellites_info[i].snr);
        line += " ";
        
        // Azimuth con padding a 3 cifre
        if (gpsData.satellites_info[i].azimuth < 10) line += "  ";
        else if (gpsData.satellites_info[i].azimuth < 100) line += " ";
        line += String(gpsData.satellites_info[i].azimuth);
        line += " ";
        
        // Elevation con padding a 2 cifre
        if (gpsData.satellites_info[i].elevation < 10) line += " ";
        line += String(gpsData.satellites_info[i].elevation);
        
        // Assicurati che la linea sia esattamente 20 caratteri
        while (line.length() < 20) line += " ";
        if (line.length() > 20) line = line.substring(0, 20);
        
        lcd.print(line);
      } else {
        lcd.print("                    ");
      }
    }
  } else {
    // Se non ci sono satelliti, mostra cursore lampeggiante
    if (millis() - lastCursorBlink >= cursorBlinkInterval) {
      cursorVisible = !cursorVisible;
      lastCursorBlink = millis();
    }
    
    // Pulisci la pagina
    for (int i = 0; i < 4; i++) {
      lcd.setCursor(0, i);
      lcd.print("                    ");
    }
    
    // Mostra il cursore lampeggiante
    if (cursorVisible) {
      lcd.setCursor(0, 0);
      lcd.write(0xFF);
    }
  }
}

void displayPage4() {
  // Usa sempre i dati GPS se disponibili, altrimenti RTC
  String displayDate = gpsData.date.length() > 0 ? gpsData.date : gpsData.rtcDate;
  String displayTime = gpsData.utcTime.length() > 0 ? gpsData.utcTime.substring(0, 8) : gpsData.rtcTime;
  String displayLocal = gpsData.localTime.length() > 0 ? gpsData.localTime.substring(0, 8) : gpsData.rtcTime;
  
  lcd.setCursor(0, 0);
  lcd.print("DATE: ");
  lcd.setCursor(6, 0);
  lcd.print("              "); // Clear resto riga
  lcd.setCursor(6, 0);
  lcd.print(displayDate);
  
  lcd.setCursor(0, 1);
  lcd.print("S-TIME: ");
  lcd.setCursor(8, 1);
  lcd.print("            "); // Clear resto riga
  lcd.setCursor(8, 1);
  lcd.print(displayTime);
  
  lcd.setCursor(0, 2);
  lcd.print("L-TIME: ");
  lcd.setCursor(8, 2);
  lcd.print("            "); // Clear resto riga
  lcd.setCursor(8, 2);
  lcd.print(displayLocal);
  
  lcd.setCursor(0, 3);
  if (gpsData.sbasSat.prn > 0) {
    lcd.print("SBAS: ");
    lcd.print(gpsData.sbasSat.prn);
    lcd.print(" ");
    lcd.print(gpsData.sbasSat.snr);
    lcd.print(" ");
    lcd.print(gpsData.sbasSat.azimuth);
    lcd.print(" ");
    lcd.print(gpsData.sbasSat.elevation);
    lcd.print("   "); // Clear resto
  } else {
    lcd.print("SBAS: NO DATA       ");
  }
}

// Fixed displayPage5 function with better formatting
void displayPage5() {
  lcd.setCursor(0, 0);
  lcd.print("MODULEVALIDITY/NFO");
  
  lcd.setCursor(0, 1);
  lcd.print("====================");
  
  lcd.setCursor(0, 2);
  lcd.print("MVR: ");
  if (gpsData.magneticVariation == 0.0) {
    lcd.print("N/A    ");
  } else {
    lcd.print(String(gpsData.magneticVariation, 1));
    lcd.print("^");
    // Clear any remaining characters
    lcd.print("   ");
  }
  
  lcd.setCursor(0, 3);
  lcd.print("VALIDITY: ");
  lcd.print(gpsData.validity);
  lcd.print(" WARN: ");
  lcd.print(gpsData.warning);
}

// ==== FUNZIONE displayPage6 CORRETTA ====
void displayPage6() {
  if (!gpsData.rfDataAvailable || gpsData.numRFBlocks == 0) {
    lcd.setCursor(0, 0);
    lcd.print("RF MON: NO DATA     ");
    lcd.setCursor(0, 1);
    lcd.print("Waiting for UBX...  ");
    lcd.setCursor(0, 2);
    lcd.print("MON-RF messages     ");
    lcd.setCursor(0, 3);
    lcd.print("                    ");
    return;
  }
  
  // Usa il primo blocco RF per il display
  auto& rf = gpsData.rfBlocks[0];
  
  // Riga 0: Stato Jamming
  lcd.setCursor(0, 0);
  lcd.print("JM-ANLSR: ");
  switch (rf.jammingState) {
    case 0: lcd.print("UNKNOWN     "); break;
    case 1: lcd.print("NO INTERF "); break;
    case 2: lcd.print("WARNING     "); break;
    case 3: lcd.print("CRITICAL    "); break;
    default: lcd.print("ERROR       "); break;
  }
  
  // Riga 1: CW Suppression e AGC
  lcd.setCursor(0, 1);
  lcd.print("CW:");
  
  // CW come percentuale (0-255 -> 0-100%)
  float cwPercent = (rf.cwSuppression * 100.0) / 255.0;
  if (cwPercent < 10.0) lcd.print(" ");
  lcd.print(String((int)cwPercent));
  lcd.print("% AGC:");
  
  // AGC come percentuale (0-8191 -> 0-100%)
  float agcPercent = (rf.agcCnt * 100.0) / 8191.0;
  if (agcPercent > 100.0) agcPercent = 100.0; // Cap al 100%
  if (agcPercent < 10.0) lcd.print(" ");
  lcd.print(String((int)agcPercent));
  lcd.print("%   ");
  
  // Riga 2: Stato Antenna
  lcd.setCursor(0, 2);
  lcd.print("ANT:");
  switch (rf.antStatus) {
    case 0: lcd.print("INIT  "); break;
    case 1: lcd.print("UNK   "); break;
    case 2: lcd.print("OK    "); break;
    case 3: lcd.print("SHORT "); break;
    case 4: lcd.print("OPEN  "); break;
    default: lcd.print("ERROR "); break;
  }
  
  // Potenza Antenna
  lcd.print("PWR:");
  switch (rf.antPower) {
    case 0: lcd.print("OFF  "); break;
    case 1: lcd.print("ON   "); break;
    case 2: lcd.print("UNK  "); break;
    default: lcd.print("ERR  "); break;
  }
  
  // Riga 3: Noise e Band
  lcd.setCursor(0, 3);
  lcd.print("NOISE:");
  
  // Noise value diretto (0-65535)
  if (rf.noisePerMS < 10) lcd.print("    ");
  else if (rf.noisePerMS < 100) lcd.print("   ");
  else if (rf.noisePerMS < 1000) lcd.print("  ");
  else if (rf.noisePerMS < 10000) lcd.print(" ");
  lcd.print(String(rf.noisePerMS));
  
  lcd.print(" B:");
  switch (rf.rfBlockGnssBand) {
    case 0: lcd.print("? "); break;   // Unknown
    case 1: lcd.print("L1"); break;   // L1 band
    case 2: lcd.print("L2"); break;   // L2 band  
    case 3: lcd.print("L3"); break;   // L3 band
    case 4: lcd.print("L5"); break;   // L5 band
    default: lcd.print("E "); break;  // Error
  }
  lcd.print(" ");
}

void displayPage7() {
  lcd.setCursor(0, 0);
  lcd.print("LSTN: ");
  lcd.print(gpsData.listenStatus);
  
  lcd.setCursor(0, 1);
  lcd.print("                    ");
  
  lcd.setCursor(0, 2);
  lcd.print("DOWNLINK:1575.42 MHz");
  
  lcd.setCursor(0, 3);
  lcd.print("LISTNG TO: L1C/A ");
}

void displayPage8() {
  lcd.setCursor(0, 0);
  lcd.print("AVCONST: SBAS:L1C/A ");
  
  lcd.setCursor(0, 1);
  lcd.print("GAL: E1  BDU: B1    ");
  
  lcd.setCursor(0, 2);
  lcd.print("GLN: L1  GPS: L1C/A ");
  
  lcd.setCursor(0, 3);
  lcd.print("QZS: L1C/A  n  L1S  ");
}


void displayPage9() {
  if (!ubxNavSatData.dataAvailable) {
    lcd.setCursor(0, 0);
    lcd.print("UBX-NAV-SAT         ");
    lcd.setCursor(0, 1);
    lcd.print("Waiting for data... ");
    lcd.setCursor(0, 2);
    lcd.print("                    ");
    lcd.setCursor(0, 3);
    lcd.print("                    ");
    return;
  }
  
  // Header
  lcd.setCursor(0, 0);
  lcd.print("  SV | Qi |ORS| HLT");
  
  // Visualizza 3 satelliti alla volta
  int startIdx = currentSatPage * 3;
  for (int i = 0; i < 3; i++) {
    int satIdx = startIdx + i;
    lcd.setCursor(0, i + 1);
    
    if (satIdx < ubxNavSatData.numSvs) {
      auto& sat = ubxNavSatData.satellites[satIdx];
      
      // SV (Constellation + PRN)
      String line = "";
      if (sat.svUsed) line += "$";
      else line += " ";
      line += getConstellationLetter(sat.gnssId);
      
      // PRN con padding
      if (sat.svId < 10) line += " ";
      line += String(sat.svId);
      line += "   ";
      
      // Quality Index
      line += String(sat.qualityInd);
      line += "   ";
      
      // Orbit Source
      line += getOrbitSourceString(sat.orbitSource);
      line += "  ";
      
      // Health
      line += String(sat.health);
      
      // Padding alla lunghezza corretta
      while (line.length() < 20) line += " ";
      if (line.length() > 20) line = line.substring(0, 20);
      
      lcd.print(line);
    } else {
      lcd.print("                    ");
    }
  }
  
  // Auto-scroll tra le pagine di satelliti ogni 3 secondi
  static unsigned long lastScroll = 0;
  if (millis() - lastScroll > 3000) {
    currentSatPage++;
    if (currentSatPage * 3 >= ubxNavSatData.numSvs) {
      currentSatPage = 0;
    }
    lastScroll = millis();
  }
}

void displayPage10() {
  lcd.setCursor(0, 0);
  lcd.print("CLKD: ");
  lcd.print(gpsData.rtcDate);
  lcd.print("      "); // Clear resto riga
  
  lcd.setCursor(0, 1);
  lcd.print("CLKT: ");
  lcd.print(gpsData.rtcTime);
  lcd.print("       "); // Clear resto riga
  

  
  lcd.setCursor(0, 2);
  
  lcd.print(String(gpsData.rtcTemperature, 1) + "K " + gpsData.errorRange);
  
  // TFST - Time of First Store (ore dall'ultimo aggiornamento RTC)
  unsigned long hoursSinceUpdate = (millis() - gpsData.lastRTCUpdate) / 3600000;
  lcd.setCursor(0, 3);
  lcd.print("TFST:");
  lcd.setCursor(5, 3);
  lcd.print(String(hoursSinceUpdate) + "h    "); // Clear resto
  
  // Debug: stampa stato RTC su seriale
  Serial.println("Pagina 9 - RTC Date: " + gpsData.rtcDate + " Time: " + gpsData.rtcTime + " Temp: " + String(gpsData.rtcTemperature));
}

// Funzione per visualizzare la pagina PRN
void displayPage11() {
  // Genera nuovo PRN ogni 10 secondi
  if (millis() - lastPrnGeneration > 10000 || currentPRN.length() == 0) {
    generatePRN();
    lastPrnGeneration = millis();
    prnScrollPosition = 0; // Reset scroll quando si genera nuovo PRN
  }
  
  // Header
  lcd.setCursor(0, 0);
  lcd.print("LOCAL-PRNG:");
  
  // Mostra PRN su 3 righe con scorrimento
  for (int row = 0; row < 3; row++) {
    lcd.setCursor(0, row + 1);
    
    if (currentPRN.length() >= 16) {
      // Calcola quale parte del PRN mostrare in questa riga
      int startPos = (prnScrollPosition + row * 5) % 16;
      String displayLine = "";
      
      // Estrai 5-6 cifre per questa riga
      int charsToShow = (row < 2) ? 5 : 16; // Ultima riga ha 6 cifre
      
      for (int i = 0; i < charsToShow; i++) {
        int pos = (startPos + i) % 16;
        displayLine += currentPRN.charAt(pos);
      }
      
      // Padding per centrare
      String paddedLine = "";
      int padding = (20 - displayLine.length()) / 2;
      for (int i = 0; i < padding; i++) paddedLine += " ";
      paddedLine += displayLine;
      while (paddedLine.length() < 20) paddedLine += " ";
      
      lcd.print(paddedLine);
    } else {
      lcd.print("Generating PRN...   ");
    }
  }
  
  // Scorrimento ogni 2 secondi
  if (millis() - lastPrnScroll > 2000) {
    prnScrollPosition = (prnScrollPosition + 1) % 16;
    lastPrnScroll = millis();
  }
}
