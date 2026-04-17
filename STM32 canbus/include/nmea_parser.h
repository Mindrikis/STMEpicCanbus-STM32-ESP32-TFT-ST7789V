/*
  NMEA Parser for GPS modules
  Parses GPRMC and GPGGA sentences
*/

#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

#include <stdint.h>
#include <stdbool.h>

struct GPSData {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint8_t days;
    uint8_t months;
    uint8_t years;
    uint8_t quality;
    uint8_t satellites;
    float accuracy;
    float altitude;
    float course;
    float latitude;
    float longitude;
    float speed;
    bool hasFix;
    bool dataValid;
};

#define NMEA_SENTENCE_MAX_LEN 82
extern char nmeaBuffer[NMEA_SENTENCE_MAX_LEN + 1];
extern uint8_t nmeaBufferIndex;

void nmeaParserInit();
bool nmeaParserProcessChar(char c, struct GPSData* gpsData);
const char* nmeaGetField(const char* sentence, uint8_t fieldIndex);
bool nmeaVerifyChecksum(const char* sentence);

#endif
