#include <Arduino.h>
#include "nmea_parser.h"
#include <stdlib.h>
#include <string.h>
#include <stddef.h>

char nmeaBuffer[NMEA_SENTENCE_MAX_LEN + 1];
uint8_t nmeaBufferIndex = 0;

void nmeaParserInit() {
    nmeaBufferIndex = 0;
    nmeaBuffer[0] = '\0';
}

static inline void parseTimeFromNMEAString(const char* timeStr, uint8_t* hours, uint8_t* minutes, uint8_t* seconds) {
    if (!timeStr || strlen(timeStr) < 6) {
        *hours = 0; *minutes = 0; *seconds = 0;
        return;
    }
    char hh[3] = {timeStr[0], timeStr[1], '\0'};
    char mm[3] = {timeStr[2], timeStr[3], '\0'};
    char ss[3] = {timeStr[4], timeStr[5], '\0'};
    *hours = (uint8_t)atoi(hh);
    *minutes = (uint8_t)atoi(mm);
    *seconds = (uint8_t)atoi(ss);
    if (*hours > 23) *hours = 0;
    if (*minutes > 59) *minutes = 0;
    if (*seconds > 59) *seconds = 0;
}

static inline void parseDateFromNMEAString(const char* dateStr, uint8_t* days, uint8_t* months, uint8_t* years) {
    if (!dateStr || strlen(dateStr) < 6) {
        *days = 1; *months = 1; *years = 25;
        return;
    }
    char dd[3] = {dateStr[0], dateStr[1], '\0'};
    char mm[3] = {dateStr[2], dateStr[3], '\0'};
    char yy[3] = {dateStr[4], dateStr[5], '\0'};
    *days = (uint8_t)atoi(dd);
    *months = (uint8_t)atoi(mm);
    *years = (uint8_t)atoi(yy);
    if (*days < 1 || *days > 31) *days = 1;
    if (*months < 1 || *months > 12) *months = 1;
    if (*years > 99) *years = 25;
}

static inline float parseLatitudeFromNMEA(const char* latStr, char ns) {
    if (!latStr || strlen(latStr) < 4) return 0.0f;
    char degStr[3] = {latStr[0], latStr[1], '\0'};
    int degrees = atoi(degStr);
    float minutes = atof(latStr + 2);
    float decimalDegrees = degrees + (minutes / 60.0f);
    if (ns == 'S' || ns == 's') decimalDegrees = -decimalDegrees;
    return decimalDegrees;
}

static inline float parseLongitudeFromNMEA(const char* lonStr, char ew) {
    if (!lonStr || strlen(lonStr) < 5) return 0.0f;
    char degStr[4] = {lonStr[0], lonStr[1], lonStr[2], '\0'};
    int degrees = atoi(degStr);
    float minutes = atof(lonStr + 3);
    float decimalDegrees = degrees + (minutes / 60.0f);
    if (ew == 'W' || ew == 'w') decimalDegrees = -decimalDegrees;
    return decimalDegrees;
}

const char* nmeaGetField(const char* sentence, uint8_t fieldIndex) {
    if (!sentence) return NULL;
    const char* p = sentence;
    uint8_t currentField = 0;
    if (*p == '$') p++;
    const char* fieldStart = p;
    while (*p) {
        if (*p == ',' || *p == '*') {
            if (currentField == fieldIndex) {
                if (fieldStart == p) return NULL;
                return fieldStart;
            }
            currentField++;
            if (*p == ',') {
                p++;
                fieldStart = p;
            } else {
                break;
            }
        } else {
            p++;
        }
    }
    if (currentField == fieldIndex && fieldStart < p) return fieldStart;
    return NULL;
}

static inline uint8_t calculateNMEAChecksum(const char* sentence) {
    if (!sentence || *sentence != '$') return 0;
    uint8_t checksum = 0;
    const char* p = sentence + 1;
    while (*p && *p != '*') checksum ^= *p++;
    return checksum;
}

bool nmeaVerifyChecksum(const char* sentence) {
    if (!sentence) return false;
    const char* star = strchr(sentence, '*');
    if (!star || strlen(star) < 3) return false;
    uint8_t calculated = calculateNMEAChecksum(sentence);
    uint8_t received = (uint8_t)strtoul(star + 1, NULL, 16);
    return (calculated == received);
}

static bool parseGPRMC(const char* sentence, struct GPSData* gpsData) {
    /* GP = GPS-only talker; GN = multi-constellation (common on u-blox 7 / GT-U7). */
    if (!sentence || (strncmp(sentence, "$GPRMC", 6) != 0 && strncmp(sentence, "$GNRMC", 6) != 0)) return false;
    if (!nmeaVerifyChecksum(sentence)) return false;

    const char* timeStr = nmeaGetField(sentence, 1);
    const char* statusStr = nmeaGetField(sentence, 2);
    const char* latStr = nmeaGetField(sentence, 3);
    const char* nsStr = nmeaGetField(sentence, 4);
    const char* lonStr = nmeaGetField(sentence, 5);
    const char* ewStr = nmeaGetField(sentence, 6);
    const char* speedStr = nmeaGetField(sentence, 7);
    const char* courseStr = nmeaGetField(sentence, 8);
    const char* dateStr = nmeaGetField(sentence, 9);

    if (!timeStr || !statusStr || !dateStr) return false;

    char timeBuf[16] = {0};
    const char* timeEnd = strchr(timeStr, ',');
    if (timeEnd) {
        size_t len = (size_t)(timeEnd - timeStr);
        if (len > 15) len = 15;
        strncpy(timeBuf, timeStr, len);
        timeBuf[len] = '\0';
    } else {
        strncpy(timeBuf, timeStr, 15);
        timeBuf[15] = '\0';
    }
    parseTimeFromNMEAString(timeBuf, &gpsData->hours, &gpsData->minutes, &gpsData->seconds);

    char dateBuf[7] = {0};
    const char* dateEnd = strchr(dateStr, ',');
    if (dateEnd) {
        size_t len = (size_t)(dateEnd - dateStr);
        if (len > 6) len = 6;
        strncpy(dateBuf, dateStr, len);
        dateBuf[len] = '\0';
    } else {
        strncpy(dateBuf, dateStr, 6);
        dateBuf[6] = '\0';
    }
    parseDateFromNMEAString(dateBuf, &gpsData->days, &gpsData->months, &gpsData->years);

    bool hasValidFix = (*statusStr == 'A' || *statusStr == 'a');
    gpsData->hasFix = hasValidFix;

    if (hasValidFix && latStr && nsStr && lonStr && ewStr) {
        char latBuf[16] = {0};
        const char* latEnd = strchr(latStr, ',');
        if (latEnd) {
            size_t len = (size_t)(latEnd - latStr);
            if (len > 15) len = 15;
            strncpy(latBuf, latStr, len);
            latBuf[len] = '\0';
        } else {
            strncpy(latBuf, latStr, 15);
            latBuf[15] = '\0';
        }
        gpsData->latitude = parseLatitudeFromNMEA(latBuf, *nsStr);

        char lonBuf[16] = {0};
        const char* lonEnd = strchr(lonStr, ',');
        if (lonEnd) {
            size_t len = (size_t)(lonEnd - lonStr);
            if (len > 15) len = 15;
            strncpy(lonBuf, lonStr, len);
            lonBuf[len] = '\0';
        } else {
            strncpy(lonBuf, lonStr, 15);
            lonBuf[15] = '\0';
        }
        gpsData->longitude = parseLongitudeFromNMEA(lonBuf, *ewStr);

        if (speedStr) {
            char speedBuf[16] = {0};
            const char* speedEnd = strchr(speedStr, ',');
            if (speedEnd) {
                size_t len = (size_t)(speedEnd - speedStr);
                if (len > 15) len = 15;
                strncpy(speedBuf, speedStr, len);
                speedBuf[len] = '\0';
            } else {
                strncpy(speedBuf, speedStr, 15);
                speedBuf[15] = '\0';
            }
            gpsData->speed = atof(speedBuf) * 1.852f;
        }

        if (courseStr) {
            char courseBuf[16] = {0};
            const char* courseEnd = strchr(courseStr, ',');
            if (courseEnd) {
                size_t len = (size_t)(courseEnd - courseStr);
                if (len > 15) len = 15;
                strncpy(courseBuf, courseStr, len);
                courseBuf[len] = '\0';
            } else {
                strncpy(courseBuf, courseStr, 15);
                courseBuf[15] = '\0';
            }
            gpsData->course = atof(courseBuf);
        }
    }

    gpsData->dataValid = true;
    return true;
}

static bool parseGPGGA(const char* sentence, struct GPSData* gpsData) {
    if (!sentence || (strncmp(sentence, "$GPGGA", 6) != 0 && strncmp(sentence, "$GNGGA", 6) != 0)) return false;
    if (!nmeaVerifyChecksum(sentence)) return false;

    const char* latStr = nmeaGetField(sentence, 2);
    const char* nsStr = nmeaGetField(sentence, 3);
    const char* lonStr = nmeaGetField(sentence, 4);
    const char* ewStr = nmeaGetField(sentence, 5);
    const char* qualityStr = nmeaGetField(sentence, 6);
    const char* numSatsStr = nmeaGetField(sentence, 7);
    const char* hdopStr = nmeaGetField(sentence, 8);
    const char* altitudeStr = nmeaGetField(sentence, 9);

    if (!qualityStr) return false;

    char qualityBuf[4] = {0};
    const char* qualityEnd = strchr(qualityStr, ',');
    if (qualityEnd) {
        size_t len = (size_t)(qualityEnd - qualityStr);
        if (len > 3) len = 3;
        strncpy(qualityBuf, qualityStr, len);
        qualityBuf[len] = '\0';
    } else {
        strncpy(qualityBuf, qualityStr, 3);
        qualityBuf[3] = '\0';
    }
    gpsData->quality = (uint8_t)atoi(qualityBuf);
    gpsData->hasFix = (gpsData->quality > 0);

    if (numSatsStr) {
        char satsBuf[4] = {0};
        const char* satsEnd = strchr(numSatsStr, ',');
        if (satsEnd) {
            size_t len = (size_t)(satsEnd - numSatsStr);
            if (len > 3) len = 3;
            strncpy(satsBuf, numSatsStr, len);
            satsBuf[len] = '\0';
        } else {
            strncpy(satsBuf, numSatsStr, 3);
            satsBuf[3] = '\0';
        }
        gpsData->satellites = (uint8_t)atoi(satsBuf);
    }

    if (gpsData->hasFix && latStr && nsStr && lonStr && ewStr &&
        (*nsStr == 'N' || *nsStr == 'S' || *nsStr == 'n' || *nsStr == 's') &&
        (*ewStr == 'E' || *ewStr == 'W' || *ewStr == 'e' || *ewStr == 'w')) {

        char latBuf[16] = {0};
        const char* latEnd = strchr(latStr, ',');
        if (latEnd) {
            size_t len = (size_t)(latEnd - latStr);
            if (len > 15) len = 15;
            strncpy(latBuf, latStr, len);
            latBuf[len] = '\0';
        } else {
            strncpy(latBuf, latStr, 15);
            latBuf[15] = '\0';
        }
        gpsData->latitude = parseLatitudeFromNMEA(latBuf, *nsStr);

        char lonBuf[16] = {0};
        const char* lonEnd = strchr(lonStr, ',');
        if (lonEnd) {
            size_t len = (size_t)(lonEnd - lonStr);
            if (len > 15) len = 15;
            strncpy(lonBuf, lonStr, len);
            lonBuf[len] = '\0';
        } else {
            strncpy(lonBuf, lonStr, 15);
            lonBuf[15] = '\0';
        }
        gpsData->longitude = parseLongitudeFromNMEA(lonBuf, *ewStr);
    }

    if (gpsData->hasFix && altitudeStr) {
        char altBuf[16] = {0};
        const char* altEnd = strchr(altitudeStr, ',');
        if (altEnd) {
            size_t len = (size_t)(altEnd - altitudeStr);
            if (len > 15) len = 15;
            strncpy(altBuf, altitudeStr, len);
            altBuf[len] = '\0';
        } else {
            strncpy(altBuf, altitudeStr, 15);
            altBuf[15] = '\0';
        }
        gpsData->altitude = atof(altBuf);
    }

    if (gpsData->hasFix && hdopStr) {
        char hdopBuf[16] = {0};
        const char* hdopEnd = strchr(hdopStr, ',');
        if (hdopEnd) {
            size_t len = (size_t)(hdopEnd - hdopStr);
            if (len > 15) len = 15;
            strncpy(hdopBuf, hdopStr, len);
            hdopBuf[len] = '\0';
        } else {
            strncpy(hdopBuf, hdopStr, 15);
            hdopBuf[15] = '\0';
        }
        gpsData->accuracy = atof(hdopBuf);
    }

    gpsData->dataValid = true;
    return true;
}

bool nmeaParserProcessChar(char c, struct GPSData* gpsData) {
    if (c == '$') {
        nmeaBufferIndex = 0;
        nmeaBuffer[0] = c;
        nmeaBufferIndex = 1;
        return false;
    } else if (c == '\r' || c == '\n') {
        if (nmeaBufferIndex > 0) {
            nmeaBuffer[nmeaBufferIndex] = '\0';
            bool parsed = false;
            if (strncmp(nmeaBuffer, "$GPRMC", 6) == 0 || strncmp(nmeaBuffer, "$GNRMC", 6) == 0) parsed = parseGPRMC(nmeaBuffer, gpsData);
            else if (strncmp(nmeaBuffer, "$GPGGA", 6) == 0 || strncmp(nmeaBuffer, "$GNGGA", 6) == 0) parsed = parseGPGGA(nmeaBuffer, gpsData);
            nmeaBufferIndex = 0;
            return parsed;
        }
        nmeaBufferIndex = 0;
        return false;
    } else if (nmeaBufferIndex < NMEA_SENTENCE_MAX_LEN) {
        nmeaBuffer[nmeaBufferIndex++] = c;
        return false;
    }
    nmeaBufferIndex = 0;
    return false;
}
