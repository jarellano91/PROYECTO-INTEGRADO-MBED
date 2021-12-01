#include "mbed.h"
#ifndef GPS_H
#define GPS_H

#define RX_MAXTIME   10000LL  // Maximum time gap between two serial bytes
#define PACKET_SIZE 80
 
class GPS : public UnbufferedSerial {
    public :
        GPS(PinName serialTX, PinName serialRX);
        int getTime();
        float getLat();
        float getLong();
        int getSats();
        float getHDOP();
        float getSpeed();
        bool updateCheck();
        void readData();
        
    private :

        Timer rxTimer;
        volatile int gpsRxLen = 0;
        int updates;                //the 19 message types to be masked
        bool parsed = false;
        bool uart_error = false;
        int msg_type;
        char GPBOD[PACKET_SIZE];
        char GPBWC[PACKET_SIZE];
        char GPGGA[PACKET_SIZE];
        char GPGLL[PACKET_SIZE];
        char GPGSA[PACKET_SIZE];
        char GPGSV[PACKET_SIZE];
        char GPHDT[PACKET_SIZE];
        char GPR00[PACKET_SIZE];
        char GPRMA[PACKET_SIZE];
        char GPRMB[PACKET_SIZE];
        char GPRMC[PACKET_SIZE];
        char GPRTE[PACKET_SIZE];
        char GPTRF[PACKET_SIZE];
        char GPSTN[PACKET_SIZE];
        char GPVBW[PACKET_SIZE];
        char GPVTG[PACKET_SIZE];
        char GPWPL[PACKET_SIZE];
        char GPXTE[PACKET_SIZE];
        char GPZDA[PACKET_SIZE];

        float time, latitude, longitude, hdop, vdop, pdop, alt, geoid;
        float track, mag, speedN, speedM;
        char mode, T, M, N, K;
        char ns, ew, unit;
        int lock, sats, checksum;
        int int_time;
        
};
 
#endif