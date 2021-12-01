#include "mbed.h"
#include "GPS.h"
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);
GPS::GPS(PinName serialTX, PinName serialRX) :
    UnbufferedSerial(serialTX, serialRX)
{
    attach(callback(this, &GPS::readData));
    rxTimer.start();
}
 
void GPS::readData()
{
    char gps_byte;
    volatile int step;
    volatile int array_title;
    rxTimer.reset();
    read(&gps_byte, 1);
        if(gpsRxLen < PACKET_SIZE) {
        if (gps_byte == '$') {
            gpsRxLen = 0; //'$' begin packet symbol
            step = 0;
            parsed = false;
				//	led1=1;
        }
        switch(step){
            case 0 :
                step++; 
                break;
            case 1 :
                if (gps_byte == 'G') 
                    step++;
                break;
            case 2 :
                if (gps_byte == 'P') {
                    step++;
                }
                break;
            case 3 :
                msg_type = gps_byte<<16;
                step++;
                break;
            case 4 :
                msg_type = msg_type + (gps_byte<<8);
                step++;
                break;
            case 5 :
                msg_type = msg_type + gps_byte;
                if (msg_type == 4345668) {     //BOD
                    array_title = 0;
                    updates += 0x01;
                    step++;
														led2=1;

                }
                else if (msg_type == 4347715) {     //BWC
                    array_title = 1;
                    updates += 0x02;
                    step++;
																							led3=1;

                }
                else if (msg_type == 4671297) {     //GGA
                    array_title = 2;
                    updates += 0x04;
                    step++;
																							led4=1;

                }
                else if (msg_type == 4672588) {     //GLL
                    array_title = 3;
                    updates += 0x08;
                    step++;
                }
                else if (msg_type == 4674369) {     //GSA
                    array_title = 4;
                    updates += 0x10;
                    step++;
                }
                else if (msg_type == 4674390) {     //GSV
                    array_title = 5;
                    updates += 0x20;
                    step++;
                }
                else if (msg_type == 4736084) {     //HDT
                    array_title = 6;
                    updates += 0x40;
                    step++;
                }
                else if (msg_type == 5386288) {     //R00
                    array_title = 7;
                    updates += 0x80;
                    step++;
                }
                else if (msg_type == 5393729) {     //RMA
                    array_title = 8;
                    updates += 0x100;
                    step++;
                }
                else if (msg_type == 5393730) {     //RMB
                    array_title = 9;
                    updates += 0x200;
                    step++;
                }
                else if (msg_type == 5393731) {     //RMC
                    array_title = 10;
                    updates = updates | 0x400;
                    step++;
                }
                else if (msg_type == 5395525) {     //RTE
                    array_title = 11;
                    updates += 0x800;
                    step++;
                }
                else if (msg_type == 5526086) {     //RRF
                    array_title = 12;
                    updates += 0x1000;
                    step++;
                }
                else if (msg_type == 5461070) {     //STN
                    array_title = 13;
                    updates += 0x2000;
                    step++;
                }
                else if (msg_type == 5653079) {     //VBW
                    array_title = 14;
                    updates += 0x4000;
                    step++;
                }
                else if (msg_type == 5657671) {     //VTG
                    array_title = 15;
                    updates += 0x8000;
                    step++;
                }
                else if (msg_type == 5722188) {     //WPL
                    array_title = 16;
                    updates += 0x10000;
                    step++;
                }
                else if (msg_type == 5788741) {     //XTE
                    array_title = 17;
                    updates += 0x20000;
                    step++;
                }
                else if (msg_type == 5915713) {     //ZDA
                    array_title = 18;
                    updates += 0x40000;
                    step++;
                }
                else uart_error = true;
                break;
            case 6 :
                switch (array_title) {
                    case 0 :
                        GPBOD[gpsRxLen] = gps_byte;
                        gpsRxLen ++;
                        break;
                    case 1 :
                        GPBWC[gpsRxLen] = gps_byte;
                        gpsRxLen ++;
                        break;
                    case 2 :
                        GPGGA[gpsRxLen] = gps_byte;
                        gpsRxLen ++;
                        break;
                    case 3 :
                        GPGLL[gpsRxLen] = gps_byte;
                        gpsRxLen ++;
                        break;
                    case 4 :
                        GPGSA[gpsRxLen] = gps_byte;
                        gpsRxLen ++;
                        break;
                    case 5 :
                        GPGSV[gpsRxLen] = gps_byte;
                        gpsRxLen ++;
                        break;
                    case 6 :
                        GPHDT[gpsRxLen] = gps_byte;
                        gpsRxLen ++;
                        break;
                    case 7 :
                        GPR00[gpsRxLen] = gps_byte;
                        gpsRxLen ++;
                        break;
                    case 8 :
                        GPRMA[gpsRxLen] = gps_byte;
                        gpsRxLen ++;
                        break;
                    case 9 :
                        GPRMB[gpsRxLen] = gps_byte;
                        gpsRxLen ++;
                        break;
                    case 10 :
                        GPRMC[gpsRxLen] = gps_byte;
                        gpsRxLen ++;
                        break;
                    case 11 :
                        GPRTE[gpsRxLen] = gps_byte;
                        gpsRxLen ++;
                        break;
                    case 12 :
                        GPTRF[gpsRxLen] = gps_byte;
                        gpsRxLen ++;
                        break;
                    case 13 :
                        GPSTN[gpsRxLen] = gps_byte;
                        gpsRxLen ++;
                        break;
                    case 14 :
                        GPVBW[gpsRxLen] = gps_byte;
                        gpsRxLen ++;
                        break;
                    case 15 :
                        GPVTG[gpsRxLen] = gps_byte;
                        gpsRxLen ++;
                        break;
                    case 16 :
                        GPWPL[gpsRxLen] = gps_byte;
                        gpsRxLen ++;
                        break;
                    case 17 :
                        GPXTE[gpsRxLen] = gps_byte;
                        gpsRxLen ++;
                        break;
                    case 18 :
                        GPZDA[gpsRxLen] = gps_byte;
                        gpsRxLen ++;
                        break;
                    }
                break;
            }
            
        }
        //parsed = false;
}
 
bool GPS::updateCheck()
{
	                sscanf(GPGGA, ",%f,%f,%c,%f,%c,%d,%d,%f,%f,%c,%f", &time, &latitude, &ns, &longitude, &ew, &lock, &sats, &hdop, &alt, &unit, &geoid);

    if (!parsed) {
        if (rxTimer.elapsed_time().count() > RX_MAXTIME) {
            gpsRxLen = 0;
            parsed = true;
            if ((updates & 0x01) > 0) {
                //printf("GPBOD ");
            }
            if ((updates & 0x02) > 0) {
                //printf("GPBWC ");
            }
            if ((updates & 0x04) > 0) {
                sscanf(GPGGA, ",%f,%f,%c,%f,%c,%d,%d,%f,%f,%c,%f", &time, &latitude, &ns, &longitude, &ew, &lock, &sats, &hdop, &alt, &unit, &geoid);
                int_time = static_cast<int>(time);
								led1=1;
                printf("GPGGA ");
            }
            if ((updates & 0x08) > 0) {
                //printf("GPGLL ");
            }
            if ((updates & 0x10) > 0) {
                //printf("GPGSA ");
            }
            if ((updates & 0x20) > 0) {
                //printf("GPGSV ");
            }
            if ((updates & 0x40) > 0) {
                //printf("GPHDT ");
            }
            if ((updates & 0x80) > 0) {
                //printf("GPR00 ");
            }
            if ((updates & 0x100) > 0) {
                //printf("GPRMA ");
            }
            if ((updates & 0x200) > 0) {
                //printf("GPRMB ");
            }
            if ((updates & 0x400) > 0) {
                //printf("GPRMC ");
            }
            if ((updates & 0x800) > 0) {
                //printf("GPRTE ");
            }
            if ((updates & 0x1000) > 0) {
                //printf("GPRTF ");
            }
            if ((updates & 0x2000) > 0) {
                //printf("GPSTN ");
            }
            if ((updates & 0x4000) > 0) {
                //printf("GPVBW ");
            }
            if ((updates & 0x8000) > 0) {
                //printf("GPVTG ");
                sscanf(GPVTG, ",%f,%c,,%c,%f,%c,%f,%c,%c", &track, &T, &M, &speedN, &N, &speedM, &K, &mode);
            }
            if ((updates & 0x10000) > 0) {
                //printf("GPWPL ");
            }
            if ((updates & 0x20000) > 0) {
                //printf("GPXTE ");
            }
            if ((updates & 0x40000) > 0) {
                //printf("GPZDA ");
            }
            updates = 0;
            uart_error = false;
            return true;
        }
        else 
					{
            return false;
					}
    }
    else 
			{
        return false;
			}

}
 
int GPS::getTime()
{
    return int_time;
}

float GPS::getLat()
{
    return latitude;
}

float GPS::getLong()
{
    return longitude;
}

int GPS::getSats()
{
    return sats;
}

float GPS::getHDOP()
{
    return hdop;
}

float GPS::getSpeed()
{
    return speedM;
}