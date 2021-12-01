#include "mbed.h"
#include "RGB_SENSOR//TCS3472_I2C.h"
#include "ACELEROMETER//MMA8451Q.h"
#include "TEMP_HUM/SI7021.h"
#include "GPS/MBed_Adafruit_GPS.h"


//Only for Information
#define ENABLE           0x00
#define TCS34725_CDATAL (0x14) 
#define TCS34725_CDATAH (0x15) 
#define TCS34725_RDATAL (0x16) 
#define TCS34725_RDATAH (0x17) 
#define TCS34725_GDATAL (0x18) 
#define TCS34725_GDATAH (0x19) 
#define TCS34725_BDATAL (0x1A) 
#define TCS34725_BDATAH (0x1B) 

//FOR PORTRAIT_LANDSCAPE
#define PORTRAIT_U 				32
#define ME 				  			208
#define LEFT 							16
#define RIGHT 						3
#define BACK							4
#define LOCKOUT 					0x40



//VARIABLES
AnalogIn soil_moisture(PA_0); 				// SOIL MOISTURE
AnalogIn light_sensor(PA_4);  				// LIGHT SENSOR
float volatile levellight = 0.0;
float volatile levelmoisture = 0.0;
float x,y,z;													// VARIABLE for Accelerometer
int rgb_readings[4]; 									// Declare a 4 element array to store RGB sensor readings
uint8_t readingTapAcel[2];
DigitalOut IndTestMode(LED1); 				// Indicator of Test Mode 
DigitalOut IndNormalMode(LED2);				// Indicator de Normal Mode
DigitalOut IndAdvancedMode(LED3);			// Indicador de Modo Avanzado
bool tickTwoSeconds;									// VARIABLE activated after two seconds in test mode, to show data
bool tickThirtySeconds;								// VARIABLE activated after 30 seconds in test mode, to show data
bool tickHour;												// VARIABLE activated after 30 seconds in normal mode, to show data
int contadorDosSegundos=0;						// Every time enters in the timer 1 sec, it gwows 1, so in 2 sec reach 2 
int contadorTreintaSegundos=0;				// Every time enters in the timer 1 sec, it gwows 1, so in 3 sec reach 30 
int contadorHora=0;

DigitalOut Rgb_Red(PH_0);
DigitalOut Rgb_Green(PH_1);
DigitalOut Rgb_Blue(PB_13);

//VARIABLES for the greater and lower values

float maxSoil = 0.0;
float minSoil = 0.0;

float maxLight = 0.0;
float minLight = 0.0;

float maxAcelX = 0.0;
float minAcelX = 0.0;

float maxAcelY = 0.0;
float minAcelY = 0.0;

float maxAcelZ = 0.0;
float minAcelZ = 0.0;

float maxTemp = 0.0;
float minTemp = 0.0;

float maxHum = 0.0;
float minHum = 0.0;

int contRed=0;
int contGreen=0;
int contBlue=0;

int indicadorTap=1; //FOR FLASH LED TAP



//VARIABLES FOR GPS
using namespace std::chrono;
UnbufferedSerial * gps_Serial;
bool gpsFound = false;

//MODE INDICATOR
int modo = 0; //0: START, 1: TEST, 2: NORMAL

//DigitalOut ControlLedRGBSensor(PB_7);


//TEMP-HUMITY

SI7021::SI7021_vector_data_t     myData;
SI7021::SI7021_status_t          mySI7021status;
uint32_t                         myState;


//INICIALIZACION DE I2C
TCS3472_I2C     rgb_sensor          (PB_9, PB_8);																									//I2C FOR RGB SENSOR
MMA8451Q        acc                 (PB_9,PB_8,0x1C<<1);																					//I2C FOR ACELEROMETER
SI7021          myTempRHsensor      ( PB_9,PB_8, SI7021::SI7021_ADDRESS, 400000 );								//I2C FOR TEMP_HUM SENSOR




//TEMP timer
Ticker tickerCountOneSec;

//INTERRUPTIONS

InterruptIn button(PB_2); //Or PB_2

InterruptIn tapAcelerometer(PA_8);
bool help = true;

//METHOD in order to swithc between modes
void startCountOneSeconds ( void )
{
		if(modo==1)
		{
			myState++;
			contadorDosSegundos++;
			
			if(contadorDosSegundos==2)
			{
				contadorDosSegundos=0;
				tickTwoSeconds =true;

			}
		}
		
		else if(modo==2)
			{
						myState++;
						contadorTreintaSegundos++;
						contadorHora++;
						if(contadorTreintaSegundos==5)//Normal Time
						{
							contadorTreintaSegundos=0;
							tickThirtySeconds=true;
						}
						if(contadorHora==10) //Hour time
						{
							contadorHora=0;
							tickHour=true;
						}
			}
}

void ChangeModo(void)
	{
		//HERE THE CODE FOR CHANGE
		if(modo==0) //Start mode
			{
				modo = 1; //READY FOR TEST MODE 
				IndTestMode = 1;
				IndNormalMode = 0;
				IndAdvancedMode = 0;
				
			}
			else if (modo==1) //TEST MODE
			{
				modo = 2; //GO TO NORMAL MODE 
				Rgb_Red=1;
				Rgb_Green=1;
				Rgb_Blue=1;
				IndTestMode = 0;
				IndNormalMode = 1;
				IndAdvancedMode = 0;


			}
			else if (modo==2) //NORMAL MODE 
			{
				modo = 3; //GO TO ADVANCED MODE 
				Rgb_Red=1;
				Rgb_Green=1;
				Rgb_Blue=1;
				IndTestMode = 0;
				IndNormalMode = 0;
				IndAdvancedMode = 1;
				

			}		
			
			else if (modo==3) //ADVANCED MODE
			{
				modo = 1; //GO TO NORMAL MODE 
				Rgb_Red=1;
				Rgb_Green=1;
				Rgb_Blue=1;
				IndTestMode = 1;
				IndNormalMode = 0;
				IndAdvancedMode = 0;
			}			
				
	}
	
	void isTapping(void)
		{
			if(indicadorTap==1){
				Rgb_Red=0;
				Rgb_Green=0;
				Rgb_Blue=0;
				indicadorTap=0;
			}
			else if(indicadorTap==0)
				{
				Rgb_Red=1;
				Rgb_Green=1;
				Rgb_Blue=1;
				indicadorTap=1;
			}
			
		}
	
	void comparar(int typeSensor, float valueread)
			{
				
				if(typeSensor==1) //COMPARATION SOIL
				{
					if(valueread > maxSoil)
					{
						maxSoil=valueread;
					}
					else if(minSoil == 0)
					{
						minSoil=valueread;
					}
					else if(valueread < minSoil)
					{
						minSoil = valueread;
					}
				}
				
				else if(typeSensor==2) //COMPARATION LIGHT
				{
					if(valueread > maxLight)
					{
						maxLight=valueread;
					}
					else if(minLight == 0)
					{
						minLight=valueread;
					}
					else if(valueread < minLight)
					{
						minLight = valueread;
					}
				}
				else if(typeSensor==3) //COMPARATION TEMPERATURE
				{
					if(valueread > maxTemp)
					{
						maxTemp=valueread;
					}
					else if(minTemp == 0)
					{
						minTemp=valueread;
					}
					else if(valueread < minTemp)
					{
						minTemp = valueread;
					}
				}
				else if(typeSensor==4) //COMPARATION HUMIDITY
				{
					if(valueread > maxHum)
					{
						maxHum=valueread;
					}
					else if(minHum == 0)
					{
						minHum=valueread;
					}
					else if(valueread < minHum)
					{
						minHum = valueread;
					}
				}
					else if(typeSensor==5) //COMPARATION ACC X
					{
						if(valueread > maxAcelX)
					{
						maxAcelX=valueread;
					}
					else if(minAcelX == 0)
					{
						minAcelX=valueread;
					}
					else if(valueread < minAcelX)
					{
						minAcelX = valueread;
					}
					
					
					}
					else if(typeSensor==6) //COMPARATION ACC Z
					{
						if(valueread > maxAcelY)
					{
						maxAcelY=valueread;
					}
					else if(minAcelY == 0)
					{
						minAcelY=valueread;
					}
					else if(valueread < minAcelY)
					{
						minAcelY = valueread;
					}
					
					}
					else if(typeSensor==7) //COMPARATION ACC Z
					{
							if(valueread > maxAcelZ)
						{
							maxAcelZ=valueread;
						}
						else if(minAcelZ == 0)
						{
							minAcelZ=valueread;
						}
						else if(valueread < minAcelZ)
						{
							minAcelZ = valueread;
						}
					}

			}

char compareColorSensor(int red, int green, int blue)
{
			if(red>green && red>blue)
			{
				return 'R';
			}
			else if(green>red && green>blue)
			{
				return 'G';
			}
			else if(blue>red && blue>green)
			{
				return 'B';
			}
			return 'N' ;
}	

void startTempHumModule(void)
{
		// Reset the device
	
    mySI7021status   =   myTempRHsensor.SI7021_SoftReset ();
    wait_us ( 1000000 );

    // Configure the device
    mySI7021status   =   myTempRHsensor.SI7021_Conf ( SI7021::SI7021_RESOLUTION_RH_12_TEMP_14, SI7021::SI7021_HTRE_DISABLED );

    // Get the Electronic Serial Number
    mySI7021status   =   myTempRHsensor.SI7021_GetElectronicSerialNumber ( &myData );

    // Get the Firmware revision
    mySI7021status   =   myTempRHsensor.SI7021_GetFirmwareRevision       ( &myData );

    // Plot all the information through the srial port.
   // printf( "Electronic Serial Number: %16x %16x\nFirmware Revision: %02x\r\n", myData.ElectronicSerialNumber_MSB, myData.ElectronicSerialNumber_LSB, myData.FirmwareRevision );

}

float calcPercentaje(int typeSensor, float valueRead)
{
	float result = 0.0;
	switch(typeSensor)
		{
		case 1: // SENSOR TYPE SOIL MOILSTURE
			if(valueRead>=0.60)
			{
				
				result = 100;
				if(modo==2)
					{
						Rgb_Red=0;
						Rgb_Blue=1;
						Rgb_Green=0;
						wait_us(250000);

					}
				
				return result;

			}
			else
			{
			
			result = ((valueRead * 100)/0.78); //0.78 MAX VALUE ANALOG READING
			break;

			}
		
		case 2: // SENSOR TYPE LIGHT
			if(valueRead>=0.90)
			{
				
				result = 100;
				if(modo==2)
				{
					Rgb_Red=0;
					Rgb_Blue=0;
					Rgb_Green=1;
					wait_us(250000);
				}
				
				return result;

			}
			else
				{
					result = ((valueRead * 100)/0.90); //0.98 MAX VALUE ANALOG LIGHT
					
					break;
				}
	}
		return result;
}




void showDataAcelerometer(void)
	{
							//PROCESS TO SHOW DATA OF ACCELEROMETER
						x=acc.getAccX();
						y=acc.getAccY();
						z=acc.getAccZ();
						//printf("WHO AM I: 0x%2X\r\n", acc.getWhoAmI());
						printf("ACELEROMETER: X_axis: %f \t Y_axis: %f\t Z_axis: %f \n",x,y,z);
						if(modo==2)
						{
							if(x>1 || y>1 || z>1)
							{
								Rgb_Green=0;
								Rgb_Blue=0;
								Rgb_Red=0;
								wait_us(250000);

							}						
							comparar(5, x);
							comparar(6, y);
							comparar(7, z);
						}
	}
	
	void showDataTemHum(void)
		{
			mySI7021status   =    myTempRHsensor.SI7021_ReadHumidity          ( &myData ); 									// Read Humidity and Temperature result
			mySI7021status   =    myTempRHsensor.SI7021_ReadTemperatureFromRH ( &myData );
			printf( "TEMP/HUM: Temperature: %0.2f C, Relative Humidity: %0.2f %% \r\n\n\n\n\n", myData.Temperature, myData.RelativeHumidity );
			myState   =  0;	// Reset state
			
			if(modo==2)
				{
					if(myData.RelativeHumidity>=50 || myData.Temperature>=20)	
					{
						Rgb_Green=0;
						Rgb_Blue=0;
						Rgb_Red=1;
						wait_us(250000);
					}
					comparar(3, myData.Temperature);
					comparar(4, myData.RelativeHumidity);

				}
				
			
		
		}
	void showDataColor(void)
		{
						//PROCESS TO SHOW COLOR
						rgb_sensor.getAllColors( rgb_readings );
						if(rgb_readings[0]>30000)
						{
							if(modo==1)
							{
								Rgb_Red=0;
								Rgb_Green=0;
								Rgb_Blue=0;
							}
							else if (modo==2)
										{
											Rgb_Red=1;
											Rgb_Green=1;
											Rgb_Blue=1;
										}
							
							printf("COLOR SENSOR: CLEAR! \n");

						}
						else
							{
								if(rgb_readings[1]<600 && rgb_readings[2]<600 && rgb_readings[3]<600) //If low: none
									{
										if(modo==1)
										{
											Rgb_Red=1;
											Rgb_Green=1;
											Rgb_Blue=1;
										}
										else if (modo==2)
										{
											Rgb_Red=1;
											Rgb_Green=1;
											Rgb_Blue=1;
										}
										printf("COLOR SENSOR: Clear: %d, Red: %d, Green: %d, Blue: %d    --Dominant color: NONE \n",
												rgb_readings[0],
												rgb_readings[1],
												rgb_readings[2],
												rgb_readings[3]);

									}
									else
										{
										if(rgb_readings[1]>rgb_readings[2] && rgb_readings[1]>=rgb_readings[3]) //If max=RED
											{
												printf("COLOR SENSOR: Clear: %d, Red: %d, Green: %d, Blue: %d    --Dominant color: RED \n",
												rgb_readings[0],
												rgb_readings[1],
												rgb_readings[2],
												rgb_readings[3]);
												if(modo==1)
												{
													Rgb_Red=0;
													Rgb_Green=1;
													Rgb_Blue=1;
												}
												
												if(modo==2)
												{
													contRed++;
												}
												
											}else if(rgb_readings[2]>rgb_readings[1] && rgb_readings[2]>rgb_readings[3]) //If max=Green
											{
												printf("COLOR SENSOR: Clear: %d, Red: %d, Green: %d, Blue: %d    --Dominant color: GREEN \n",
												rgb_readings[0],
												rgb_readings[1],
												rgb_readings[2],
												rgb_readings[3]);
												if(modo==1)
															{
																Rgb_Red=1;
																Rgb_Green=0;
																Rgb_Blue=1;
															}
												
												if(modo==2)
												{
													contGreen++;
												}
											}
											else if(rgb_readings[3]>rgb_readings[1] && rgb_readings[3]>rgb_readings[2])   //If max=Blue
											{
												printf("COLOR SENSOR: Clear: %d, Red: %d, Green: %d, Blue: %d    --Dominant color: BLUE \n",
												rgb_readings[0],
												rgb_readings[1],
												rgb_readings[2],
												rgb_readings[3]);
												if(modo==1)
												{
													Rgb_Red=1;
													Rgb_Green=1;
													Rgb_Blue=0;
												}
												
												
												if(modo==2)
												{
													contBlue++;
												}												
											}
										}
							}
			
			}
		
			
			


			//PROCESS SOIL MOISTURE
			void showDataSoilMoisture(void)
		{
			levelmoisture = soil_moisture.read();
			if(modo==2)
			{
				comparar(1,calcPercentaje(1, levelmoisture));
			}
			printf("SOIL MOISTURE: %0.2f %% \n", calcPercentaje(1, levelmoisture)); // 1 SOIL MOISTURE
		}
		
		
		//PROCESS LIGHT
		void showDataLight(void)
			{
				levellight = light_sensor.read();
				if(modo==2)
				{
					comparar(2,calcPercentaje(2, levellight));
				}
				printf("LIGHT: %0.2f %% \n", calcPercentaje(2, levellight));
			}
		
		
		
int main() 
	{
		//startTempHumModule(); 																						// START TEMP-HUM SENSOR
		rgb_sensor.enablePowerAndRGBC();																	// START RGB SENSOR
		//LEDS of the board
		IndTestMode = 0;
		IndNormalMode = 0;
		IndAdvancedMode = 0;
		
		
		//Switch mode
		button.mode(PullUp);
		button.fall(&ChangeModo);
		
		//DETECT TAP
		tapAcelerometer.mode(PullUp);
		tapAcelerometer.fall(&isTapping);
		
		//- TURN OFF WITH 1 - LED ANODO COMUN
		Rgb_Red=1;
		Rgb_Green=1;
		Rgb_Blue=1;
		
		
		//START GPS
		gps_Serial = new UnbufferedSerial(PA_9, PA_10,9600); //serial object for use w/ GPS
    Adafruit_GPS myGPS(gps_Serial); //object of Adafruit's GPS class
    char c; //when read via Adafruit_GPS::read(), the class returns single character stored here
    Timer refresh_Timer; //sets up a timer for use in loop; how often do we print GPS info?
    const int refresh_Time = 2000; //refresh time in ms

    myGPS.begin(9600);  //sets baud rate for GPS communication; note this may be changed via Adafruit_GPS::sendCommand(char *)
                        //a list of GPS commands is available at http://www.adafruit.com/datasheets/PMTK_A08.pdf

    myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGA); //these commands are defined in MBed_Adafruit_GPS.h; a link is provided there for command creation
    myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    myGPS.sendCommand(PGCMD_ANTENNA);
    ThisThread::sleep_for(1s);
    refresh_Timer.start();  //starts the clock on the timer
		
		//START COUNTER TEST MODE 
		tickerCountOneSec.attach( &startCountOneSeconds, 1000ms );       


		
		printf("INITIALIZED SERVICES. WAITING TO CHOOSE THE WORKING MODE. \n\n\n");
		while(true)
			{
				c = myGPS.read();   //queries the GPS
				//if (c) { printf("%c", c); } //this line will echo the GPS data if not paused
				//check if we recieved a new message from GPS, if so, attempt to parse it,
				if ( myGPS.newNMEAreceived() ) 
				{
					if ( !myGPS.parse(myGPS.lastNMEA())) 
					{
						continue;
					}
				}
				//check if enough time has passed to warrant printing GPS info to screen
				//note if refresh_Time is too low or pc.baud is too low, GPS data may be lost during printing
       if (duration_cast<milliseconds>(refresh_Timer.elapsed_time()).count() >= refresh_Time) 
				 {
						 //KEEP COMMENT // if (refresh_Timer.read_ms() >= refresh_Time) {
					   refresh_Timer.reset();

					 if ((int)myGPS.fixquality > 0) 
						 {

							 gpsFound = true;
						 }
						 else
						 {
							 gpsFound = false;
						 }
        }
				//************************************MODE TEST***********************************
				if ( myState == 1 ) 
						{
							// Trigger a new Humidity conversion ( the temperature conversion is triggered by default )
							mySI7021status   =    myTempRHsensor.SI7021_TriggerHumidity       ( SI7021::SI7021_HOLD_MASTER_MODE );

						} 
						else if ( myState == 2 ) 
						{

						}
				if(tickTwoSeconds && modo==1)   
					{
						//SOIL MOISTURE
						showDataSoilMoisture();
						
						//LIGHT SENSOR
						showDataLight();
						
						
						//GPS
						if(gpsFound == true)
							{
							
							 printf
							("GPS: #Sats: %d, Lat(UTC): %5.2f %c, Long(UTC): %5.2f %c, Altitude: %5.2f m, Date: %d/%d/20%d,  GPS_time: %d:%d:%d \n",
								 myGPS.satellites,
								 myGPS.latitude/100, myGPS.lat, myGPS.longitude/100, myGPS.lon,
								 myGPS.altitude,
								 myGPS.day, myGPS.month, myGPS.year,
								 myGPS.hour +1, myGPS.minute, myGPS.seconds
								 );
							}
							else if (gpsFound == false)
							{
								printf("GPS: Searching for a Gps Satellite \n");
							
							}
							
							//COLOR SENSOR
							showDataColor();

							//ACELEROMETER
							showDataAcelerometer();
						
							//TEM/HUM
							showDataTemHum();
							
						//RESET COUNT
						tickTwoSeconds = false;					

					 }
					 //************************************MODE NORMAL***********************************
					 else if(tickThirtySeconds && modo==2)
						 {
							 	//SOIL MOISTURE
							 showDataSoilMoisture();
						
								//LIGHT SENSOR
								showDataLight();
							 
							 if(gpsFound == true)
							{
							
							 printf
							("GPS: #Sats: %d, Lat(UTC): %5.2f %c, Long(UTC): %5.2f %c, Altitude: %5.2f m, Date: %d/%d/20%d,  GPS_time: %d:%d:%d \n",
								 myGPS.satellites,
								 myGPS.latitude/100, myGPS.lat, myGPS.longitude/100, myGPS.lon,
								 myGPS.altitude,
								 myGPS.day, myGPS.month, myGPS.year,
								 myGPS.hour + 1 , myGPS.minute, myGPS.seconds // +1 FROM UTC TO LOCAL TIME
								 );
							}
							else if (gpsFound == false)
							{
								printf("GPS: Searching for a Gps Satellite \n");
							
							}
							 
								//COLOR SENSOR
								showDataColor();
							 
								//ACELEROMETER
								showDataAcelerometer(); 
							
								//TEM/HUM
								showDataTemHum();
    
							 
							//RESET COUNT
							tickThirtySeconds=false;
							if(tickHour)
							{
								printf("MINIMUM AND MAXIMUM VALUES REPORT \n\n");
								printf("SOIL MOISTUE: Max: %f, Min: %f \n", maxSoil, minSoil);
								printf("LIGHT: Max: %f, Min: %f \n", maxLight, minLight);
								if(compareColorSensor(contRed, contGreen, contBlue)=='R')
									{
																	printf("COLOR SENSOR: Dominant: Red \n" );
									}
									else if(compareColorSensor(contRed, contGreen, contBlue)=='G')
									{
																	printf("COLOR SENSOR: Dominant: Green \n" );
									}
									else if(compareColorSensor(contRed, contGreen, contBlue)=='B')
									{
																	printf("COLOR SENSOR: Dominant: Blue \n" );
									}
									else
										{
											printf("COLOR SENSOR: Dominant: None \n" );
										}
								printf("ACELEROMETER: Max_X: %f, Min_X: %f, Max_Y: %f, Min_Y: %f, Max_Z: %f, Min_Z: %f \n",
								maxAcelX, minAcelX,
								maxAcelY, minAcelY,
								maxAcelZ, minAcelZ
								);								
								printf("TEMPERATURA: Max: %f, Min: %f \n", maxTemp, minTemp);
								printf("HUMEDAD: Max: %f, Min: %f \n\n\n", maxHum, minHum);
								
								//RESET COUNT
								tickHour=false;
							}
						 }	
						 else if(modo==3)
							 {
								 
								 //printf("WELCOME TO ADVANCED MODE! \n");

								 //printf("TAP: %f\n\n", acc.readTap());
								 if (help==true)
									 {
										 acc.setupTap(0x20, 0x20, 0x40);
										 help=false;
									 }

								 }
						
			}
	}			