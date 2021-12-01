#include "MMA8451Q.h"
 
#define REG_WHO_AM_I      0x0D
#define REG_CTRL_REG_1    0x2A
#define REG_OUT_X_MSB     0x01
#define REG_OUT_Y_MSB     0x03
#define REG_OUT_Z_MSB     0x05
#define PULSE_CFG  				0x21
#define	PULSE_SRC  				0x22
#define	PULSE_THSX  			0x23
#define	PULSE_THSY  			0x24
#define	PULSE_THSZ  			0x25
#define	PULSE_TMLT  			0x26
#define	PULSE_LTCY  			0x27
#define	PULSE_WIND  			0x28
#define CTRL_REG4  				0x2D
#define CTRL_REG5 			 	0x2E

//FOR PORTRAIT AND LANDSCAPE
#define PL_STATUS  				0x10
#define PL_CFG  					0x11
#define PL_COUNT					0x12
#define PORTRAIT_U 				0
#define PORTRAIT_D 				1
#define LANDSCAPE_R 			2
#define LANDSCAPE_L 			3
#define LOCKOUT 					0x40







 
#define UINT14_MAX        16383
 
MMA8451Q::MMA8451Q(PinName sda, PinName scl, int addr) : m_i2c(sda, scl), m_addr(addr) {
    // activate the peripheral
    uint8_t data[2] = {REG_CTRL_REG_1, 0x01};
    writeRegs(data, 2);
}
 
MMA8451Q::~MMA8451Q() { }
 
uint8_t MMA8451Q::getWhoAmI() {
    uint8_t who_am_i = 0;
    readRegs(REG_WHO_AM_I, &who_am_i, 1);
    return who_am_i;
}
 
float MMA8451Q::getAccX() {
    return (float(getAccAxis(REG_OUT_X_MSB))/4096.0);
}
 
float MMA8451Q::getAccY() {
    return (float(getAccAxis(REG_OUT_Y_MSB))/4096.0);
}
 
float MMA8451Q::getAccZ() {
    return (float(getAccAxis(REG_OUT_Z_MSB))/4096.0);
}
 
void MMA8451Q::getAccAllAxis(float * res) {
    res[0] = getAccX();
    res[1] = getAccY();
    res[2] = getAccZ();
}
 
int16_t MMA8451Q::getAccAxis(uint8_t addr) {
    int16_t acc;
    uint8_t res[2];
    readRegs(addr, res, 2);
 
    acc = (res[0] << 6) | (res[1] >> 2);
    if (acc > UINT14_MAX/2)
        acc -= UINT14_MAX;
 
    return acc;
}


 
void MMA8451Q::readRegs(int addr, uint8_t * data, int len) {
    char t[1] = {(char)addr};
    m_i2c.write(m_addr, t, 1, true);
    m_i2c.read(m_addr, (char *)data, len);
}
 
void MMA8451Q::writeRegs(uint8_t * data, int len) {
    m_i2c.write(m_addr, (char *)data, len);
}

void MMA8451Q::writeRegsGeneral(uint8_t addr, uint8_t * data, int len) {
    m_i2c.write(addr, (char *)data, len);
}


void MMA8451Q::setupTap(uint8_t xThs, uint8_t yThs, uint8_t zThs)
{
	// Set up single and double tap - 5 steps:
	// for more info check out this app note:
	// http://cache.freescale.com/files/sensors/doc/app_note/AN4072.pdf
	// Set the threshold - minimum required acceleration to cause a tap.
	
	uint8_t temp = 0;
	
	uint8_t x_thresh[2] 		= {PULSE_THSX, xThs};
	uint8_t y_thresh[2] 		= {PULSE_THSY, yThs};
	uint8_t z_thresh[2] 		= {PULSE_THSZ, zThs};
	uint8_t interrupt[2]		= {CTRL_REG4, 0x08};	
	uint8_t interruptPin[2]	= {CTRL_REG5, 0x08};	
	

	
	temp |= 0x3; 					// Enable taps on x 	// DECIMAL 3
	temp |= 0xC; 					// Enable taps on y 	//DECIMAL 12
	temp |= 0x30; 				// Enable taps on z  	//DECIMAL 48
	temp |= 0x40;					// Set up single and/or double tap detection on each axis individually. // DECIMAL 64


	
	uint8_t single_double_tap[2]        = {PULSE_CFG,  0x3F}; //TEMP POR 0x15
	
	
	uint8_t maxTime[2]                  = {PULSE_TMLT, 0x18};
	uint8_t pulseLatency[2]             = {PULSE_LTCY, 0x28};
	uint8_t pulseWindows[2]             = {PULSE_WIND, 0x3C};
	uint8_t standbymode[2]							=	{REG_CTRL_REG_1, 0x10};
	uint8_t activeMode[2] = {REG_CTRL_REG_1, 0x01};
	
	
	writeRegs(standbymode,2);
	writeRegs(single_double_tap,2);
	writeRegs(x_thresh,2);
	writeRegs(y_thresh,2);
	writeRegs(z_thresh,2);
	
	writeRegs(maxTime,2);
	writeRegs(pulseLatency,2);
	writeRegs(pulseWindows,2);
		
	writeRegs(interrupt,2);
	writeRegs(interruptPin,2);
	
	writeRegs(activeMode, 2);



}

float MMA8451Q::readTap()
{
	uint8_t tapStat;
  uint8_t res[2];
  readRegs(PULSE_SRC, res, 2); //0x22
	tapStat = (res[0] << 6) | (res[1] >> 2);
	
	return (float(tapStat));

	/*
	if (tapStat & 0x80) // Read EA bit to check if a interrupt was generated
	{
		return (float(tapStat & 0x7F));
	}
	else
		return 0;*/
}

void MMA8451Q::setupPL()
{
	uint8_t standbymode[3]				=	{REG_CTRL_REG_1, 0x10, 0xFE};
	writeRegs(standbymode,3);
	
	uint8_t dataRate[3]						=	{REG_CTRL_REG_1, 0xC7, 0x20};
	writeRegs(dataRate,3);

	uint8_t enablePL[2]							=	{PL_CFG, 0x40};
	writeRegs(enablePL,2);
	
	uint8_t enable[2]				=	{REG_CTRL_REG_1, 0x01};
	writeRegs(enable,2);

	
	

}

uint8_t MMA8451Q::readPL()
{
	uint8_t plStat;
	uint8_t res[2];
	readRegs(PL_STATUS, res, 2); 
	plStat = (res[0] << 6) | (res[1] >> 2);
	return plStat;
	
	//plStat = res[0];

	/*
	if (plStat & 0x40) // Z-tilt lockout
	{
		return LOCKOUT;
	}
	else // Otherwise return LAPO status
	{
		return (plStat & 0x6) >> 1;
	}*/
}
