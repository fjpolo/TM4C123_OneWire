/*-------------------------------------------------------------------------------------------
 ********************************************************************************************
 *-------------------------------------------------------------------------------------------
 *
 *				DATA LOGGER DE VARIABLES AMBIENTALES INTERNAS
 *							CIMEC CONICET - UTN FRP
 *								     2016
 *
 *						Polo, Franco		fjpolo@frp.utn.edu.ar
 *						Burgos, Sergio		sergioburgos@frp.utn.edu.ar
 *						Bre, Facundo		facubre@cimec.santafe-conicet.gov.ar
 *
 *	datalogger.c v1.12
 *	20.09.2016
 *
 *	Descripción:
 *
 *   Funciones relacionadas con los sensores, protocolos de comunicacion y el
 *	modulo de hibernaci�n, asi como tambien interrupciones e incializacion.
 *
 *  Desarrollo del firmware de la placa base del data logger, constando de:
 *
 *  - Periféricos I2C:
 *  	a) HR y Tbs		HIH9131		0b0100111		0x27
 *  	b) Ev			TSL2563		0b0101001		0x29
 *  	c) Va			ADS			0b1001000		0x48
 *  	d) Tg			LM92		0b1001011		0x51
 *  	e) RTC			DS1703		0b1101000		0x68
 *
 *  - Periféricos OneWire@PD6
 *  	a) Ts01			MAX31850	ROM_Addr		0x3B184D8803DC4C8C
 *  	b) Ts02			MAX31850	ROM_Addr		0x3B0D4D8803DC4C3C
 *  	c) Ts03			MAX31850	ROM_Addr		0x3B4D4D8803DC4C49
 *  	d) Ts04			MAX31850	ROM_Addr		0x3B234D8803DC4C99
 *  	e) Ts05			MAX31850	ROM_Addr		0x3B374D8803DC4C1E
 *  	f) Ts06			MAX31850	ROM_Addr
 *
 *  - IHM
 *  	a) RESET		!RST
 *  	b) SW_SD		PC6
 *  	c) SW_ON		PC5
 *  	d) SW_1			PC7
 *  	e) WAKE			PF2
 *  	f) LEDON		PE0
 *  	g) LED1			PE1
 *  	h) LED2			PE2
 *
 *  - SD
 *  	a) SD_IN		PA6
 *  	b) SD_RX		PA4
 *  	c) SD_TX		PA5
 *  	d) SD_CLK		PA2
 *  	e) SD_FSS		PA3
 *
 *--------------------------------------------------------------------------------------------
 *********************************************************************************************
 *-------------------------------------------------------------------------------------------*/

/*********************************************************************************************
 * INCLUDES
 ********************************************************************************************/
// standard C
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
// inc
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_i2c.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ssi.h"
// driverlib
#include "driverlib/fpu.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/hibernate.h"
#include "driverlib/i2c.h"
#include "driverlib/ssi.h"
// datalogger
#include "datalogger/datalogger.h"
#include "datalogger/delay.h"
// third_party
#include "fatfs/src/ff.h"
#include "fatfs/src/diskio.h"
// onewire
#include "oneWire.h"


/*****************************************************************************************************
 * Defines
 ****************************************************************************************************/
// SWITCHES
#define SW_PORT 	GPIO_PORTC_BASE
#define SW_ON 		GPIO_PIN_5
#define SW_SD 		GPIO_PIN_6
#define SW_1 		GPIO_PIN_7
// LEDS
#define LED_PORT 	GPIO_PORTE_BASE
#define LED_ON 		GPIO_PIN_0
#define LED_1 		GPIO_PIN_1
#define LED_2 		GPIO_PIN_2
// SD Card
#define SD_PORT		GPIO_PORTA_BASE
#define SD_IN		GPIO_PIN_6
#define SD_RX		GPIO_PIN_4
#define SD_TX		GPIO_PIN_5
#define SD_CLK		GPIO_PIN_2
#define SD_FSS		GPIO_PIN_3
// Timer0
#define TOGGLE_FREQUENCY 1
// Tg
#define SLAVE_ADDRESS_TG 0x4B
#define TG_REG_READ 0x00
#define TG_REG_LOWPOW 0x01
// Ev
#define SLAVE_ADDRESS_EV 0x29
// RTC
#define SLAVE_ADDR_RTC 0x68
#define SEC 0x00
#define MIN 0x01
#define HRS 0x02
#define DAY 0x03
#define DATE 0x04
#define MONTH 0x05
#define YEAR 0x06
#define CNTRL 0x07
// Va ADC slave address
#define SLAVE_ADDR_VA 0x48
#define VA_REG_READ 0x00
// I2C3
#define GPIO_PD0_I2C3SCL        0x00030003
#define GPIO_PD1_I2C3SDA        0x00030403
// SSI1
#define SSI1_PORT				GPIO_PORTF_BASE
#define SSI1_RX					GPIO_PIN_0
#define SSI1_TX					GPIO_PIN_1
#define SSI1_CLK				GPIO_PIN_2
#define SSI1_CS					GPIO_PIN_3
// str2string
#define MAXDIGIT 10

/*********************************************************************************************
 * Macros
 * ******************************************************************************************/
#define Int2CharM(x) ((char)(x%10) + 48);

/*********************************************************************************************
 * Functions
 * ******************************************************************************************/
extern void Hibernate_IRQHandler (void);


/*********************************************************************************************
 * Global variables
 * ******************************************************************************************/
extern unsigned long int _MSDELAY_BASE;
extern unsigned long int _USDELAY_BASE;
unsigned long ulPeriod;

//Datos a mantener durante la hibernacion
extern unsigned long HibReg[12];
//0: sensor flag
//1: hibernate flag
//2: year
//3: month
//4: day
//5: hour
//6: time
typedef struct
{
	unsigned int data[MAXDIGIT];
	unsigned int p;
	unsigned int mx;
}stStack;
stStack stack = {{0}, 0, MAXDIGIT};
static double PRECISION = 0.001;
static int MAX_NUMBER_STRING_SIZE = 6;
extern char line[512];
//
oneWireCfg oneWirePort;
searchRomCfg allRoms;
unsigned char data[9];
//double Ts =0;
// TODO
uint16_t iTemp;
uint16_t Ts_sign;
uint16_t internalT, internalSign;
float internalDec;
double Ts_internal;
float Ts_dec = 0;
int j;



/*********************************************************************************************
 * Initialize
 * ******************************************************************************************/
void Initialize(void){
	FPULazyStackingEnable();
	InitClock();
	InitGPIO();
	//InitTimer0();
	InitGPIOInt();
	InitSDCard();
	//InitHibernation();
	InitI2C3();
	InitSPI1();
}

/*********************************************************************************************
 * InitClock
 * ******************************************************************************************/
void InitClock(void){
	// 400Mhz PLL
	// Half divider implicit
	// Sysdiv /5
	// Clock = 500/(5x2)= 40MHz
	//SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	//80MHZ
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	// Delay values
	_MSDELAY_BASE = (SysCtlClockGet()/3)/1000;
	_USDELAY_BASE = _MSDELAY_BASE/1000;

}
/*********************************************************************************************
 * InitGPIO
 * ******************************************************************************************/
void InitGPIO(void){
	//
	// IHM
	//
	//Entradas
	//Habilito el puerto C
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlDelay(10);
	//PC5,PC6PC7 como entradas
	GPIOPinTypeGPIOInput(SW_PORT, SW_ON|SW_SD|SW_1);
	//Enable pull-up resistor
	GPIOPadConfigSet(SW_PORT,SW_ON,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(SW_PORT,SW_SD,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(SW_PORT,SW_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	//
	//Salidas
	//Habilito el puerto E
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlDelay(10);
	//PE0,PE1 y PE2 salidas
	GPIOPinTypeGPIOOutput(LED_PORT, LED_ON|LED_1|LED_2);

}

/*********************************************************************************************
 * InitTimer0
 * ******************************************************************************************/
void InitTimer0(void){
	// Timer0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE,TIMER_CFG_A_PERIODIC);
	// Period
	ulPeriod=(SysCtlClockGet()/TOGGLE_FREQUENCY)/2;
	TimerLoadSet(TIMER0_BASE,TIMER_A,ulPeriod-1);
	// Enable interrupts
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
	IntMasterEnable();
}

/*********************************************************************************************
 * InitGPIOInt
 * ******************************************************************************************/
void InitGPIOInt(){
	//
	// Interrupt using GPIO
	//
	GPIOIntTypeSet(SW_PORT, SW_1, GPIO_RISING_EDGE );
	GPIOIntEnable(SW_PORT, SW_1);
	IntMasterEnable();
}

/*********************************************************************************************
 * InitHibernation
 * ******************************************************************************************/
void InitHibernation(void){
	// Enable Hibernate module
	SysCtlPeripheralEnable(SYSCTL_PERIPH_HIBERNATE);
	// Clock source
	HibernateEnableExpClk(SysCtlClockGet());
}

/*********************************************************************************************
 * InitOneWire
 * ******************************************************************************************/
void InitOneWire(int timer){
	//
	// Configuramos el pin, puerto y timer a utilizar
	// Puerto
	oneWirePort.periphGPIO = SYSCTL_PERIPH_GPIOD;
	oneWirePort.portGPIO = GPIO_PORTD_BASE;
	oneWirePort.pinGPIO = GPIO_PIN_6;
	oneWirePort.pinTimerId = timer;
}

/*********************************************************************************************
 * DataLoggingON
 * ******************************************************************************************/
void DataLoggingON(unsigned long *Status){
	// Hibernate clock
	HibernateEnableExpClk(SysCtlClockGet());
	//Retencion de estado de los pines
	HibernateGPIORetentionEnable();
	SysCtlDelay(SysCtlClockGet()/10);
	HibernateRTCEnable();
	//RTC
	HibernateRTCSet(0);
	// TODO: Set to 15 minutes - 900 seconds
	HibernateRTCMatchSet(0,5);
	//Condiciones de wake
	HibernateWakeSet(HIBERNATE_WAKE_PIN | HIBERNATE_WAKE_RTC);
	// Guardo info
	HibernateDataSet(HibReg, 12);
	//Interrupciones
	HibernateIntEnable(HIBERNATE_INT_RTC_MATCH_0 | HIBERNATE_INT_PIN_WAKE);
	// Clear any pending status.
	*Status = HibernateIntStatus(0);
	HibernateIntClear(*Status);
	//
	HibernateIntClear(HIBERNATE_INT_PIN_WAKE | HIBERNATE_INT_LOW_BAT | HIBERNATE_INT_RTC_MATCH_0);
	HibernateIntRegister(Hibernate_IRQHandler);
	//
	//Hibernamos
	//
	HibernateRequest();
}
/*********************************************************************************************
 * DataLoggingOFF
 * ******************************************************************************************/
void DataLoggingOFF(void){
	// Hibernation module disabled
	HibernateDisable();
}
/*********************************************************************************************
 * InitSDCard
 * ******************************************************************************************/
void InitSDCard(void){
	// Habilito el puerto A
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlDelay(10);
	// PA6
	GPIOPinTypeGPIOInput(SD_PORT, SD_IN);
	// Enable pull-up resistor
	GPIOPadConfigSet(SD_PORT, SD_IN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

}

/*********************************************************************************************
 * InitI2C3
 * ******************************************************************************************/
void InitI2C3(void){
	//
	// The I2C0 peripheral must be enabled before use.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
	//
	// For this example I2C3 is used with PortB[0:1].  The actual port and
	// pins used may be different on your part, consult the data sheet for
	// more information.  GPIO port D needs to be enabled so these pins can
	// be used.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	//
	// Configure the pin muxing for I2C3 functions on port D0 and D1.
	// This step is not necessary if your part does not support pin muxing.
	//
	GPIOPinConfigure(GPIO_PD0_I2C3SCL);
	GPIOPinConfigure(GPIO_PD1_I2C3SDA);
	//
	// Select the I2C function for these pins.  This function will also
	// configure the GPIO pins pins for I2C operation, setting them to
	// open-drain operation with weak pull-ups.  Consult the data sheet
	// to see which functions are allocated per pin.
	//
	GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
	GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);
	//
	// Enable and initialize the I2C3 master module.  Use the system clock for
	// the I2C0 module.  The last parameter sets the I2C data transfer rate.
	// If false the data rate is set to 100kbps and if true the data rate will
	// be set to 400kbps.  For this example we will use a data rate of 100kbps.
	//
	I2CMasterInitExpClk(I2C3_BASE, SysCtlClockGet(), false);

}

/*********************************************************************************************
 * InitSPI1
 * ******************************************************************************************/
void InitSPI1(void){
	// PF0 Rx
	// PF1 Tx Not used
	// PF2 Clk
	// PF3 Fss/CS
	//
	// Enable SSI1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlDelay(10);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
	SysCtlDelay(10);
	// Unlock PF0
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= GPIO_PIN_0;
	HWREG(GPIO_PORTF_BASE + GPIO_O_AFSEL) |= GPIO_PIN_0;
	HWREG(GPIO_PORTF_BASE + GPIO_O_DEN) |= GPIO_PIN_0;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
	//SSIDisable(SSI1_BASE);
	SysCtlDelay(10);
	// Set clock source
	SSIClockSourceSet(SSI1_BASE, SSI_CLOCK_SYSTEM);
	// GPIO config
	GPIOPinConfigure(GPIO_PF2_SSI1CLK);
	//GPIOPinConfigure(GPIO_PF3_SSI1FSS);
	GPIOPinConfigure(GPIO_PF0_SSI1RX);
	GPIOPinConfigure(GPIO_PF1_SSI1TX);
	GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2);
	SSIConfigSetExpClk(SSI1_BASE, 400000, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 1000, 16);
	// CS/FSS as GPIO
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
	// Enable SSI1
	SSIEnable(SSI1_BASE);
}

/*********************************************************************************************
 * I2CSend
 * ******************************************************************************************/
void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...){
	// Tell the master module what address will place on the bus when
	// communicating with the slave.
	I2CMasterSlaveAddrSet(I2C3_BASE, slave_addr, false);

	// Stores list of variable number of arguments
	va_list vargs;
	// Specifies the va_list to "open" and the last fixed argument
	//so vargs knows where to start looking
	va_start(vargs, num_of_args);

	// Put data to be sent into FIFO
	I2CMasterDataPut(I2C3_BASE, va_arg(vargs, uint32_t));
	// If there is only one argument, we only need to use the
	//single send I2C function
	if(num_of_args == 1)
	{
		//Initiate send of data from the MCU
		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);
		// Wait until MCU is done transferring.
		SysCtlDelay(500);
		while(I2CMasterBusy(I2C3_BASE));
		//"close" variable argument list
		va_end(vargs);
	}
	// Otherwise, we start transmission of multiple bytes on the
	//I2C bus
	else
	{
		// Initiate send of data from the MCU
		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);
		// Wait until MCU is done transferring.
		SysCtlDelay(500);
		while(I2CMasterBusy(I2C3_BASE));
		// Send num_of_args-2 pieces of data, using the
		//BURST_SEND_CONT command of the I2C module
		unsigned char i;
		for(i = 1; i < (num_of_args - 1); i++)
		{
			// Put next piece of data into I2C FIFO
			I2CMasterDataPut(I2C3_BASE, va_arg(vargs, uint32_t));
			// Send next data that was just placed into FIFO
			I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
			// Wait until MCU is done transferring.
			SysCtlDelay(500);
			while(I2CMasterBusy(I2C3_BASE));
		}
		// Put last piece of data into I2C FIFO
		I2CMasterDataPut(I2C3_BASE, va_arg(vargs, uint32_t));
		// Send next data that was just placed into FIFO
		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
		// Wait until MCU is done transferring.
		SysCtlDelay(500);
		while(I2CMasterBusy(I2C3_BASE));
		//"close" variable args list
		va_end(vargs);
	}
}

/*********************************************************************************************
 * I2CReceive
 * ******************************************************************************************/
uint32_t I2CReceive(uint32_t slave_addr, uint8_t reg){
	// Specify that we are writing (a register address) to the
	//slave device
	I2CMasterSlaveAddrSet(I2C3_BASE, slave_addr, false);
	//specify register to be read
	I2CMasterDataPut(I2C3_BASE, reg);
	//send control byte and register address byte to slave device
	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	//wait for MCU to finish transaction
	SysCtlDelay(500);
	while(I2CMasterBusy(I2C3_BASE));
	//specify that we are going to read from slave device
	I2CMasterSlaveAddrSet(I2C3_BASE, slave_addr, true);
	//send control byte and read from the register we
	//specified
	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
	//wait for MCU to finish transaction
	SysCtlDelay(500);
	while(I2CMasterBusy(I2C3_BASE));
	//return data pulled from the specified register
	return I2CMasterDataGet(I2C3_BASE);
}

/*********************************************************************************************
 * readHIH
 * ******************************************************************************************/
void readHIH(uint16_t *values){
	//
	uint16_t txdata;
	// Measurement Request command
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
	SSIDataPutNonBlocking(SSI1_BASE, 0xAA);
	SSIDataGet(SSI1_BASE, &txdata);
	delayUS(160);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
	// Conversion time: 36.65mS
	delayMS(40);
	// Data Fetch
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
	SSIDataPutNonBlocking(SSI1_BASE, 0x0000);
	SSIDataGet(SSI1_BASE, &values[0]);
	SSIDataPutNonBlocking(SSI1_BASE, 0x0000);
	SSIDataGet(SSI1_BASE, &values[1]);
	delayUS(330);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
}
/*********************************************************************************************
 * getHR
 * ******************************************************************************************/
double getHR(uint16_t *values){
	// Convert to get relative humidity
	double final;
	//
	final = values[0] & 0x3FFF;
	final = final *100;
	final = final /16382;
	//
	return final;
}
/*********************************************************************************************
 * getTBS
 * ******************************************************************************************/
double getTBS(uint16_t *values){
	// Convert to get Tbs
	double final;
	//
	final = values[1] >> 2;
	final = final*165;
	final = final/16382;
	final = final-40;
	//
	return final;
}

/*********************************************************************************************
 * getTG
 * ******************************************************************************************/
float getTG(void){
	//
	static unsigned long Tg_Raw, Tg_MSB,Tg_LSB, Tg_Sign;
	float finaltemp;
	//
	I2CMasterSlaveAddrSet(I2C3_BASE,SLAVE_ADDRESS_TG, false);
	I2CMasterDataPut(I2C3_BASE, TG_REG_READ);
	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while(!I2CMasterBusy(I2C3_BASE));
	while(I2CMasterBusy(I2C3_BASE));
	I2CMasterSlaveAddrSet(I2C3_BASE, SLAVE_ADDRESS_TG, true);
	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
	while(!I2CMasterBusy(I2C3_BASE));
	while(I2CMasterBusy(I2C3_BASE));
	// Most significant bits
	//Tg_MSB = I2CMasterErr(I2C3_MASTER_BASE);
	Tg_MSB = I2CMasterDataGet(I2C3_BASE) << 8;
	// Less significant bits
	I2CMasterControl (I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
	while(!I2CMasterBusy(I2C3_BASE));
	while(I2CMasterBusy(I2C3_BASE));
	Tg_LSB = I2CMasterDataGet(I2C3_BASE);
	// Low power mode on
	I2CMasterSlaveAddrSet(I2C3_BASE,SLAVE_ADDRESS_TG, false);
	I2CMasterDataPut(I2C3_BASE, TG_REG_LOWPOW);
	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while(!I2CMasterBusy(I2C3_BASE));
	//while(I2CMasterBusy(I2C3_BASE));
	// Final value
	Tg_Raw = (Tg_MSB+ Tg_LSB) >> 3;
	Tg_Sign = Tg_Raw & 0x1000;
	// Check for a negative temperature
	if(Tg_Sign){
		// Complemento a 2 de la temperatura
		Tg_Raw = Tg_Raw&0xFFF;
		Tg_Raw = 3696-Tg_Raw;
		finaltemp = Tg_Raw*(-0.0625);
	}
	else finaltemp = Tg_Raw*0.0625;
	//
	return finaltemp;
}
/*****************************************************************************************************
 * dec2bcd
 ****************************************************************************************************/
//decimal to BCD conversion
unsigned char dec2bcd(unsigned char val)
{
	return (((val / 10) << 4) | (val % 10));
}
// convert BCD to binary
unsigned char bcd2dec(unsigned char val)
{
	return (((val & 0xF0) >> 4) * 10) + (val & 0x0F);
}

/*****************************************************************************************************
 * SetTimeDate
 ****************************************************************************************************/
//Set Time
void SetTimeDate(unsigned char sec, unsigned char min, unsigned char hour,unsigned char day, unsigned char date, unsigned char month,unsigned char year)
{
	I2CSend(SLAVE_ADDR_RTC,8,SEC,dec2bcd(sec),dec2bcd(min),dec2bcd(hour),dec2bcd(day),dec2bcd(date),dec2bcd(month),dec2bcd(year));
}

/*****************************************************************************************************
 * GetClock
 ****************************************************************************************************/
//Get Time and Date
unsigned char GetClock(unsigned char reg)
{
	unsigned char clockData = I2CReceive(SLAVE_ADDR_RTC,reg);
	return bcd2dec(clockData);
}

/*****************************************************************************************************
 * GetStrDate
 ****************************************************************************************************/
// Get date in a string
void GetStrDate(char *char_date){
	unsigned char date, month, year;
	//
	date = GetClock(DATE);
	month = GetClock(MONTH);
	year = GetClock(YEAR);
	//
	char_date[0] = Int2CharM(date/10);
	char_date[1] = Int2CharM(date%10);
	char_date[2] = '-';
	char_date[3] = Int2CharM(month/10);
	char_date[4] = Int2CharM(month%10);
	char_date[5] = '-';
	char_date[6] = Int2CharM(year/10);
	char_date[7] = Int2CharM(year%10);
	char_date[8] = '\0';
}

/*****************************************************************************************************
 * GetStrTime
 ****************************************************************************************************/
// Get time in a string
void GetStrTime(char *char_time){
	unsigned char sec, min, hour;
	//
	sec = GetClock(SEC);
	min = GetClock(MIN);
	hour = GetClock(HRS);
	//
	char_time[0]=Int2CharM(hour/10);
	char_time[1]=Int2CharM(hour%10);
	char_time[2]=':';
	char_time[3]=Int2CharM(min/10);
	char_time[4]=Int2CharM(min%10);
	char_time[5]=':';
	char_time[6]=Int2CharM(sec/10);
	char_time[7]=Int2CharM(sec%10);
	char_time[8] = '\0';
}
/*********************************************************************************************
 * getEV
 * ******************************************************************************************/
double getEV(void){
	static uint32_t Ev_MSB,Ev_LSB;
	unsigned long Ev1,Ev2;
	double Ev;
	//
	//
	// Ev
	//
	// A byte sent to the TSL256x with the most ignificant bit (MSB)
	// equal to 1 will be interpreted as a COMMAND byte
	//
	// The lower four bits of the COMMAND byte form the register
	// select address
	//
	/* ADDRESS RESISTER NAME REGISTER FUNCTION
	 *  −− COMMAND 			Specifies register address
	 *  0h CONTROL 			Control of basic functions
	 *  1h TIMING 			Integration time/gain control
	 *  2h THRESHLOWLOW 		Low byte of low interrupt threshold
	 *  3h THRESHLOWHIGH 	High byte of low interrupt threshold
	 *  4h THRESHHIGHLOW 	Low byte of high interrupt threshold
	 *  5h THRESHHIGHHIGH 	High byte of high interrupt threshold
	 *  6h INTERRUPT 		Interrupt control
	 *  7h −− 				Reserved
	 *  8h CRC 				Factory test — not a user register
	 *  9h −− 				Reserved
	 *  Ah ID 				Part number/ Rev ID
	 *  Bh −− 				Reserved
	 *  Ch DATA0LOW 			Low byte of ADC channel 0
	 *  Dh DATA0HIGH 		High byte of ADC channel 0
	 *  Eh DATA1LOW 			Low byte of ADC channel 1
	 *  Fh DATA1HIGH 		High byte of ADC channel 1
	 *  */
	I2CMasterSlaveAddrSet(I2C3_BASE,SLAVE_ADDRESS_EV, false);
	I2CMasterDataPut(I2C3_BASE, 0x80);
	I2CMasterDataPut(I2C3_BASE, 0x03);
	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while(!I2CMasterBusy(I2C3_BASE));
	while(I2CMasterBusy(I2C3_BASE));
	delayUS(400);
	//ADC0
	Ev_LSB = I2CReceive(SLAVE_ADDRESS_EV, 0x8C);
	Ev_MSB = I2CReceive(SLAVE_ADDRESS_EV, 0x8D);
	Ev1 = 256*Ev_MSB + Ev_LSB;
	//ADC1
	Ev_LSB = I2CReceive(SLAVE_ADDRESS_EV, 0x8E);
	Ev_MSB = I2CReceive(SLAVE_ADDRESS_EV, 0x8F);
	Ev2 = 256*Ev_MSB + Ev_LSB;
	// LowPow
	I2CMasterSlaveAddrSet(I2C3_BASE,SLAVE_ADDRESS_EV, false);
	I2CMasterDataPut(I2C3_BASE, 0x80);
	I2CMasterDataPut(I2C3_BASE, 0x00);
	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while(!I2CMasterBusy(I2C3_BASE));
	while(I2CMasterBusy(I2C3_BASE));
	// Final value
	if (Ev2/Ev1 <= 0.50){
		Ev = (0.0304*Ev1) - ((0.062*Ev2) * powf((Ev2/Ev1), 1.4));
	}
	else if(Ev2/Ev1 <= 0.61){
		Ev = (0.0224*Ev1) - (0.031*Ev2);
	}
	else if(Ev2/Ev1 <= 0.80){
		Ev = (0.0128*Ev1) - (0.0153*Ev2);
	}
	else if(Ev2/Ev1 <= 1.30){
		Ev = (0.00146*Ev1) - (0.00112*Ev2);
	}
	else Ev = 0;
	//
	return Ev;
}

/*********************************************************************************************
 * WriteFirstRow
 * ******************************************************************************************/
void WriteFirstLine(char *path_date, char *path_time){
	//
	// Estado del modulo hibernacion
	//
	unsigned long ulStatus;
	unsigned long ulPeriod;
	//
	// SD Card variables
	//
	FATFS FatFs;    /* Work area (file system object) for logical drive */
	FIL fil;        /* File object */
	FRESULT fr;     /* FatFs return code */
	FILINFO fno;
	UINT br;    /* File read count */
	DIR *dir;
	unsigned char sec,min,hour,day,date,month,year;
	const char str_date[] = "Fecha";
	const char str_hour[] = "Hora";
	const char str_tg[] = "Temperatura de globo [�C]";
	const char str_hr[] = "Humedad relativa [%]";
	const char str_tbs[] = "Temperatura de bulbo seco [�C]";
	const char str_ev[] = "Iluminancia [lux]";
	const char str_va[] = "Velocidad de aire [m/s]";
	const char newline[] = "\r\n";
	const char comma[] = ";";
	const char null[]="0";
	int filesize;
	//
	// Write first row of the CSV file
	//
	// Get time and date
	//
	sec = GetClock(SEC);
	min = GetClock(MIN);
	hour = GetClock(HRS);
	//day = GetClock(DAY);
	date = GetClock(DATE);
	month = GetClock(MONTH);
	year = GetClock(YEAR);
	//
	// Date
	//
	//inttostr(((month*100)+date), path_date);
	path_date[0] = Int2CharM(year/10);
	path_date[1] = Int2CharM(year%10);
	path_date[2] = '-';
	path_date[3] = Int2CharM(month/10);
	path_date[4] = Int2Char(month%10);
	path_date[5] = '-';
	path_date[6] = Int2CharM(date/10);
	path_date[7] = Int2CharM(date%10);
	path_date[8] = '\0';
	// Time
	//inttostr(((hour*100)+min), path_time);
	path_time[0] = Int2CharM(hour/10);
	path_time[1] = Int2CharM(hour%10);
	path_time[2] = '_';
	path_time[3] = Int2CharM(min/10);
	path_time[4] = Int2CharM(min%10);
	path_time[5] = '\0';
	//
	// Save path for hibernation
	//
	HibReg[2] = path_date[0];
	HibReg[3] = path_date[1];
	HibReg[4] = path_date[3];
	HibReg[5] = path_date[4];
	HibReg[6] = path_date[6];
	HibReg[7] = path_date[7];
	HibReg[8] = path_time[0];
	HibReg[9] = path_time[1];
	HibReg[10] = path_time[3];
	HibReg[11] = path_time[4];
	//
	if( f_mount(0,&FatFs) == FR_OK ){
		f_mkdir(path_date);
		f_chdir(path_date);
		f_mkdir(path_time);
		f_chdir(path_time);
		if(f_open(&fil, "Data.csv", FA_WRITE | FA_OPEN_ALWAYS ) == FR_OK){
			// First row str_date
			fr = f_write(&fil, str_date, strlen(str_date), &br);
			fr = f_write(&fil, comma, strlen(comma), &br);
			fr = f_write(&fil, str_hour, strlen(str_hour), &br);
			fr = f_write(&fil, comma, strlen(comma), &br);
			fr = f_write(&fil, str_tbs, strlen(str_tbs), &br);
			fr = f_write(&fil, comma, strlen(comma), &br);
			fr = f_write(&fil, str_hr, strlen(str_hr), &br);
			fr = f_write(&fil, comma, strlen(comma), &br);
			fr = f_write(&fil, str_tg, strlen(str_tg), &br);
			fr = f_write(&fil, comma, strlen(comma), &br);
			fr = f_write(&fil, str_ev, strlen(str_ev), &br);
			fr = f_write(&fil, comma, strlen(comma), &br);
			fr = f_write(&fil, str_va, strlen(str_va), &br);
			fr = f_write(&fil, comma, strlen(comma), &br);
			fr = f_write(&fil, newline, strlen(newline), &br);
			//end of f_write
		}
		//end of f_open
		f_close(&fil);
	}
	//end of f_mount
	f_mount(0, null);
}

/*****************************************************************************************************
 * GetNewLine
 ****************************************************************************************************/
void GetNewLine(char *addline){
	// TODO Not working!
	char null[]="0";
	const char comma[] = ";";
	char new_date[9];
	char new_time[9];
	double newTbs;
	double newHR;
	char newcharTg[6];
	char newcharHR[6];
	char newcharTbs[6];
	char newcharEv[6];
	double newEv;
	float newTg=0;
	uint16_t newHIHRaw[2];
	//
	// Get time and date
	//
	// Date
	GetStrDate(new_date);
	// Time
	GetStrTime(new_time);
	//
	// Get RH and Tbs
	//
	uint16_t txdata;
	// Measurement Request command
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
	SSIDataPutNonBlocking(SSI1_BASE, 0xAA);
	SSIDataGet(SSI1_BASE, &txdata);
	delayUS(160);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
	// Conversion time: 36.65mS
	delayMS(40);
	// Data Fetch
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
	SSIDataPutNonBlocking(SSI1_BASE, 0x0000);
	SSIDataGet(SSI1_BASE, &newHIHRaw[0]);
	SSIDataPutNonBlocking(SSI1_BASE, 0x0000);
	SSIDataGet(SSI1_BASE, &newHIHRaw[1]);
	delayUS(330);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
	// RH
	newHR = newHIHRaw[0] & 0x3FFF;
	newHR = newHR *100;
	newHR = newHR /16382;
	// Tbs
	newTbs = newHIHRaw[1] >> 2;
	newTbs = newTbs*165;
	newTbs = newTbs/16382;
	newTbs = newTbs-40;
	//readHIH(newHIHRaw);
	//newHR = getHR(newHIHRaw);
	//newTbs = getTBS(newHIHRaw);
	newTg = getTG();
	newEv = getEV();
	//
	// Convert to string
	//
	dtoa(newcharTg, newTg);
	dtoa(newcharHR, newHR);
	dtoa(newcharTbs, newTbs);
	dtoa(newcharEv, newEv);
	//
	// Concatenate to a single string
	//
	strcat(addline, new_date);
	strcat(addline, comma);
	strcat(addline, new_time);
	strcat(addline, comma);
	strcat(addline, newcharTbs);
	strcat(addline, comma);
	strcat(addline, newcharHR);
	strcat(addline, comma);
	strcat(addline, newcharTg);
	strcat(addline, comma);
	strcat(addline, newcharEv);
	strcat(addline, comma);
	strcat(addline, null);
	strcat(addline, comma);
}

/*****************************************************************************************************
 * push
 ****************************************************************************************************/
//Push from stack?
void push(unsigned int v)
{
	if(stack.p < stack.mx)
	{
		stack.data[stack.p] = v;
		stack.p ++;
	}
}

/*****************************************************************************************************
 * pop
 ****************************************************************************************************/
//Pop from stack?
unsigned int pop(void)
{
	unsigned int ret = 0;
	if(stack.p > 0)
	{
		stack.p--;
		ret = stack.data[stack.p];
	}
	return ret;
}

/*****************************************************************************************************
 * full
 ****************************************************************************************************/
//Stack full?
unsigned int full(void)
{
	return !(stack.mx - stack.p);
}
unsigned int empty(void)
{
	return !stack.p;
}

/*****************************************************************************************************
 * inttostr
 ****************************************************************************************************/
//Convert int to string
void inttostr(unsigned int v, char *str)
{
	unsigned int part;
	while(v && !full())
	{
		part = v % 10;
		push(part + 48);
		v = v / 10;
	}
	while(!empty())
	{
		*str = pop();
		str++;
	}
	*str = '\0';
}

/*****************************************************************************************************
 * Int2Char
 ****************************************************************************************************/
// Date and time are used to create directories, one value at a time.
char Int2Char(int var){
	return var+'0';
}

/*****************************************************************************************************
 * Int2Char
 ****************************************************************************************************/
inline char Int2Char2(unsigned int v)
{
	return (char)(v%10) + 48;
}

/*****************************************************************************************************
 * itoa
 ****************************************************************************************************/
// Integer to char string
char* itoa(int value, char* result, int base) {
	// check that the base if valid
	if (base < 2 || base > 36) { *result = '\0'; return result; }

	char* ptr = result, *ptr1 = result, tmp_char;
	int tmp_value;

	do {
		tmp_value = value;
		value /= base;
		*ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
	} while ( value );

	// Apply negative sign
	if (tmp_value < 0) *ptr++ = '-';
	*ptr-- = '\0';
	while(ptr1 < ptr) {
		tmp_char = *ptr;
		*ptr--= *ptr1;
		*ptr1++ = tmp_char;
	}
	return result;
}
/*****************************************************************************************************
 * my_strcat
 ****************************************************************************************************/
// Concatenate strings
char *my_strcat(char *dest, const char *src)
{
	char *rdest = dest;

	while (*dest)
		dest++;
	while (*dest++ = *src++)
		;
	return rdest;
}

/*****************************************************************************************************
 * FloatToString
 ****************************************************************************************************/
// convert float to string one decimal digit at a time
// assumes float is < 65536 and ARRAYSIZE is big enough
// problem: it truncates numbers at size without rounding
// str is a char array to hold the result, float is the number to convert
// size is the number of decimal digits you want
void FloatToString(char *str, float f, char size)

{
	char pos;  // position in string
	char len;  // length of decimal part of result
	char* curr;  // temp holder for next digit
	int value;  // decimal digit(s) to convert
	pos = 0;  // initialize pos, just to be sure
	value = (int)f;  // truncate the floating point number
	itoa(value,str, 10);  // this is kinda dangerous depending on the length of str
	// now str array has the digits before the decimal
	if (f < 0 )  // handle negative numbers
	{
		f *= -1;
		value *= -1;
	}
	len = strlen(str);  // find out how big the integer part was
	pos = len;  // position the pointer to the end of the integer part
	str[pos++] = '.';  // add decimal point to string

	while(pos < (size + len + 1) )  // process remaining digits
	{
		f = f - (float)value;  // hack off the whole part of the number
		f *= 10;  // move next digit over
		value = (int)f;  // get next digit
		itoa(value, curr, 10); // convert digit to string
		str[pos++] = *curr; // add digit to result string and increment pointer
	}
}

/*****************************************************************************************************
 * dtoa
 ****************************************************************************************************/
char * dtoa(char *s, double n) {
	// handle special cases
	if (isnan(n)) {
		strcpy(s, "nan");
	} else if (isinf(n)) {
		strcpy(s, "inf");
	} else if (n == 0.0) {
		strcpy(s, "0");
	} else {
		int digit, m, m1;
		char *c = s;
		int neg = (n < 0);
		if (neg)
			n = -n;
		// calculate magnitude
		m = log10(n);
		int useExp = (m >= 14 || (neg && m >= 9) || m <= -9);
		if (neg)
			*(c++) = '-';
		// set up for scientific notation
		if (useExp) {
			if (m < 0)
				m -= 1.0;
			n = n / pow(10.0, m);
			m1 = m;
			m = 0;
		}
		if (m < 1.0) {
			m = 0;
		}
		// convert the number
		while (n > PRECISION || m >= 0) {
			double weight = pow(10.0, m);
			if (weight > 0 && !isinf(weight)) {
				digit = floor(n / weight);
				n -= (digit * weight);
				*(c++) = '0' + digit;
			}
			if (m == 0 && n > 0)
				*(c++) = '.';
			m--;
		}
		if (useExp) {
			// convert the exponent
			int i, j;
			*(c++) = 'e';
			if (m1 > 0) {
				*(c++) = '+';
			} else {
				*(c++) = '-';
				m1 = -m1;
			}
			m = 0;
			while (m1 > 0) {
				*(c++) = '0' + m1 % 10;
				m1 /= 10;
				m++;
			}
			c -= m;
			for (i = 0, j = m-1; i<j; i++, j--) {
				// swap without temporary
				c[i] ^= c[j];
				c[j] ^= c[i];
				c[i] ^= c[j];
			}
			c += m;
		}
		*(c) = '\0';
	}
	return s;
}


/*****************************************************************************************************
 * OneWiregetROMs
 ****************************************************************************************************/
void OneWiregetROMs(void){
	//
	oneWireInit(&oneWirePort);
	oneWireResetPulse(&oneWirePort);
	oneWireSearchInit(&allRoms, MAXDEVS);
	oneWireSearchDev(&oneWirePort, &allRoms, 0);
}

/*****************************************************************************************************
 * OneWireStartConvertion
 ****************************************************************************************************/
void OneWireStartConvertion(void){
	oneWireResetPulse(&oneWirePort);
	//oneWireSelectDev(&oneWirePort, allRoms.roms[Ts_index].rom);
	oneWireSkipRom(&oneWirePort);
	//oneWireResetPulse(&oneWirePort);
	oneWireConvertTemp(&oneWirePort);
}

/*****************************************************************************************************
 * OneWireGetTemp
 ****************************************************************************************************/
int OneWireGetTemp(double *Ts, int Ts_index){
	//
	/*
	uint16_t iTemp;
	uint16_t Ts_sign;
	uint16_t internalT, internalSign;
	float internalDec;
	double Ts_internal;
	float Ts_dec = 0;
	int j;
	*/
	//
	//
	//
	// Start conversion
	//oneWireResetPulse(&oneWirePort);
	//oneWireSelectDev(&oneWirePort, allRoms.roms[Ts_index].rom);
	//oneWireSkipRom
	//oneWireConvertTemp(&oneWirePort);
	//
	oneWireResetPulse(&oneWirePort);
	oneWireSelectDev(&oneWirePort, allRoms.roms[Ts_index].rom);
	oneWireReadMem(&oneWirePort, data);
	// if CRC OK
	if(oneWireValidatePkg(data, 9))
	{
		//
		// MSB
		iTemp = data[1];
		// Desplazo bits
		iTemp = iTemp << 8;
		// MSB+LSB
		iTemp = iTemp | data[0];
		//
		// Check sign
		Ts_sign = (data[1] & 0x80 )>> 7;
		// if positive
		if (Ts_sign == 0)
		{
			// Get decimal value
			Ts_dec = ((data[0] & 0x0C) >> 2)*0.25;
			// Discard b0 and b1 and decimals to get an int
			iTemp = iTemp >> 4;
			// Get Ts
			*Ts = (iTemp) + Ts_dec;
			SysCtlDelay(50);
			//return Ts;
		}
		// else negative
		// TODO: NO FUNCIONA
		else
		{
			// Get int value without sign
			iTemp = iTemp & 0x7FFF;
			// Discard b0 and b1 and decimals to get an int
			iTemp = iTemp >> 4;
			// decimal value
			Ts_dec = ((data[0] & 0x0C) >> 2)*0.25;
			// Get Ts
			*Ts = (-1)*(2048 - iTemp)+(Ts_dec);
			SysCtlDelay(50);
			//return Ts;
		}
		// Internal temperature
		// MSB
		internalT = data[3];
		// Check sign
		internalSign = data[3] >> 7;
		if (internalSign == 0)
		{
			// Get decimal values
			internalDec = data[2] && 0xF0;
			internalDec = internalDec*0.0625;
			Ts_internal = internalT + internalDec;
			SysCtlDelay(50);

		}
	}
	if (*Ts >= 1000){
		return -1;
	}
	else return 0;
}
