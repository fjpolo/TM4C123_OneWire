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
 *	datalogger.h v1.12
 *	20.09.2016
 *
 *	Descripci�n:
 *
 *	 Funciones relacionadas con los sensores, protocolos de comunicacion y el
 *	modulo de hibernaci�n, asi como tambien interrupciones e incializacion.
 *
 *  Desarrollo del firmware de la placa base del data logger, constando de:
 *
 *  - Perif�ricos I2C:
 *  	a) HR y Tbs		HIH9131		0b0100111		0x27
 *  	b) Ev			TSL2563		0b0101001		0x29
 *  	c) Va			ADS			0b1001000		0x48
 *  	d) Tg			LM92		0b1001011		0x51
 *  	e) RTC			DS1703		0b1101000		0x68
 *
 *  - Perif�ricos OneWire@PD6
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

#ifndef DATALOGGER_H_
#define DATALOGGER_H_
//
// Initialization
//
void Initialize(void);
void InitClock(void);
void InitGPIO(void);
void InitTimer0(void);
void InitGPIOInt(void);
void InitHibernation(void);
void InitSDCard(void);
void InitI2C3(void);
void InitSPI1(void);
void InitOneWire(int timer);
//
// I2C
//
void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...);
uint32_t I2CReceive(uint32_t slave_addr, uint8_t reg);
//
// DS1307 - RTC
//
unsigned char dec2bcd(unsigned char val);
unsigned char bcd2dec(unsigned char val);
void SetTimeDate(unsigned char sec,
				unsigned char min,
				unsigned char hour,
				unsigned char day,
				unsigned char date,
				unsigned char month,
				unsigned char year);
unsigned char GetClock(unsigned char reg);
void GetStrDate(char *char_date);
void GetStrTime(char *char_time);
//
// LM92 - Tg
//
float getTG(void);
//
// SSI1 HIH9131 - HR & Tbs
//
void readHIH(uint16_t *values);
double getHR(uint16_t *values);
double getTBS(uint16_t *values);
//
// TSL2563 - Ev
//
double getEV(void);
//
// Data logging / Hibernation
//
void DataLoggingON(unsigned long *Status);
void DataLoggingOFF(void);
//
// SDCard
//
void WriteFirstLine(char *path_date, char *path_time);
void GetNewLine(char *addline);
//void AppendData(char *path_date, char *path_time);
void Signature(char *path_date, char *path_time);
//
// MAX31850 - OneWire
//
void OneWiregetROMs(void);
void OneWireStartConvertion(void);
int OneWireGetTemp(double *Ts, int Ts_index);
//
// Hibernation
//
void DataLoggingON(unsigned long *Status);
void DataLoggingOFF(void);
//
// MISC
//
void push(unsigned int v);
unsigned int pop(void);
unsigned int full(void);
void inttostr(unsigned int v, char *str);
char Int2Char(int var);
inline char Int2Char2(unsigned int v);
void FloatToString(char *str, float f, char size);
char *my_strcat(char *dest, const char *src);
char* itoa(int value, char* result, int base);
char * dtoa(char *s, double n);
//
#endif /* DATALOGGER_H_ */
