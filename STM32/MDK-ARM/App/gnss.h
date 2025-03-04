#ifndef __GNSS_H__
#define	__GNSS_H__

#include "stm32l4xx_hal.h"

#define GNSSBUFSIZE  512       // GNSS buffer size

typedef struct
{
	float longitude_deg;
	float latitude_deg;
	float altitude_m;
	
	uint16_t utc_year;
	uint8_t utc_month;
	uint8_t utc_day;
	uint8_t utc_hour;
	uint8_t utc_minute;
	uint8_t utc_second;
	
	int quality;
	int sat_number;
	
	uint8_t valid;
} GNSS_t;

extern uint8_t gnss_rx;
extern uint8_t gnss_msg[75];
extern uint8_t gnss_gga[75];
extern uint8_t gnss_rmc[75];
extern GNSS_t GNSS;

void GNSS_UartReset(void);
void GNSS_UartRecv(void);
uint8_t GNSS_CheckMsg(void);
int8_t GNSS_ProcessMsg(void);
#endif
