#include "gnss.h"
#include <stdio.h>
#include <string.h>

uint8_t gnss_rx = 0;
uint8_t gnss_rx_buffer[GNSSBUFSIZE];
uint16_t gnss_rx_wrIdx = 0;
uint16_t gnss_rx_rdIdx = 0;
uint8_t gnss_rx_msgLen = 0;
uint8_t gnss_msg[75] = {0};
uint8_t gnss_gga[75] = {0};
uint8_t gnss_rmc[75] = {0};
GNSS_t GNSS;

#define GNSS_RXRB_INDEX(x, y) (((x) + (y)) % GNSSBUFSIZE)

void GNSS_UartReset()
{
	gnss_rx_wrIdx = 0;
	gnss_rx_rdIdx = 0;
	memset(gnss_rx_buffer, 0, GNSSBUFSIZE);
}

void GNSS_UartRecv()
{
	if (gnss_rx != '\n')
	{
		gnss_rx_buffer[gnss_rx_wrIdx] = gnss_rx;
		gnss_rx_wrIdx = GNSS_RXRB_INDEX(gnss_rx_wrIdx, 1);
	}
	else
	{
		GNSS_UartReset();
	}
}

uint8_t GNSS_CheckMsg()
{
	uint8_t i = 0;
	
	if (gnss_rx_rdIdx == gnss_rx_wrIdx) return 1;		// No rx
	
	while ((gnss_rx_rdIdx != gnss_rx_wrIdx) && (gnss_rx_buffer[gnss_rx_rdIdx] != '$'))	// Find header "$"
	{
		gnss_rx_rdIdx = GNSS_RXRB_INDEX(gnss_rx_rdIdx, 1);
	}
	
	while ((GNSS_RXRB_INDEX(gnss_rx_rdIdx, gnss_rx_msgLen) != gnss_rx_wrIdx) && (gnss_rx_buffer[GNSS_RXRB_INDEX(gnss_rx_rdIdx, gnss_rx_msgLen)] != '*'))	// Find end "*"
	{
		gnss_rx_msgLen ++;
		if (gnss_rx_msgLen > 75)	// End not found, reset
		{
			gnss_rx_msgLen = 0;
			gnss_rx_rdIdx = gnss_rx_wrIdx;
			return 1;
		}
	}
	
	if (gnss_rx_buffer[GNSS_RXRB_INDEX(gnss_rx_rdIdx, gnss_rx_msgLen)] != '*') return 1;
	
	if (	(gnss_rx_buffer[GNSS_RXRB_INDEX(gnss_rx_rdIdx, 3)] == 'G' &&
				gnss_rx_buffer[GNSS_RXRB_INDEX(gnss_rx_rdIdx, 4)] == 'G' &&
				gnss_rx_buffer[GNSS_RXRB_INDEX(gnss_rx_rdIdx, 5)] == 'A') ||
				(gnss_rx_buffer[GNSS_RXRB_INDEX(gnss_rx_rdIdx, 3)] == 'R' &&
				gnss_rx_buffer[GNSS_RXRB_INDEX(gnss_rx_rdIdx, 4)] == 'M' &&
				gnss_rx_buffer[GNSS_RXRB_INDEX(gnss_rx_rdIdx, 5)] == 'C'))
	{
		for (i = 0; i < 75; i ++)
		{
			if (i < gnss_rx_msgLen)
			{
				gnss_msg[i] = gnss_rx_buffer[GNSS_RXRB_INDEX(gnss_rx_rdIdx, i)];
				gnss_rx_buffer[GNSS_RXRB_INDEX(gnss_rx_rdIdx, i)] = 0;
			}
			else gnss_msg[i] = 0;
		}
		gnss_rx_rdIdx = GNSS_RXRB_INDEX(gnss_rx_rdIdx, gnss_rx_msgLen + 2);
		gnss_rx_msgLen = 0;
		return 0;
	}
	
	gnss_rx_rdIdx = GNSS_RXRB_INDEX(gnss_rx_rdIdx, gnss_rx_msgLen + 2);
	gnss_rx_msgLen = 0;
	
	return 1;
}

float GNSS_NEMA2Dec(float deg_coord, char dir) {
	int degree = (int)(deg_coord / 100);
	float minutes = deg_coord - degree * 100;
	float dec_deg = minutes / 60;
	float decimal = degree + dec_deg;
	
	if (dir == 'S' || dir == 'W') {
		decimal *= -1;
	}
	return decimal;
}

void GNSS_ConvertUTC(float nema, uint8_t *hour, uint8_t *minute, uint8_t *second)
{
	*hour = (int)(nema / 10000) % 100;
	*minute = (int)(nema / 100) % 100;
	*second = (int)(nema) % 100;
}

void GNSS_ConvertDate(int nema, uint16_t *year, uint8_t *month, uint8_t *day)
{
	*day = (nema / 10000) % 100;
	*month = (nema / 100) % 100;
	*year  = nema % 100 + 2000;
}

int8_t GNSS_ProcessMsg()
{
	uint8_t read = 0;
	float utc_time;
	float lat;
	float lon;
	char lat_dir, lon_dir;
	int quality;
	int sat;
	float hdop;
	float alt;
	char a_unit;
	float speed;
	float course;
	int date;
	char valid;
	
	if (strncmp((char*)gnss_msg, "$GNGGA", 6) == 0)
	{
		memset(gnss_gga, 0, 75);
		memcpy(gnss_gga, gnss_msg, 75);
		read = sscanf((char*)gnss_msg, "$GNGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c", &utc_time, &lat, &lat_dir, &lon, &lon_dir, &quality, &sat, &hdop, &alt, &a_unit);
			
		if (read <= 1) return -1;
		
		GNSS.latitude_deg = GNSS_NEMA2Dec(lat, lat_dir);
		GNSS.longitude_deg = GNSS_NEMA2Dec(lon, lon_dir);
		GNSS.quality = quality;
		GNSS.sat_number = sat;
		GNSS.altitude_m = alt;
		GNSS_ConvertUTC(utc_time, &GNSS.utc_hour, &GNSS.utc_minute, &GNSS.utc_second);
		memset(gnss_msg, 0, 75);
		return 2;
	}
	else if (strncmp((char*)gnss_msg, "$GNRMC", 6) == 0)
	{
		memset(gnss_rmc, 0, 75);
		memcpy(gnss_rmc, gnss_msg, 75);
		read = sscanf((char*)gnss_msg, "$GNRMC,%f,%c,%f,%c,%f,%c,%f,%f,%d", &utc_time, &valid, &lat, &lat_dir, &lon, &lon_dir, &speed, &course, &date);
			
		if (read <= 1) return -2;
		if (valid == 'A') GNSS.valid = 1;
		else GNSS.valid = 0;
		GNSS.latitude_deg = GNSS_NEMA2Dec(lat, lat_dir);
		GNSS.longitude_deg = GNSS_NEMA2Dec(lon, lon_dir);
		GNSS_ConvertUTC(utc_time, &GNSS.utc_hour, &GNSS.utc_minute, &GNSS.utc_second);
		GNSS_ConvertDate(date, &GNSS.utc_year, &GNSS.utc_month, &GNSS.utc_day);
		memset(gnss_msg, 0, 75);
		return 1;
	}
	return -3;
}