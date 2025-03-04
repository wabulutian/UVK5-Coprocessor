/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gnss.h"
#include "sgp4lib.h"
#include "time.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	char name[64];						//64
	struct sgp_data sgpParam;	//100
	uint32_t  	downlinkFreq;	//4		Hz
	uint32_t  	uplinkFreq;		//4		Hz
}st_satData;	//170

//--------For I2C Read-----------

// The purpose is to move as much calcluation as possible from K5 to STM32
// The UVK5 is only a "control and display device"

typedef struct
{
	uint8_t	hh;		//2
	uint8_t mm;		//2
	uint8_t ss;		//2
	int8_t tz;		//2
}st_time24MsgPack;

typedef struct
{
	// send integer+decimal
	// NOTICE:
	// The only purpose of UVK5 for longitude and latitude values is to display
	// Send integer+decimal to avoid additional calculation to save K5's flash space
	// THIS TAKES MUCH SPACE: 				sprintf(String, "%03d.%03d", siteLat_Deg, (siteLat_Deg * 1000) % 1000)
	// THIS TAKES ALMOST NO SPACE: 		sprintf(String, "%03d.%03d", siteLat_Deg, siteLat_0_001Deg)
	
	uint16_t siteLat_Deg;
	uint16_t siteLon_Deg;
	uint16_t siteLat_0_001Deg;
	uint16_t siteLon_0_001Deg;
	// send character to display directly
	char NS;
	char EW;
	char siteMaidenhead[8];
}st_siteInfoMsgPack;

typedef struct
{
	char name[11];						//11
	uint8_t valid;						//1
	uint32_t  	downlinkFreq;	//4
	uint32_t  	uplinkFreq;		//4
	int16_t satAz_1Deg;			//2
	int16_t satEl_1Deg;			//2
	int16_t satSpd_1mps;			//2
	int16_t downlinkDoppler;	//2
	int16_t	uplinkDoppler;		//2
}st_satStatusMsgPack;	//30

typedef struct
{
	// This az-el pair is for drawing radar-plot in UVK5
	// The drawing function in K5 only has 2-deg resolution
	// Send index in 2-deg to avoid /2 operation
	uint8_t az_2Deg[60];
	int8_t	el_2Deg[60];
	uint16_t nextEvent_sec;
	uint8_t nextEvent_mm;
	uint8_t nextEvent_ss;
}st_satPredict120min_2min;//124

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_DIR_SLAVE_OUT		0
#define I2C_DIR_SLAVE_IN		1
#define I2C_PASSTHROUGH_BUFFER_SIZE	300
#define I2C_RXRB_INDEX(x,y)	((x+y)%I2C_PASSTHROUGH_BUFFER_SIZE)

#define FLASH_SAVE_satList_ADDR						0x08010000
#define FLASH_SAVE_satList_SIZE64					220
#define FLASH_SAVE_validSatIdx_ADDR				0x08011000
#define FLASH_SAVE_validSatIdx_SIZE64			2
#define FLASH_SAVE_siteData_ADDR					0x08011010
#define FLASH_SAVE_siteData_SIZE64				3
#define FLASH_SAVE_tz_ADDR								0x08011020
#define FLASH_SAVE_tz_SIZE64							1
#define FLASH_SAVE_site_ADDR							0x08011800
#define FLASH_SAVE_site_SIZE64						3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t validSatIdx[16];	// valid range 0~9, padding to 15 for memory alignment
uint8_t firstFix = 0;			// first gnss fix after power up
uint8_t satCalSlice = 0;	// timeslice for satellite calculate, 0~9 -> sat index 0~9
uint8_t satCalDone = 0;		// is calculate task in this slice complete
uint8_t recvGNSS = 1;			// enable gnss messagne rx
uint8_t recvGNSS_old = 1;	// for posedge detect

union
{
	st_satData satList[10];		// 0x6E0 = 1760 = 220 * u64
	uint64_t array[220];
}un_satData;

st_sgpSatStatus predSgpSatStatusTmp;	// for predict
st_sgpSatStatus satStatusList[10];		// for real-time data

union
{
	st_siteData siteData;		//24
	uint64_t array[3];
}un_siteData;

tm_t dateTime, calDateTime, predDateTime;
int8_t timeZone = 8;

union
{
	uint8_t u8Arr[2];
	uint16_t u16Addr;
} i2cAddr;

uint8_t i2cDataDirection = I2C_DIR_SLAVE_OUT;
uint8_t isI2CRegAddrRecv = 0;		// 0 - no; 1 - msb received; 2 - addr rx cplt

// uart_to_i2c passthrough: PC >-UART-> UVK5 >-I2C-> STM32, for TLE import
uint8_t i2cPassthroughBuffer[I2C_PASSTHROUGH_BUFFER_SIZE];
uint16_t i2cPassthrougBufferWritePointer = 0;
uint16_t i2cPassthrougBufferReadPointer = 0;
char i2cTleRx[5][71];
int i2cTleReadStatus;
uint8_t i2cNewTleValid = 0;
struct tle_ascii tleTmp;
uint32_t  	dlTmp;
uint32_t  	ulTmp;

// i2c com related
uint8_t *i2cDataPtr = 0;
uint16_t i2cDataPtrOffset = 0;
uint8_t i2cAddSatDataIdx = 0;
uint8_t i2cDelSatDataIdx = 0;
uint8_t i2cSatFocusMode = 0;
uint8_t i2cSatFocusIdx = 0;
struct
{
	st_siteInfoMsgPack i2cSiteInfo;
	st_time24MsgPack i2cLocalTime;
	st_satStatusMsgPack i2cSatLiveList[10];
	st_satPredict120min_2min i2cSatPredict120mList[10];
}i2cReg;
int8_t i2cTzSet = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


/*
*		TLE Format:
*		TLE+
*		{{Satellite Name}}
*		{{TLE Line1}}
*		{{TLE Line2}}
*		{{transponder uplink(earth-to-satellite) frequency in Hz}}
*		{{transponder down(satellite-to-earth) frequency in Hz}}
*		$
*/
int8_t I2C_TLEReader()
{
	uint16_t MsgRxLength;

	if (i2cPassthrougBufferReadPointer < i2cPassthrougBufferWritePointer) {
			MsgRxLength = i2cPassthrougBufferWritePointer - i2cPassthrougBufferReadPointer;
	} else {
			MsgRxLength = (i2cPassthrougBufferWritePointer + sizeof(I2C_PASSTHROUGH_BUFFER_SIZE)) - i2cPassthrougBufferReadPointer;
	}
	if (MsgRxLength < 10) return -1;

	while (1) {

		while (i2cPassthrougBufferReadPointer != i2cPassthrougBufferWritePointer &&
		(i2cPassthroughBuffer[i2cPassthrougBufferReadPointer] != 'T' ||
		i2cPassthroughBuffer[I2C_RXRB_INDEX(i2cPassthrougBufferReadPointer, 1)] != 'L' ||
		i2cPassthroughBuffer[I2C_RXRB_INDEX(i2cPassthrougBufferReadPointer, 2)] != 'E' ||
		i2cPassthroughBuffer[I2C_RXRB_INDEX(i2cPassthrougBufferReadPointer, 3)] != '+')) {
			i2cPassthrougBufferReadPointer = I2C_RXRB_INDEX(i2cPassthrougBufferReadPointer, 1);
		}
		
		if (i2cPassthrougBufferReadPointer == i2cPassthrougBufferWritePointer) return -2;
		
		if (i2cPassthrougBufferReadPointer < i2cPassthrougBufferWritePointer) {
			MsgRxLength = i2cPassthrougBufferWritePointer - i2cPassthrougBufferReadPointer;
		} else {
			MsgRxLength = (i2cPassthrougBufferWritePointer + sizeof(I2C_PASSTHROUGH_BUFFER_SIZE)) - i2cPassthrougBufferReadPointer;
		}

 		if (MsgRxLength < 10) return -3;

		int16_t endPos = -1;
		for (int i = 0; i < MsgRxLength; i++) {
			if (i2cPassthroughBuffer[I2C_RXRB_INDEX(i2cPassthrougBufferReadPointer, i)] == '$') {
				endPos = i;
				break;
			}
		}
		
		if (MsgRxLength > 230)
		{
			i2cPassthrougBufferReadPointer = i2cPassthrougBufferWritePointer;
			return -4;
		}
 		if (endPos == -1) return -5;

 		uint16_t readIdx = 6;
		uint8_t lineIdx = 0;
		uint8_t charIdx = 0;
		for (int i = 6; i < endPos; i++) // 'T''L''E''+''\r''\n'
		{	
			char tmpChar = i2cPassthroughBuffer[I2C_RXRB_INDEX(i2cPassthrougBufferReadPointer, readIdx)];
			readIdx ++;
			if (tmpChar == '\r') continue;
			else if (tmpChar == '\n')
			{
				lineIdx ++;
				charIdx = 0;
			}
			else
			{
				i2cTleRx[lineIdx][charIdx] = tmpChar;
				charIdx ++;
			}
		}
 		i2cPassthrougBufferReadPointer = i2cPassthrougBufferWritePointer;

 		return 0;
	}
}

void SetSite(st_siteData *dst, double latDeg, double lonDeg, double altKm)
{
    dst->siteLat = latDeg * pi / 180.0f;
    dst->siteLon = lonDeg * pi / 180.0f;
    dst->siteAlt = altKm;
}

//-------------STM32 Internal RTC Moudule--------------------
void RTC_SetTime (uint8_t HH, uint8_t MM, uint8_t SS)
{
	RTC_TimeTypeDef sTime = {0};	// init by 0 to prevent 1 hour off
	sTime.Hours = HH;
	sTime.Minutes = MM;
	sTime.Seconds = SS;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}
}
void RTC_SetDate (uint8_t YY, uint8_t MM, uint8_t DD)
{
	RTC_DateTypeDef sDate = {0};
	sDate.WeekDay = RTC_WEEKDAY_MONDAY;
	sDate.Month = MM;
	sDate.Date = DD;
	sDate.Year = YY;
	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}
}
void RTC_SetDateTime(int yr, int mon, int day, int hour, int min, int sec)
{
	if (mon == 99 || day == 99 || hour == 99 || min == 99 || sec == 99) return;
	RTC_SetDate(yr, mon, day);
	RTC_SetTime(hour, min, sec);
}
void GetDateTime(tm_t* tm)
{
	RTC_DateTypeDef nowDate ;
	RTC_TimeTypeDef nowTime ;

	HAL_RTC_GetTime (&hrtc,&nowTime,RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc,&nowDate,RTC_FORMAT_BIN);
	SetTmTime(tm, nowDate.Year + 2000, nowDate.Month, nowDate.Date, nowTime.Hours, nowTime.Minutes, nowTime.Seconds);
}
void ConvertTimeZone(tm_t* input, tm_t* output, int8_t zone) {

    long long unixTime = input->tv;

    unixTime += zone * 3600;


    int days = unixTime / 86400;
    int seconds = unixTime % 86400;
    
    output->hour = seconds / 3600;
    output->min = (seconds % 3600) / 60;
    output->sec = seconds % 60;

    int year = 1970;
    int month = 1;
    int monthDays;

    while (days >= (monthDays = GetDaysInMonth(year, month))) {
        days -= monthDays;
        if (++month > 12) {
            month = 1;
            year++;
        }
    }

    output->year = year;
    output->month = month;
    output->day = days + 1;
    output->tv = unixTime;
		output->zone = zone;
}
void Constructtime24MsgPack(st_time24MsgPack *dst, tm_t* src, int8_t tz)
{
	int8_t tmp = src->hour;
	tmp += tz;
	tmp = tmp % 24;
	dst->hh = tmp;
	dst->mm = src->min;
	dst->ss = src->sec;
	dst->tz = tz;
}
//------------------------------------------------

void SaveSiteData()
{
	int i;
	FLASH_EraseInitTypeDef myFlash;
	uint32_t pageError = 0U;
	
	HAL_I2C_DisableListen_IT(&hi2c2);
	HAL_NVIC_DisableIRQ(USART1_IRQn);
	myFlash.TypeErase = FLASH_TYPEERASE_PAGES;
	myFlash.Page = 35;
	myFlash.NbPages = 1;
	
	HAL_FLASH_Unlock();
	
	HAL_FLASHEx_Erase(&myFlash, &pageError);
	
	for (i = 0; i < FLASH_SAVE_site_SIZE64; i ++)
	{
		HAL_FLASH_Program(0, FLASH_SAVE_site_ADDR + (i * 8), un_siteData.array[i]);
	}
	HAL_FLASH_Lock();
	HAL_I2C_EnableListen_IT(&hi2c2);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void SaveSysData()
{
	int i;
	FLASH_EraseInitTypeDef myFlash;
	uint32_t pageError = 0U;
	
	HAL_I2C_DisableListen_IT(&hi2c2);
	HAL_NVIC_DisableIRQ(USART1_IRQn);
	myFlash.TypeErase = FLASH_TYPEERASE_PAGES;
	myFlash.Page = 32;
	myFlash.NbPages = 3;
	
	HAL_FLASH_Unlock();
	
	HAL_FLASHEx_Erase(&myFlash, &pageError);
	
	for (i = 0; i < FLASH_SAVE_satList_SIZE64; i ++)
	{
		HAL_FLASH_Program(0, FLASH_SAVE_satList_ADDR + (i * 8), un_satData.array[i]);
	}
	for (i = 0; i < FLASH_SAVE_validSatIdx_SIZE64; i ++)
	{
		HAL_FLASH_Program(0, FLASH_SAVE_validSatIdx_ADDR + (i * 8), *((uint64_t*)&validSatIdx[i * 8]));
	}
	HAL_FLASH_Program(0, FLASH_SAVE_tz_ADDR, timeZone + 12);
	HAL_FLASH_Lock();
	HAL_I2C_EnableListen_IT(&hi2c2);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void RestoreSysData()
{
	int i;
	timeZone = *((uint8_t *)FLASH_SAVE_tz_ADDR) - 12;
	for (i = 0; i < FLASH_SAVE_satList_SIZE64; i ++)
	{
		un_satData.array[i] = *((uint64_t*)FLASH_SAVE_satList_ADDR + i);
	}
	*(uint64_t*)&validSatIdx[0] = *(uint64_t*)FLASH_SAVE_validSatIdx_ADDR;
	*(uint64_t*)&validSatIdx[8] = *((uint64_t*)FLASH_SAVE_validSatIdx_ADDR + 1);
	
	for (i = 0; i < FLASH_SAVE_site_SIZE64; i ++)
	{
		un_siteData.array[i] = *((uint64_t*)FLASH_SAVE_site_ADDR + i);
	}
}

uint8_t LatLon2Maindenhead(double latDeg, double lonDeg, char* dst) {
    if (dst == NULL) {
        return 1;
    }

    while (lonDeg < -180.0) lonDeg += 360.0;
    while (lonDeg > 180.0) lonDeg -= 360.0;

    if (latDeg < -90.0 || latDeg > 90.0) {
        return 2;
    }

    // convert lat and lon to maidenhead range
    lonDeg += 180.0;  // 0-360
    latDeg += 90.0;   // 0-180

    // calculate 1st pair (field)
    dst[0] = 'A' + (int)(lonDeg / 20.0);
    dst[1] = 'A' + (int)(latDeg / 10.0);

    // calculate 2nd pair (block)
    dst[2] = '0' + (int)((int)lonDeg % 20) / 2;
    dst[3] = '0' + (int)((int)latDeg % 10);

    // calculate 3rd pair (sub block)
    dst[4] = 'A' + (int)((lonDeg - (int)lonDeg) *12.0);
    dst[5] = 'A' + (int)((latDeg - (int)latDeg)* 24.0);

    // end
    dst[6] = '\0';

    return 0;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_I2C_EnableListen_IT(&hi2c2);
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, &gnss_rx, 1);
	
//#define INIT_FLASH		1
#ifdef INIT_FLASH
	int aa;
	SetSite(&un_siteData.siteData, 30, 120, 0);
	RTC_SetDateTime(25, 2, 26, 8, 0, 0);
	timeZone = 8;
	strcpy(tleTmp.l[0], "ISS");
	strcpy(tleTmp.l[1], "1 25544U 98067A   25057.69551956  .00051272  00000-0  91556-3 0  9991");
	strcpy(tleTmp.l[2], "2 25544  51.6387 134.2889 0005831 315.8203 179.6729 15.49515680498024");
	strcpy(un_satData.satList[0].name, tleTmp.l[0]);
	un_satData.satList[0].downlinkFreq = 437800000;
	un_satData.satList[0].uplinkFreq = 145990000;
	Convert_Satellite_Data(tleTmp, &un_satData.satList[0].sgpParam);
	validSatIdx[0] = 1;
	strcpy(tleTmp.l[0], "RS-44");
	strcpy(tleTmp.l[1], "1 44909U 19096E   25057.59056133  .00000048  00000-0  14707-3 0  9996");
	strcpy(tleTmp.l[2], "2 44909  82.5204 295.7264 0217445 176.1243 184.1608 12.79738608241568");
	strcpy(un_satData.satList[1].name, tleTmp.l[0]);
	un_satData.satList[1].downlinkFreq = 	435640000;
	un_satData.satList[1].uplinkFreq = 		145965000;
	validSatIdx[1] = 1;
	Convert_Satellite_Data(tleTmp, &un_satData.satList[1].sgpParam);
	SaveSysData();
	SaveSiteData();
#else
	RestoreSysData();
#endif
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		recvGNSS = !i2cSatFocusMode;	// stop receive GNSS message while focus mode enabled
		if (!recvGNSS_old && recvGNSS)
		{
			// idk why but i should send this command twice to restart uart listening
			HAL_UARTEx_ReceiveToIdle_IT(&huart1, &gnss_rx, 1);
			HAL_UARTEx_ReceiveToIdle_IT(&huart1, &gnss_rx, 1);
		}
		recvGNSS_old = recvGNSS;
		
		// GNSS message process
		if (recvGNSS)
		{
			if (GNSS_CheckMsg() == 0)
			{
				if (GNSS_ProcessMsg() == 1 && GNSS.valid == 1)
				{
					SetSite(&un_siteData.siteData, GNSS.latitude_deg, GNSS.longitude_deg, GNSS.altitude_m / 1000.0f);
					RTC_SetDateTime(GNSS.utc_year - 2000, GNSS.utc_month, GNSS.utc_day, GNSS.utc_hour, GNSS.utc_minute, GNSS.utc_second);
					if (!firstFix)
					{
						// Save site data to flash after first GNSS fix
						firstFix = 1;
						SaveSiteData();
					}
				}
			}
		}
		
		// has pending calculate task?
		if (!satCalDone)
		{
			satCalDone = 1;
			
			// update i2c msg pack
			i2cReg.i2cSatLiveList[satCalSlice].valid = validSatIdx[satCalSlice];
			memcpy(i2cReg.i2cSatLiveList[satCalSlice].name, un_satData.satList[satCalSlice].name, 10);
			i2cReg.i2cSatLiveList[satCalSlice].downlinkFreq = un_satData.satList[satCalSlice].downlinkFreq;
			i2cReg.i2cSatLiveList[satCalSlice].uplinkFreq = un_satData.satList[satCalSlice].uplinkFreq;
			
			// only calculate valid sat
			if (validSatIdx[satCalSlice] != 0)
			{
				uint16_t predSec;
				int32_t diffUp, diffDn;
				int8_t elTmp;
				uint8_t alreadyAoS = 0;
				uint8_t pred2MinIdx = 0, aosLos2MinMark = 0;

				memcpy(&calDateTime, &dateTime, 64);
				SGP4_FindSatStatus(calDateTime, &un_satData.satList[satCalSlice].sgpParam, &un_siteData.siteData, &satStatusList[satCalSlice]); 
				
				// trasnform data into i2c pack
				i2cReg.i2cSatLiveList[satCalSlice].satAz_1Deg = satStatusList[satCalSlice].satAz;
				i2cReg.i2cSatLiveList[satCalSlice].satEl_1Deg = satStatusList[satCalSlice].satEl;
				i2cReg.i2cSatLiveList[satCalSlice].satSpd_1mps = satStatusList[satCalSlice].satSpd * 1000;
				diffUp = un_satData.satList[satCalSlice].uplinkFreq / 299792 * satStatusList[satCalSlice].satSpd;
				i2cReg.i2cSatLiveList[satCalSlice].uplinkDoppler = diffUp;
				diffDn = un_satData.satList[satCalSlice].downlinkFreq / 299792 * satStatusList[satCalSlice].satSpd;
				i2cReg.i2cSatLiveList[satCalSlice].downlinkDoppler = diffDn;
				i2cReg.i2cSatPredict120mList[satCalSlice].az_2Deg[0] = satStatusList[satCalSlice].satAz / 2;
				i2cReg.i2cSatPredict120mList[satCalSlice].el_2Deg[0] = satStatusList[satCalSlice].satEl / 2;
				
				if (i2cSatFocusMode)
				{
					elTmp = satStatusList[satCalSlice].satEl / 2;
					if (elTmp >= 0) alreadyAoS = 1;
					pred2MinIdx = 1;
					for (predSec = (2 * 60); predSec < (120 * 60); predSec += (2 * 60))
					{
						SGP4_AddTime(predSec, &calDateTime, &predDateTime);
						SGP4_FindSatStatus(predDateTime, &un_satData.satList[satCalSlice].sgpParam, &un_siteData.siteData, &predSgpSatStatusTmp); 
						i2cReg.i2cSatPredict120mList[satCalSlice].az_2Deg[pred2MinIdx] = predSgpSatStatusTmp.satAz / 2;
						i2cReg.i2cSatPredict120mList[satCalSlice].el_2Deg[pred2MinIdx] = predSgpSatStatusTmp.satEl / 2;
						elTmp = predSgpSatStatusTmp.satEl;
						
						// No AOSLOS record + Not already AOS + Rise
						// No AOSLOS record + Already AOS + Fall
						// ==> Record time mark
						if ((aosLos2MinMark == 0) && ((!alreadyAoS && elTmp > 0)|| (alreadyAoS && elTmp <= 0))) aosLos2MinMark = pred2MinIdx;
						pred2MinIdx ++;
					}
					// if a mark exist, search previous 120s for the exact second
					if (aosLos2MinMark != 0)
					{
						for (predSec = ((aosLos2MinMark - 1) * 120); predSec < (aosLos2MinMark * 120); predSec ++)
						{
							SGP4_AddTime(predSec, &calDateTime, &predDateTime);
							SGP4_FindSatStatus(predDateTime, &un_satData.satList[satCalSlice].sgpParam, &un_siteData.siteData, &predSgpSatStatusTmp);
							elTmp = predSgpSatStatusTmp.satEl;
							if ((!alreadyAoS && elTmp > 0)|| (alreadyAoS && elTmp <= 0))
							{
								i2cReg.i2cSatPredict120mList[satCalSlice].nextEvent_sec = predSec;
								i2cReg.i2cSatPredict120mList[satCalSlice].nextEvent_mm = predSec / 60;
								i2cReg.i2cSatPredict120mList[satCalSlice].nextEvent_ss = predSec % 60;
								break;
							}
						}
					}
				}
			}
		}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// 200ms
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2)
	{
		GetDateTime(&dateTime);
		Constructtime24MsgPack(&i2cReg.i2cLocalTime, &dateTime, timeZone);
		double tmpLa = un_siteData.siteData.siteLat / pi * 180;
		double tmpLo = un_siteData.siteData.siteLon / pi * 180;
		
		if (tmpLa > 0)
		{
			i2cReg.i2cSiteInfo.NS = 'N';
		}
		else
		{
			tmpLa = -tmpLa;
			i2cReg.i2cSiteInfo.NS = 'S';
		}
		
		if (tmpLo > 0)
		{
			i2cReg.i2cSiteInfo.EW = 'E';
		}
		else
		{
			tmpLo = -tmpLo;
			i2cReg.i2cSiteInfo.EW = 'W';
		}
		
		i2cReg.i2cSiteInfo.siteLat_Deg = (int)tmpLa;
		i2cReg.i2cSiteInfo.siteLat_0_001Deg = ((int)(tmpLa * 1000) % 1000);
		i2cReg.i2cSiteInfo.siteLon_Deg = (int)tmpLo;
		i2cReg.i2cSiteInfo.siteLon_0_001Deg = ((int)(tmpLo * 1000) % 1000);

		LatLon2Maindenhead(tmpLa, tmpLo, i2cReg.i2cSiteInfo.siteMaidenhead);
		satCalDone = 0;
		if (i2cSatFocusMode) satCalSlice = i2cSatFocusIdx;
		else
		{
			satCalSlice ++;
			satCalSlice = satCalSlice % 10;
		}
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (recvGNSS == 1)
	{
		GNSS_UartRecv();
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, &gnss_rx, 1);
	}
}

// After an COMPLETED i2c com
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
  isI2CRegAddrRecv = 0;
	if (i2cDataDirection == I2C_DIR_SLAVE_IN)
	{
		if (i2cAddr.u16Addr == 0x0080)
		{
			timeZone = i2cTzSet;
			SaveSysData();
		}
		else if (i2cAddr.u16Addr == 0x0010)
		{
			if (!i2cNewTleValid) return;
			if (i2cAddSatDataIdx < 10)
			{
				strcpy(un_satData.satList[i2cAddSatDataIdx].name, tleTmp.l[0]);
				un_satData.satList[i2cAddSatDataIdx].downlinkFreq = dlTmp;
				un_satData.satList[i2cAddSatDataIdx].uplinkFreq = ulTmp;
				validSatIdx[i2cAddSatDataIdx] = 1;
				Convert_Satellite_Data(tleTmp, &un_satData.satList[i2cAddSatDataIdx].sgpParam);
				SaveSysData();
			}
		}
		else if (i2cAddr.u16Addr == 0x0011)
		{
			if (i2cDelSatDataIdx < 10)
			{
				validSatIdx[i2cDelSatDataIdx] = 0;
				
				SaveSysData();
			}
		}
		else if (i2cAddr.u16Addr == 0x0081)
		{
			i2cNewTleValid = 0;
			i2cPassthrougBufferWritePointer += i2cDataPtrOffset;
			i2cPassthrougBufferWritePointer = i2cPassthrougBufferWritePointer % I2C_PASSTHROUGH_BUFFER_SIZE;
			i2cTleReadStatus = I2C_TLEReader();
			if (i2cTleReadStatus == 0)
			{
				memcpy(tleTmp.l[0], i2cTleRx[0], 71);
				memcpy(tleTmp.l[1], i2cTleRx[1], 71);
				memcpy(tleTmp.l[2], i2cTleRx[2], 71);
				sscanf((char*)i2cTleRx[3], "%d", &ulTmp);
				sscanf((char*)i2cTleRx[4], "%d", &dlTmp);
				i2cNewTleValid = 1;
			}
		}
	}
  HAL_I2C_EnableListen_IT(hi2c); // slave is ready again
}
// After device addr match
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  if(TransferDirection == I2C_DIRECTION_TRANSMIT) 
  {
		i2cDataDirection = I2C_DIR_SLAVE_IN;
    if(isI2CRegAddrRecv == 0) 
    {
			// Ready to receive reg addr
      HAL_I2C_Slave_Seq_Receive_IT(hi2c, &i2cAddr.u8Arr[0], 1, I2C_NEXT_FRAME);
    } 
  } 
  else 
  {
		i2cDataDirection = I2C_DIR_SLAVE_OUT;
		// Transmit data
		HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t*)(i2cDataPtr + i2cDataPtrOffset), 1, I2C_NEXT_FRAME);
  }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (isI2CRegAddrRecv == 0)
	{
		isI2CRegAddrRecv = 1;
		i2cDataPtrOffset = 0;
		HAL_I2C_Slave_Seq_Receive_IT(hi2c, &i2cAddr.u8Arr[1], 1, I2C_NEXT_FRAME);
		i2cPassthrougBufferWritePointer = 0;
		i2cPassthrougBufferReadPointer = 0;
	}
	else if (isI2CRegAddrRecv == 1)
	{
		isI2CRegAddrRecv = 2;
		
		// check addr
		if (i2cAddr.u16Addr < 0x0100)	// 0x0000 misc data
		{
			switch (i2cAddr.u16Addr)
			{
				case 0x0000: i2cDataPtr = (uint8_t*)&i2cReg.i2cSiteInfo; break;		// Site
				case 0x0001: i2cDataPtr = (uint8_t*)&i2cReg.i2cLocalTime; break;	// Time(local, with timezone)
				case 0x0002: i2cDataPtr = gnss_gga; break;												// Last GPGGA message
				case 0x0003: i2cDataPtr = gnss_rmc; break;												// Last GPRMC message
				case 0x0004: i2cDataPtr = &GNSS.valid; break;											// GNSS valid flag
				
				// TLE import
				// 1. Send TLE in specific format (mentioned above) through UART2I2C-PASSTHROUGH
				// 2. If the TLE is valid, the 0x0083: i2cNewTleValid byte is asserted
				// 3. Send desired save slot(0~9) to 0x0010: i2cAddSatDataIdx
				// 4. The last imported TLE is converted and saved in un_satData.satList[i2cAddSatDataIdx]
				// Remove Sat
				// 1. Send the index of the sat to be deleted to 0x0011: i2cDelSatDataIdx
				// 2. The "valid" field in that slot will be set to 0
				case 0x0010: i2cDataPtr = &i2cAddSatDataIdx; break;
				case 0x0011: i2cDataPtr = &i2cDelSatDataIdx; break;
				
				case 0x0012: i2cDataPtr = &i2cSatFocusMode; break;
				case 0x0013: i2cDataPtr = &i2cSatFocusIdx; break;
				
				// Send new Time Zone (-12~+12) to 0x0080: i2cTzSet to set new Time Zone
				case 0x0080: i2cDataPtr = (uint8_t*)&i2cTzSet; break;
				// UART2I2C PASSTHROUGH rx address = 0x0081
				case 0x0081: i2cDataPtr = &i2cPassthroughBuffer[i2cPassthrougBufferWritePointer]; break;
				// Last imported TLE = 0x0082
				case 0x0082: i2cDataPtr = (uint8_t*)i2cTleRx; break;
				case 0x0083: i2cDataPtr = &i2cNewTleValid; break;
			
				default: break;
			};
		}
		else if (i2cAddr.u16Addr < 0x0200)	// 0x0100~0x0109 sat live data
		{
			i2cDataPtr = (uint8_t*)&i2cReg.i2cSatLiveList[i2cAddr.u16Addr - 0x100];
		}
		else if (i2cAddr.u16Addr < 0x0300)	// 0x0200~0x0209 sat pred data
		{
			i2cDataPtr = (uint8_t*)&i2cReg.i2cSatPredict120mList[i2cAddr.u16Addr - 0x200];
		}
		HAL_I2C_Slave_Seq_Receive_IT(hi2c, (uint8_t*)(i2cDataPtr + i2cDataPtrOffset), 1, I2C_NEXT_FRAME);
	}
	else
	{
		i2cDataPtrOffset ++;
		HAL_I2C_Slave_Seq_Receive_IT(hi2c, (uint8_t*)(i2cDataPtr + i2cDataPtrOffset), 1, I2C_NEXT_FRAME);
	}
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  i2cDataPtrOffset++;
  HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t*)(i2cDataPtr + i2cDataPtrOffset), 1, I2C_NEXT_FRAME);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
