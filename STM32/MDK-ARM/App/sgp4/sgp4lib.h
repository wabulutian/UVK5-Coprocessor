#include "math.h"
#include "sgp.h"
#include "sgp_int.h"

#define pi 3.14159265358979323846
#define MAX_PASS  10

#ifdef  IS_MCU
    #define _U8         uint8_t
    #define _U16        uint16_t
    #define _I16        int16_t
    #define _U32        uint32_t
    #define _I32        int32_t
#else
    #define _U8         unsigned int
    #define _U16        unsigned int
    #define _I16        int
    #define _U32        unsigned int
    #define _I32        int
#endif

typedef struct
{
	double satLat;	//8
	double satLon;	//8
	double satAlt;	//8
	double satAz;		//8
	double satEl;		//8
	double satDist;	//8
	double satSpd;	//8
}st_sgpSatStatus;	//56

typedef struct
{
  _U16 year;		//2
  _U8 month;		//1
  _U8 day;			//1
  _U8 hour;			//1
  _U8 min;			//1
  _U8 sec;			//1
	_U8 zone;			//1
  long long tv;	//8
}tm_t;//16

typedef struct
{
	double siteLat;	//8
	double siteLon;	//8
	double siteAlt;	//8
}st_siteData;		//24

int isLeapYear(int year);
int GetDaysInMonth(int year, int month);
void SGP4_AddTime(_U8 addSec, tm_t* src, tm_t* dst);
void SGP4_SetSite(double latDeg, double lonDeg, double altKm);
void SetTmTime(tm_t* tm, int yr, int mon, int day, int hour, int min, int sec);
void SGP4_FindSatStatus(tm_t tm, struct sgp_data *satData, st_siteData *siteData, st_sgpSatStatus *sgpSatStatus);
