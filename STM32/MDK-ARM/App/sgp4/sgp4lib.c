#include "sgp4lib.h"

//satPass_t passList[MAX_PASS] = {0};

/***-----------------------------------------
 *   Functions for datetime converting
 *-------------------------------------------
*/

double getJulianFromUnix(double unixSecs)
{
   return ( unixSecs / 86400.0 ) + 2440587.5;
}

int isLeapYear(int year){
  return (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
}

int GetDaysInMonth(int year, int month) {
    static const int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (month == 2 && isLeapYear(year)) {
        return 29;
    }
    return daysInMonth[month - 1];
}

/***-----------------------------------------
 *   End of datetime converting functions
 *-------------------------------------------
*/

/***-----------------------------------------
 *   Functions for post-process SGP4 data
 *-------------------------------------------
*/

double  mag(double x[3])
{
    return sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]);
}  // end mag

double  sgn(double x)
{
    if (x < 0.0)
    {
        return -1.0;
    }
    else
    {
        return 1.0;
    }

}  // end sgn

double floatmod(double a, double b)
{
    return (a - b * floor(a / b));
}

double  gstime(double jdut1)
{
    const double twopi = 2.0 * pi;
    const double deg2rad = pi / 180.0;
    double       temp, tut1;

    tut1 = (jdut1 - 2451545.0) / 36525.0;
    temp = -6.2e-6* tut1 * tut1 * tut1 + 0.093104 * tut1 * tut1 +
            (876600.0*3600 + 8640184.812866) * tut1 + 67310.54841;  // sec
    temp = floatmod(temp * deg2rad / 240.0, twopi); //360/86400 = 1/240, to deg, to rad

    // ------------------------ check quadrants ---------------------
    if (temp < 0.0)
        temp += twopi;

    return temp;
}  // end gstime
void rot3(double invec[3], double xval, double outvec[3])
{
    double temp = invec[1];
    double c = cos(xval);
    double s = sin(xval);
    
    outvec[1] = c*invec[1] - s*invec[0];
    outvec[0] = c*invec[0] + s*temp;
    outvec[2] = invec[2];
}

void rot2(double invec[3], double xval, double outvec[3])
{
    double temp = invec[2];
    double c = cos(xval);
    double s = sin(xval);
    
    outvec[2] = c*invec[2] + s*invec[0];
    outvec[0] = c*invec[0] - s*temp;
    outvec[1] = invec[1];
}
/*
polarm

This function calulates the transformation matrix that accounts for polar
motion. Polar motion coordinates are estimated using IERS Bulletin
rather than directly input for simplicity.

Author: David Vallado, 2007
Ported to C++ by Grady Hillhouse with some modifications, July 2015.

INPUTS          DESCRIPTION                     RANGE/UNITS
jdut1           Julian date                     days

OUTPUTS         DESCRIPTION
pm              Transformation matrix for ECEF - PEF
*/

void polarm(double jdut1, double pm[3][3])
{
    double MJD; //Julian Date - 2,400,000.5 days
    double A;
    double C;
    double xp; //Polar motion coefficient in radians
    double yp; //Polar motion coefficient in radians
    
    //Predict polar motion coefficients using IERS Bulletin - A (Vol. XXVIII No. 030)
    MJD = jdut1 - 2400000.5;
    A = 2 * pi * (MJD - 57226) / 365.25;
    C = 2 * pi * (MJD - 57226) / 435;
    
    xp = (0.1033 + 0.0494*cos(A) + 0.0482*sin(A) + 0.0297*cos(C) + 0.0307*sin(C)) * 4.84813681e-6;
    yp = (0.3498 + 0.0441*cos(A) - 0.0393*sin(A) + 0.0307*cos(C) - 0.0297*sin(C)) * 4.84813681e-6;
    
    pm[0][0] = cos(xp);
    pm[0][1] = 0.0;
    pm[0][2] = -sin(xp);
    pm[1][0] = sin(xp) * sin(yp);
    pm[1][1] = cos(yp);
    pm[1][2] = cos(xp) * sin(yp);
    pm[2][0] = sin(xp) * cos(yp);
    pm[2][1] = -sin(yp);
    pm[2][2] = cos(xp) * cos(yp);
}

/*
ijk2ll

This function calulates the latitude, longitude and altitude
given the ECEF position matrix.

Author: David Vallado, 2007
Ported to C++ by Grady Hillhouse with some modifications, July 2015.

INPUTS          DESCRIPTION                     RANGE/UNITS
r               Position matrix (ECEF)          km

OUTPUTS         DESCRIPTION
latlongh        Latitude, longitude, and altitude (rad, rad, and km)
*/

void ijk2ll(double r[3], double latlongh[3])
{
    double twopi = 2.0*pi;
    double small = 0.00000001;          //small value for tolerances
    double re = 6378.137;               //radius of earth in km
    double eesqrd = 0.006694385000;     //eccentricity of earth sqrd
    double magr, temp, rtasc;
    
    magr = mag(r);
    temp = sqrt(r[0]*r[0] + r[1]*r[1]);
 
    if(fabs(temp) < small)
    {
        rtasc = sgn(r[2]) * pi * 0.5;
    }
    else
    {
        rtasc = atan2(r[1], r[0]);
    }
    
    latlongh[1] = rtasc;
    
    if (fabs(latlongh[1]) >= pi)
    {
        if (latlongh[1] < 0.0)
        {
            latlongh[1] += twopi;
        }
        else
        {
            latlongh[1] -= twopi;
        }
    }
    
    latlongh[0] = asin(r[2] / magr);
    
    //Iterate to find geodetic latitude
    int i = 1;
    double olddelta = latlongh[0] + 10.0;
    double sintemp, c = 0;
    
    while ( (fabs(olddelta - latlongh[0]) >= small) && (i < 10) )
    {
        olddelta = latlongh[0];
        sintemp = sin(latlongh[0]);
        c = re / sqrt(1.0 - eesqrd*sintemp*sintemp);
        latlongh[0] = atan( (r[2] + c*eesqrd*sintemp) / temp );
        i++;
    }
    
    if (0.5*pi - fabs(latlongh[0]) > pi/180.0)
    {
        latlongh[2] = (temp/cos(latlongh[0])) - c;
    }
    else
    {
        latlongh[2] = r[2]/sin(latlongh[0]) - c*(1.0 - eesqrd);
    }
}

/*
site

This function finds the position and velocity vectors for a site. The
outputs are in the ECEF coordinate system. Note that the velocity vector
is zero because the coordinate system rotates with the earth.

Author: David Vallado, 2007
Ported to C++ by Grady Hillhouse with some modifications, July 2015.

INPUTS          DESCRIPTION                     RANGE/UNITS
latgd           Site geodetic latitude          -pi/2 to pi/2 in radians
lon             Longitude                       -2pi to 2pi in radians
alt             Site altitude                   km

OUTPUTS         DESCRIPTION
rs              Site position vector            km
vs              Site velocity vector            km/s
*/

void site(double latgd, double lon, double alt, double rs[3])
{
    double sinlat, re, eesqrd, cearth, rdel, rk;
    re = 6378.137;              //radius of earth in km
    eesqrd = 0.006694385000;    //eccentricity of earth sqrd
    
    //Find rdel and rk components of site vector
    sinlat = sin(latgd);
    cearth = re / sqrt( 1.0 - (eesqrd*sinlat*sinlat) );
    rdel = (cearth + alt) * cos(latgd);
    rk = ((1.0 - eesqrd) * cearth + alt ) * sinlat;
    
    //Find site position vector (ECEF)
    rs[0] = rdel * cos( lon );
    rs[1] = rdel * sin( lon );
    rs[2] = rk;
}

/*
teme2ecef

This function transforms a vector from a true equator mean equinox (TEME)
frame to an earth-centered, earth-fixed (ECEF) frame.

Author: David Vallado, 2007
Ported to C++ by Grady Hillhouse with some modifications, July 2015.

INPUTS          DESCRIPTION                     RANGE/UNITS
rteme           Position vector (TEME)          km
vteme           Velocity vector (TEME)          km/s
jdut1           Julian date                     days

OUTPUTS         DESCRIPTION                     RANGE/UNITS
recef           Position vector (ECEF)          km  
vecef           Velocity vector (ECEF)          km/s
*/

void teme2ecef(double rteme[3], double jdut1, double recef[3])
{
    double gmst;
    double st[3][3];
    double rpef[3];
    double pm[3][3];
    
    //Get Greenwich mean sidereal time
    gmst = gstime(jdut1);
    
    //st is the pef - tod matrix
    st[0][0] = cos(gmst);
    st[0][1] = -sin(gmst);
    st[0][2] = 0.0;
    st[1][0] = sin(gmst);
    st[1][1] = cos(gmst);
    st[1][2] = 0.0;
    st[2][0] = 0.0;
    st[2][1] = 0.0;
    st[2][2] = 1.0;
    
    //Get pseudo earth fixed position vector by multiplying the inverse pef-tod matrix by rteme
    rpef[0] = st[0][0] * rteme[0] + st[1][0] * rteme[1] + st[2][0] * rteme[2];
    rpef[1] = st[0][1] * rteme[0] + st[1][1] * rteme[1] + st[2][1] * rteme[2];
    rpef[2] = st[0][2] * rteme[0] + st[1][2] * rteme[1] + st[2][2] * rteme[2];
    
    //Get polar motion vector
    polarm(jdut1, pm);
    
    //ECEF postion vector is the inverse of the polar motion vector multiplied by rpef
    recef[0] = pm[0][0] * rpef[0] + pm[1][0] * rpef[1] + pm[2][0] * rpef[2];
    recef[1] = pm[0][1] * rpef[0] + pm[1][1] * rpef[1] + pm[2][1] * rpef[2];
    recef[2] = pm[0][2] * rpef[0] + pm[1][2] * rpef[1] + pm[2][2] * rpef[2];
}

/*
rv2azel

This function calculates the range, elevation, and azimuth (and their rates)
from the TEME vectors output by the SGP4 function.

Author: David Vallado, 2007
Ported to C++ by Grady Hillhouse with some modifications, July 2015.

INPUTS          DESCRIPTION                     RANGE/UNITS
ro              Sat. position vector (TEME)     km
vo              Sat. velocity vector (TEME)     km/s
latgd           Site geodetic latitude          -pi/2 to pi/2 in radians
lon             Site longitude                  -2pi to 2pi in radians
alt             Site altitude                   km
jdut1           Julian date                     days

OUTPUTS         DESCRIPTION
razel           Range, azimuth, and elevation matrix
*/

void rv2azel(double ro[3], double latgd, double lon, double alt, double jdut1, double razel[3])
{
    //Locals
    double halfpi = pi * 0.5;
    double small  = 0.00000001;
    double temp;
    double rs[3];
    double recef[3];
    double rhoecef[3];
    double tempvec[3];
    double rhosez[3];
    double magrhosez;
    double rho, az, el;
    
    //Get site vector in ECEF coordinate system
    site(latgd, lon, alt, rs);
    
    //Convert TEME vectors to ECEF coordinate system
    teme2ecef(ro, jdut1, recef);
    
    //Find ECEF range vectors
    for (int i = 0; i < 3; i++)
    {
        rhoecef[i] = recef[i] - rs[i];
    }
    rho = mag(rhoecef); //Range in km
    
    //Convert to SEZ (topocentric horizon coordinate system)
    rot3(rhoecef, lon, tempvec);
    rot2(tempvec, (halfpi-latgd), rhosez);

    //Calculate azimuth, and elevation
    temp = sqrt(rhosez[0]*rhosez[0] + rhosez[1]*rhosez[1]);
    if (temp < small)
    {
        el = sgn(rhosez[2]) * halfpi;
        az = NAN;
    }
    else
    {
        magrhosez = mag(rhosez);
        el = asin(rhosez[2]/magrhosez);
        az = atan2(rhosez[1], -rhosez[0]);
    }
    //Move values to output vectors
    razel[0] = rho;             //Range (km)
    razel[1] = az;              //Azimuth (radians)
    razel[2] = el;              //Elevation (radians)
}

/***----------------------------------------
 *   End of Post-Process Functions
 *------------------------------------------
*/

/***----------------------------------------
 *   Utility Functions
 *------------------------------------------
*/

void Days2mdhms(unsigned int year, double days,unsigned int* mon,unsigned int* day,unsigned int* hr,unsigned int* minute, unsigned int* sec){
	int i, inttemp, dayofyr;
	double    temp;
	int lmonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

	dayofyr = (int)floor(days);
	/* ----------------- find month and day of month ---------------- */
	if ( (year % 4) == 0 )
	lmonth[1] = 29;

	i = 1;
	inttemp = 0;
	while ((dayofyr > inttemp + lmonth[i-1]) && (i < 12))
	{
	inttemp = inttemp + lmonth[i-1];
	i++;
	}
	*mon = i;
	*day = dayofyr - inttemp;

	/* ----------------- find hours minutes and seconds ------------- */
	temp = (days - dayofyr) * 24.0;
	*hr   = (int)floor(temp);
	temp = (temp - *hr) * 60.0;
	*minute  = (int)floor(temp);
	*sec  = (temp - *minute) * 60.0;
}

void SetTmTime(tm_t* tm, int yr, int mon, int day, int hour, int min, int sec){
	unsigned int daysInMonth[] = {0,31,28,31,30,31,30,31,31,30,31,30,31};
  long long totalSeconds = 0;
  for (int y = 1970; y < yr; y ++){
    totalSeconds += isLeapYear(y) ? 31622400 : 31536000;
  }
  for (int m =  1; m < mon; m ++){
    totalSeconds += daysInMonth[m] * 86400;
    if (m == 2 && isLeapYear(yr)){
      totalSeconds += 86400;
    }
  }
  totalSeconds += (day - 1) * 86400 + hour * 3600 + min * 60 + sec;

  tm->year = yr;
  tm->month = mon;
  tm->day = day;
  tm->hour = hour;
  tm->min = min;
  tm->sec = sec;
  tm->tv = totalSeconds;
}

void SGP4_AddTime(_U8 addSec, tm_t* src, tm_t* dst) {
    dst->tv = src->tv + addSec;
    
    int seconds = dst->tv % 86400;
    dst->hour = seconds / 3600;
    dst->min = (seconds % 3600) / 60;
    dst->sec = seconds % 60;

    int days = dst->tv / 86400;
    int year = dst->year;
    int month = dst->month;
    int day = dst->day;

    while (days >= GetDaysInMonth(year, month)) {
        days -= GetDaysInMonth(year, month);
        if (++month > 12) {
            month = 1;
            year++;
        }
    }

    day += days;
    while (day > GetDaysInMonth(year, month)) {
        day -= GetDaysInMonth(year, month);
        if (++month > 12) {
            month = 1;
            year++;
        }
    }

    dst->year = year;
    dst->month = month;
    dst->day = day;
}

void SGP4_FindSatStatus(tm_t tm, struct sgp_data *satData, st_siteData *siteData, st_sgpSatStatus *sgpSatStatus)
{
  struct vector pos, vel;
	double tsince;
	double razel[3];
	double latlongh[3];
	double recef[3];
	double jdC;

    // Convert datetime to Julian format
    jdC = getJulianFromUnix(tm.tv);

    // Find satellite epoch time (in second)
    tsince = (jdC - satData->julian_epoch) * 24.0 * 60.0;

    // Find satellite position and velocity vecotr using SGP4 model
    sgp4(tsince, &pos, &vel, satData);

    // Convert position vector to azimuth-elevation
    Convert_Sat_State(&pos,&vel);
    rv2azel(pos.v, siteData->siteLat, siteData->siteLon, siteData->siteAlt, jdC, razel);

    // Convert data format to earth-centered, earth-fixed (ECEF)
    teme2ecef(pos.v, jdC, recef);

    // Find satellite Lat, Lon and Alt
    ijk2ll(recef, latlongh);

    //Save data
    sgpSatStatus->satLat = latlongh[0]*180/pi;                     //Latidude of sattelite (degrees)
    sgpSatStatus->satLon = latlongh[1]*180/pi;                     //longitude of sattelite (degrees)
    sgpSatStatus->satAlt = latlongh[2];                            //Altitude of sattelite (degrees)
    sgpSatStatus->satAz = floatmod( razel[1]*180/pi+360.0, 360.0); //Azemith of sattelite (degrees)
    sgpSatStatus->satEl = razel[2]*180/pi;                         //elevation of sattelite (degrees)
    sgpSatStatus->satDist = razel[0];                              //Distance to sattelite (km)
		
		//Calculate next second for speed
		jdC = getJulianFromUnix(tm.tv + 1);
    tsince = (jdC - satData->julian_epoch) * 24.0 * 60.0;

    // Find satellite position and velocity vecotr using SGP4 model
    sgp4(tsince, &pos, &vel, satData);

    // Convert position vector to azimuth-elevation
    Convert_Sat_State(&pos,&vel);
    rv2azel(pos.v, siteData->siteLat, siteData->siteLon, siteData->siteAlt, jdC, razel);
		
		sgpSatStatus->satSpd = razel[0] - sgpSatStatus->satDist;                              //Speed of sattelite (km/s)
}
