/* sgp_time.c -- a part of the SGP - C Library
 *
 * Copyright (c) 2001-2002 Dominik Brodowski
 *           (c) 1992-2000 Dr TS Kelso
 *
 * This file is part of the SGP C Library.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * This SGP C Library is based on the SGP4 Pascal Library by Dr TS Kelso.
 *
 * You can reach Dominik Brodowski by electronic mail at
 * mail@brodo.de
 * and by paper mail at 
 * Karlstrasse 11a; 72072 Tuebingen; Germany
 */



/* ------------------------------------------------------------------- *
 * ------------------------------ includes --------------------------- *
 * ------------------------------------------------------------------- */

#include "math.h"
#include "sgp.h"
#include "sgp_int.h"



/* ------------------------------------------------------------------- *
 * ------------------------ Julian_Date_of_Year ---------------------- *
 * ------------------------------------------------------------------- */

double Julian_Date_of_Year(double year)
{
	/* Astronomical Formulae for Calculators, Jean Meeus, pages 23-25 
	   Calculate Julian Date of 0.0 Jan year */
	double A, B, JDoY;
	year = year - 1.0;
	A = floor(year / 100.0);
	B  = 2.0 - A + floor(A / 4.0);
	JDoY = floor(365.25 * year) + floor(30.6001 * 14.0) + 1720994.5 + B;
	return JDoY;
}



/* ------------------------------------------------------------------- *
 * ----------------------- Julian_Date_of_Epoch ---------------------- *
 * ------------------------------------------------------------------- */

double Julian_Date_of_Epoch(double epoch)
{
	double year, day, JDoE;
	/* Valid 1957 through 2056 */
	year = floor(epoch*1.0E-3);
	if (year < 57)
		year += 2000;
	else
		year += 1900;
	day  = epoch - (floor(epoch*1.0E-3))*1.0E3;
	JDoE = Julian_Date_of_Year(year) + day;
	return JDoE;
}



/* ------------------------------------------------------------------- *
 * -------------------------------- ThetaG --------------------------- *
 * ------------------------------------------------------------------- */

double ThetaG(double epoch, double *ds50)
{
	/* Reference:  The 1992 Astronomical Almanac, page B6. */
	double year, day, UT, jd, TU, GMST, TG;
	/* Modification to support Y2K - Valid 1957 through 2056 */
	year = floor(epoch*1.0E-3);
	if (year < 57)
		year = year + 2000;
	else
		year = year + 1900;
	/* End modification */
	day  = epoch - (floor(epoch*1.0E-3))*1.0E3;
	UT   = day - floor(day);
	day  = floor(day);
	jd   = Julian_Date_of_Year(year) + day;
	TU   = (jd - 2451545.0)/36525;
	GMST = 24110.54841 + TU * (8640184.812866 + TU * (0.093104 - TU * 6.2E-6));
	GMST = modulus(GMST + _secday * _omega_E * UT , _secday);
	TG = 2.0 * _pi * GMST / _secday;
	*ds50 = jd - 2433281.5 + UT;
	/* TG := modulus((6.3003880987*ds50 + 1.72944494),(2*_pi)); */
	return TG;
}
