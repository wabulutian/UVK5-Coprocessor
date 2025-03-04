/* sgp_conv.c -- a part of the SGP - C Library
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
 * ---------------------------- double_value ------------------------- *
 * ------------------------------------------------------------------- */

double double_value(char *c1, int pos, int len)
{
	double d = 0;
	register int i;
	int j=0;
	for (i=0;(i<len);i++)
	{
		switch (c1[i+pos]) {
		case ' ' :
			break;
		case '0' :
			d *= 10;
			break;
		case '1' :
			d *= 10;
			d++;
			break;
		case '2' :
			d *= 10;
			d += 2;
			break;
		case '3' :
			d *= 10;
			d += 3;
			break;
		case '4' :
			d *= 10;
			d += 4;
			break;
		case '5' :
			d *= 10;
			d += 5;
			break;
		case '6' :
			d *= 10;
			d += 6;
			break;
		case '7' :
			d *= 10;
			d += 7;
			break;
		case '8' : 
			d *= 10;
			d += 8;
			break;
		case '9' :
			d *= 10;
			d += 9;
			break;
		case '.' :
			j = len-i-1;
			break;
		default : 
			break;
		}
	}
	d/=pow (10.0, j);
	return d;
}



/* ------------------------------------------------------------------- *
 * --------------------------- integer_value ------------------------- *
 * ------------------------------------------------------------------- */

int integer_value(char *c1, int pos, int len)
{
	int d = 0;
	register int i;
	for (i=0;(i<len);i++)
	{
		switch (c1[i+pos]) {
		case ' ' :
			break;
		case '0' :
			d *= 10;
			break;
		case '1' :
			d *= 10;
			d++;
			break;
		case '2' :
			d *= 10;
			d += 2;
			break;
		case '3' :
			d *= 10;
			d += 3;
			break;
		case '4' :
			d *= 10;
			d += 4;
			break;
		case '5' :
			d *= 10;
			d += 5;
			break;
		case '6' :
			d *= 10;
			d += 6;
			break;
		case '7' :
			d *= 10;
			d += 7;
			break;
		case '8' : 
			d *= 10;
			d += 8;
			break;
		case '9' :
			d *= 10;
			d += 9;
			break;
		default : 
			break;
		}
	}
	return d;
}



/* ------------------------------------------------------------------- *
 * ----------------------- Convert_Satellite_Data -------------------- *
 * ------------------------------------------------------------------- */

void Convert_Satellite_Data (struct tle_ascii tle, struct sgp_data *data)
{
	int iexp, ibexp;
	double a1,ao,del1,delo,xnodp,temp, xke;
	
	/* first line */
	data->epoch = double_value(tle.l[1], 18, 14);
	data->epochyr = floor(data->epoch*1.0E-3);
	data->epochyr = data->epochyr < 57 ? (data->epochyr + 2000) : (data->epochyr + 1900);
	data->julian_epoch = Julian_Date_of_Epoch(data->epoch);
	data->xndt2o = double_value(tle.l[1], 33, 10);
	data->xndd6o = double_value(tle.l[1], 44, 6) * 1.0E-5;
	iexp = integer_value(tle.l[1], 50, 2);
	data->bstar = double_value(tle.l[1], 53, 6) * 1.0E-5;
	ibexp = integer_value(tle.l[1], 59, 2);

	/* second line */
	data->xincl = double_value(tle.l[2], 8, 8);
	data->xnodeo = double_value(tle.l[2], 17, 8);
	data->eo = double_value(tle.l[2], 26, 7) * 1E-7;
	data->omegao = double_value(tle.l[2], 34, 8);
	data->xmo = double_value(tle.l[2], 43, 8);
	data->xno = double_value(tle.l[2], 52, 11);
	
	/* Convert to proper units */
	data->xndd6o = (data->xndd6o) * pow (10.0, -iexp);
	data->bstar = (data->bstar) * pow (10.0, -ibexp) / _ae;
	data->xnodeo = radians(data->xnodeo);
	data->omegao = radians(data->omegao);
	data->xmo = radians(data->xmo);
	data->xincl = radians(data->xincl);
	data->xno = (data->xno) * 2.0 * _pi / _xmnpda;
	data->xndt2o = (data->xndt2o) * 2.0 * _pi / sqr (_xmnpda);
	data->xndd6o = (data->xndd6o) * 2.0 * _pi / cube (_xmnpda);

	/* Determine whether Deep-Space Model is needed */
	xke = sqrt((3600.0 * ge) / cube(xkmper));
	a1 = pow ((xke/(data->xno)),(2.0/3.0));
	temp = 1.5 * CK2 * ( 3 * sqr ( cos (data->xincl)) - 1.0 ) / pow ( ( 1.0 - sqr (data->eo) ), 1.5);
	del1 = temp / (sqr (a1));
	ao = a1 * ( 1 - del1 * (0.5 * (2.0/3.0) + del1 * (1.0 + (134.0/81.0)* del1 )));
	delo = temp / (sqr (ao));
	xnodp = (data->xno) / (1 + delo);
	if (((2 * _pi) / xnodp) >= 225) 
	    data->ideep = 1;
	else
	    data->ideep = 0;
}



/* ------------------------------------------------------------------- *
 * ------------------------- Convert_Sat_State ----------------------- *
 * ------------------------------------------------------------------- */

void Convert_Sat_State (struct vector *p, struct vector *v)
{
	register int i;
	for (i=0;(i<3);i++)
	{
		p->v[i] = p->v[i] * xkmper;
		v->v[i] = v->v[i] * xkmper / 60;
	}
	Magnitude (p);
	Magnitude (v);
}



// /* ------------------------------------------------------------------- *
//  * --------------------------------- sgp ----------------------------- *
//  * ------------------------------------------------------------------- */

// int sgp (int mode, double time, struct tle_ascii tle, struct vector *pos, struct vector *vel)
// {
// 	struct sgp_data satdata;
// 	Convert_Satellite_Data (tle, &satdata);
// 	switch (mode)
// 	{
// 	    case _SGP0 :
// 		sgp0call(time, pos, vel, &satdata);
// 		break;
// 	    case _SGP4 :
// 	    case _SDP4 :
// 		sgp4call(time, pos, vel, &satdata);
// 		break;
// 	    case _SGP8 :
// 	    case _SDP8 :
// 		sgp8call(time, pos, vel, &satdata);
// 		break;
// 	    default :
// 		return -1;
// 	}
// 	Convert_Sat_State (pos, vel);
// 	return 0;
// }
