/* sgp_math.c -- a part of the SGP - C Library
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
 * -------------------------------- cube ----------------------------- *
 * ------------------------------------------------------------------- */

__inline_double cube(double x)
{
	x = pow(x , 3.0);
	return x;
}



/* ------------------------------------------------------------------- *
 * --------------------------------- sqr ---------------------------- *
 * ------------------------------------------------------------------- */

__inline_double sqr(double x)
{
	x *= x;
	return x;
}



/* ------------------------------------------------------------------- *
 * ------------------------------ modulus ---------------------------- *
 * ------------------------------------------------------------------- */

__inline_double modulus (double arg1, double arg2)
{
	double modu;
	modu = arg1 - floor(arg1/arg2) * arg2;
	if (modu >= 0)
		return modu;
	else
	{
		modu += arg2;
		return modu;
	}
	return -1; /* shouldn't happen */
}



/* ------------------------------------------------------------------- *
 * ------------------------------- fmod2p ---------------------------- *
 * ------------------------------------------------------------------- */

__inline_double fmod2p (double x)
{
	x = modulus (x, 2.0 * _pi );
	return x;
}



/* ------------------------------------------------------------------- *
 * ------------------------------- radians --------------------------- *
 * ------------------------------------------------------------------- */

__inline_double radians (double x)
{
	x *= _pi / 180.0;
	return x;
}



/* ------------------------------------------------------------------- *
 * ------------------------------ Magnitude -------------------------- *
 * ------------------------------------------------------------------- */

__inline_void Magnitude (struct vector *v)
{
	v->v[3] = sqrt ( sqr (v->v[0]) + sqr (v->v[1]) + sqr(v->v[2]));
}


/* ------------------------------------------------------------------- *
 * ------------------------------ Magnitude -------------------------- *
 * ------------------------------------------------------------------- */

__inline_double actan (double y, double x)
{
	double t = atan2(y,x);
	if (t < 0)
		t += 2 * _pi;
	return t;
}

/* -----------------------------------------------------------------------------
*
*                           function dot
*
*  this function finds the dot product of two vectors.
*
*  author        : david vallado                  719-573-2600    1 mar 2001
*
*  inputs          description                    range / units
*    vec1        - vector number 1
*    vec2        - vector number 2
*
*  outputs       :
*    dot         - result
*
*  locals        :
*    none.
*
*  coupling      :
*    none.
*
* --------------------------------------------------------------------------- */

__inline_double dot (double x[3], double y[3])
{
	return (x[0]*y[0] + x[1]*y[1] + x[2]*y[2]);
}  // end dot
