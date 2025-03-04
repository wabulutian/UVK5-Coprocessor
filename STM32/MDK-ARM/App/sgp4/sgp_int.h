/* sgp_int.h -- a part of the SGP - C Library
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
 * ------------------------------ constants -------------------------- *
 * ------------------------------------------------------------------- */

/* defined constants */
#ifndef _xmnpda
#define _xmnpda 1440.0
#define _secday 86400.0
#define _omega_E 1.00273790934
#define _tothrd (2.0/3.0)
#define J2 1.0826158E-3
#define XJ3  -2.53881E-6
#define _J4value -1.65597E-6
#define _e6a 1.0E-6
#define _pi 3.1415926535897932384626433279 
#define _ae 1.0
/* WGS '72 - constants */
/* Earth gravitational constant */
#define ge 398600.8        
/* Earth equatorial radius - kilometers */
#define xkmper 6378.135  
#define CK2 (1.0826158E-3 / 2.0)
#define CK4 (-3.0 * -1.65597E-6 / 8.0)
#endif


/* ------------------------------------------------------------------- *
 * ------------------------------- structs --------------------------- *
 * ------------------------------------------------------------------- */

struct val_deep_init {
	double eosq, sinio, cosio, betao, aodp, theta2;
	double sing, cosg, betao2, xmdot, omgdot;
	double xnodott, xnodpp;
};

struct val_deep_sec {
	double xmdf, omgadf, xnode, emm, xincc, xnn, tsince;
};

struct val_deep_per {
	double e, xincc, omgadf, xnode, xmam;
};



/* ------------------------------------------------------------------- *
 * ------------------------ function prototypes ---------------------- *
 * ------------------------------------------------------------------- */

/* sgp_math.c */
double cube(double x);
double sqr(double x);
double fmod2p(double x);
double modulus (double arg1, double arg2);
double radians(double x);
void Magnitude (struct vector *v);
double actan (double y, double x);
double dot (double x[3], double y[3]);
/* sgp_time.c */
double ThetaG(double epoch, double *ds50);

/* sgp_deep.c */
void deep(int ideep, struct sgp_data *satdata);
void call_dpinit(struct val_deep_init *values, struct sgp_data *satdata);
void call_dpsec(struct val_deep_sec *values, struct sgp_data *satdata);
void call_dpper(struct val_deep_per *values, struct sgp_data *satdata);




