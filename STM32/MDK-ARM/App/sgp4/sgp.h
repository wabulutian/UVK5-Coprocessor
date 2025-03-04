/* sgp.h -- a part of the SGP - C Library
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
 * --------------------------- definitions --------------------------- *
 * ------------------------------------------------------------------- */

#define _SGP0 0
#define _SGP4 1
#define _SDP4 2
#define _SGP8 3
#define _SDP8 4
/* when you call the function int sgp(int mode,...), you can pass one of 
 * these as first parameter - "mode". This makes your code more readable.
 */



/* ------------------------------------------------------------------- *
 * ----------------------------- structs ----------------------------- *
 * ------------------------------------------------------------------- */

struct vector {
	double v[4];
};
/* 3-dimensional vector, the fourth element contains the magnitude 
 */

struct tle_ascii {
	char l[3][71];
};
/* a standard three-line Two-Line-Element. Yes, that's right: the first
 * line usually is a header stating the name of the object and sometimes
 * additional information, and the second and the third line form the
 * classic "Two-Line-Element".
 */

struct sgp_data {
	double epoch;				//8
	double epochyr;			//8
	double julian_epoch;//8
	double xno;					//8
	double bstar;				//8
	double xincl;				//8
	double eo;					//8
	double xmo;					//8
	double omegao;			//8
	double xnodeo;			//8
	double xndt2o;			//8
	double xndd6o;			//8
	int ideep;					//4
};	//100
/* Convert_Satellite_Data(...) transforms the ascii-Two-Line-Element 
 * into binary values and saves them in this struct.
 */



// /* ------------------------------------------------------------------- *
//  * --------------------------- prototypes ---------------------------- *
//  * ------------------------------------------------------------------- */


// int sgp (int mode, double time, struct tle_ascii tle, struct vector *pos, struct vector *vel);
// /* Whenever you want to access any of the SGP, SGP4/SDP4 or SGP8/SDP8
//  * models, it is strongly recommended that you use this function as
//  * only interface. 
//  *
//  * INPUT:
//  * int                 mode :   this specifies the model you want to use 
//  *                              for the calculation. Please use _SGP0 for 
//  *                              SGP, _SGP4 or _SDP4 for SGP4/SDP4, and
//  *	 	                _SGP8 or _SDP8 for SGP8/SDP8.
//  * double              time :   julian time for which the position and
//  *                              veloicity shall be determined.
//  * struct tle_ascii    tle  :   input data - a standard TwoLine Element in
//  *                              lines 2 and 3; line 1 (usually including
//  *                              object name and additional information)
//  *                              is not used by sgp-c-lib.
//  *
//  * OUTPUT:
//  * struct vector       *pos :   postion of object at "time" - in km.
//  * struct vector       *vel :   veloicity of object at "time" - in km/s
//  */


double Julian_Date_of_Epoch(double epoch);

double Julian_Date_of_Epoch(double epoch);
/* Julian_Date_of_Epoch returns the julian date of the date specified by 
 * the "TLE-style epoch" value given as parameter. This is especially
 * useful for correctly determing the "time" parameter for sgp(...)
 */

void Convert_Satellite_Data (struct tle_ascii tle, struct sgp_data *data);
/* Convert_Satellite_Data tranforms an ascii-TLE to the binary struct
 * sgp_data needed for all propagation models
 */

void Convert_Sat_State (struct vector *p, struct vector *v);
/* Convert_Sat_State transfers the output to km, km/s and calculates the
 * magnitude of these two vectors.
 */


// /* sgp0.c */
// void sgp0 (double tsince, struct vector *pos, struct vector *vel, struct sgp_data *satdata);
// void sgp0call (double time, struct vector *pos, struct vector *vel, struct sgp_data *satdata);
// /* sgp0 and sgp0call offer direct access to calculations with the SGP model. 
//  * Note that all parameters have to be in the appropriate units. Use of this 
//  * direct access is strongly discouraged. */


/* sgp4sdp4.c */
void sgp4 (double tsince, struct vector *pos, struct vector *vel, struct sgp_data *satdata);
// void sdp4 (double tsince, struct vector *pos, struct vector *vel, struct sgp_data *satdata);
// void sgp4call (double time, struct vector *pos, struct vector *vel, struct sgp_data *satdata);
// /* sgp4, sdp4 and sgp0call offer direct access to calculations with the 
//  * SGP4/SDP4 model. Note that all parameters have to be in the appropriate
//  * units. Use of this direct access is strongly discouraged. */


// /* sgp8sdp8.c */
// void sgp8 (double tsince, struct vector *pos, struct vector *vel, struct sgp_data *satdata);
// void sdp8 (double tsince, struct vector *pos, struct vector *vel, struct sgp_data *satdata);
// void sgp8call (double time, struct vector *pos, struct vector *vel, struct sgp_data *satdata);
// /* sgp8, sdp8 and sgp0call offer direct access to calculations with the 
//  * SGP8/SDP8 model. Note that all parameters have to be in the appropriate
//  * units. Use of this direct access is strongly discouraged. */


/* ------------------------------------------------------------------- */
/* ----------------------- COMPILATION TWEAKS ------------------------ */
/* ------------------------------------------------------------------- */

/* "inlining" of functions */
/*   Some compilers (e.g. gcc) allow the inlining of functions into the
 *   calling routine. This means a lot of overhead can be removed, and
 *   the execution of the program runs much faster. However, the filesize
 *   and thus the loading time is increased.
 */
#ifdef INLINE
#define __inline_double inline double
#define __inline_void inline void
#else
#define __inline_double double
#define __inline_void void
#endif
