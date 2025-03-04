/* sgp4sdp4.c -- a part of the SGP - C Library
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
 * ----------------------------- includes ---------------------------- *
 * ------------------------------------------------------------------- */

#include "math.h"
#include "sgp.h"
#include "sgp_int.h"



/* ------------------------------------------------------------------- *
 * ------------------------------- sgp4 ------------------------------ *
 * ------------------------------------------------------------------- */

void sgp4 (double tsince, struct vector *pos, struct vector *vel, struct sgp_data *satdata)
{
	struct vector MV, NV, UV, VV;
	double a1=0, a3ovk2=0, ao=0, aodp=0, aycof=0;
        double betao=0, betao2=0, c1=0, c1sq=0;
	double c2=0, c3=0, c4=0, c5=0, coef=0, coef1=0;
	double cosio=0, d2=0, d3=0, d4=0, del1=0, delmo=0;
	double delo=0, eeta=0, eosq=0, eta=0, etasq=0, qoms24=0;
	double omgcof=0, omgdot=0, perige=0, pinvsq=0, psisq=0;
	double s4=0, sinio=0, sinmo=0, t2cof=0, t3cof=0, t4cof=0;
	double t5cof=0, temp=0, temp1=0, temp2=0, temp3=0, theta2=0;
	double theta4=0, tsi=0, x1m5th=0, x1mth2=0, x3thm1=0, x7thm1=0;
	double xhdot1=0, xlcof=0, xmcof=0, xmdot=0, xnodcf=0, xnodot=0;
	double xnodp=0;
	int isimp=0, i=0;
	double rfdotk;
	double rdotk,xinck,xnodek,uk,rk;
	double cos2u,sin2u,u,sinu,cosu,betal,rfdot,rdot,r,pl,elsq;
	double esine,ecose,epw,temp6,temp5,temp4,cosepw,sinepw;
	double capu,ayn,xlt,aynl,xll,axn,xn,beta,xl,e,a,tfour;
	double tcube,delm,delomg,templ,tempe,tempa,xnode,tsq,xmp;
	double omega,xnoddf,omgadf,xmdf;
	double xke=0, s=0, qo=0, qoms2t=0, temp7=0;
        /* Constants */
	s = _ae + 78 / xkmper;
	qo = _ae + 120 / xkmper;
	xke = sqrt((3600.0 * ge) / cube(xkmper));
	qoms2t = sqr ( sqr (qo - s));
	temp2 = xke / (satdata->xno);
	a1 = pow (temp2 , _tothrd);
	cosio = cos (satdata->xincl);
	theta2 = sqr(cosio);
	x3thm1 = 3.0 * theta2 - 1.0;
	eosq = sqr (satdata->eo);
	betao2 = 1.0 - eosq;
	betao = sqrt (betao2);
	del1 = 1.5 * CK2 * x3thm1 / (sqr(a1) * betao * betao2);
	ao = a1 * ( 1.0 - del1*((1.0/3.0) + del1 * (1.0 + (134.0/81.0) * del1)));
	delo = 1.5 * CK2 * x3thm1 / (sqr(ao) * betao * betao2);
	xnodp = (satdata->xno)/(1.0 + delo);
	aodp = ao/(1.0 - delo);
        /* Initialization */
	/* For perigee less than 220 kilometers, the isimp flag is set and
	   the equations are truncated to linear variation in sqrt a and
	   quadratic variation in mean anomaly.  Also, the c3 term, the
	   delta omega term, and the delta m term are dropped. */
	isimp = 0;
	if ((aodp * (1.0 - satdata->eo)/ _ae) < (220.0/xkmper + _ae))
	    isimp = 1;
	/* For perigee below 156 km, the values of s and qoms2t are altered.*/
	s4 = s;
	qoms24 = qoms2t;
	perige = (aodp * (1.0 - satdata->eo) - _ae) * xkmper;
	if (perige < 156)
	{
	    s4 = perige - 78.0;
	    if (perige <= 98)
		s4 = 20.0;
	    qoms24 = pow (((120.0 - s4) * _ae / xkmper), 4.0);
	    s4 = s4 / xkmper + _ae;
	}
	pinvsq = 1.0 / ( sqr(aodp) * sqr(betao2) );
	tsi = 1.0 / (aodp - s4);
	eta = aodp * (satdata->eo) * tsi;
	etasq = sqr (eta);
	eeta = (satdata->eo) * eta;
	psisq = fabs( 1.0 - etasq);
	coef = qoms24 * pow( tsi, 4.0);
	coef1 = coef / pow ( psisq, 3.5);
	c2 = coef1 * xnodp * (aodp * (1.0 + 1.5 * etasq + eeta * (4.0 + etasq)) + 0.75 * CK2 * tsi / psisq * x3thm1 * (8.0 + 3.0 * etasq * (8.0 + etasq)));
	c1 = (satdata->bstar) * c2;
	sinio = sin(satdata->xincl);
	a3ovk2 = -XJ3 / CK2 * pow( _ae , 3.0);
	c3 = coef * tsi * a3ovk2 * xnodp * _ae * sinio / (satdata->eo);
	x1mth2 = 1.0 - theta2;
	c4 = 2.0 * xnodp * coef1 * aodp * betao2 * ( eta * (2.0 + 0.5 * etasq) + (satdata->eo) * (0.5 + 2.0 * etasq) - 2.0 * CK2 * tsi / (aodp * psisq) * ( -3.0 * x3thm1 * ( 1.0 - 2.0 * eeta + etasq * (1.5 - 0.5*eeta)) + 0.75 * x1mth2 * (2.0 * etasq - eeta * (1.0 + etasq)) * cos(2.0 * (satdata->omegao))));
	c5 = 2.0 * coef1 * aodp * betao2 * (1.0 + 2.75 * (etasq + eeta) + eeta * etasq);
	theta4 = sqr(theta2);
	temp1 = 3.0 * CK2 * pinvsq * xnodp;
	temp2 = temp1 * CK2 * pinvsq;
	temp3 = 1.25 * CK4 * pinvsq * pinvsq * xnodp;
	xmdot = xnodp + 0.5 * temp1 * betao * x3thm1 + 0.0625 * temp2 * betao * (13.0 - 78.0 * theta2 + 137.0 * theta4);
	x1m5th = 1.0 - 5.0 * theta2;
	omgdot = -0.5 * temp1 * x1m5th + 0.0625 * temp2 * (7.0 - 114.0 * theta2 + 395.0 * theta4) + temp3 * (3.0 - 36.0 * theta2 + 49.0 * theta4);
	xhdot1 = -temp1 * cosio;
	xnodot = xhdot1 + (0.5 * temp2 * (4.0 - 19.0 * theta2) + 2.0 * temp3 * (3.0 - 7.0 * theta2)) * cosio;
	omgcof = (satdata->bstar) * c3 * cos(satdata->omegao);
	xmcof = -(2.0/3.0) * coef * (satdata->bstar) * _ae / eeta;
	xnodcf = 3.5 * betao2 * xhdot1 * c1;
	t2cof = 1.5 * c1;
	xlcof = 0.125 * a3ovk2 * sinio * (3.0 + 5.0 * cosio) / (1.0 + cosio);
	aycof = 0.25 * a3ovk2 * sinio;
	delmo = pow ((1.0 + eta * cos(satdata->xmo)), 3);
	sinmo = sin(satdata->xmo);
	x7thm1 = 7.0 * theta2 - 1.0;
	if (isimp==0)
	{
	    c1sq = sqr(c1);
	    d2 = 4.0 * aodp * tsi * c1sq;
	    temp = d2 * tsi * c1 / 3.0;
	    d3 = (17.0 * aodp + s4)*temp;
	    d4 = 0.5 * temp * aodp * tsi * (221.0 * aodp + 31.0 * s4) * c1;
	    t3cof = d2 + 2.0*c1sq;
	    t4cof = 0.25 * (3.0 * d3 + c1 * (12.0 * d2 + 10.0 * c1sq));
	    t5cof = 0.2 * (3.0 * d4 + 12.0 * c1 * d3 + 6.0 * d2 * d2 + 15.0 * c1sq * (2.0 * d2 + c1sq));
	}
	/* Update for secular gravity and atmospheric drag. */
	xmdf = satdata->xmo + xmdot * tsince;
	omgadf = satdata->omegao + omgdot * tsince;
	xnoddf = satdata->xnodeo + xnodot * tsince;
	omega = omgadf;
	xmp = xmdf;
	tsq = sqr (tsince);
	xnode = xnoddf + xnodcf * tsq;
	tempa = 1.0 - c1 * tsince;
	tempe = (satdata->bstar) * c4 * tsince;
	templ = t2cof * tsq;
	if (isimp == 0)
	{
	    delomg = omgcof * tsince;
	    delm = xmcof*(pow((1.0 + eta * cos(xmdf)), 3.0) - delmo);
	    temp = delomg + delm;
	    xmp = xmdf + temp;
	    omega = omgadf - temp;
	    tcube = tsq * tsince;
	    tfour = tsince * tcube;
	    tempa = tempa - d2 * tsq - d3 * tcube - d4 * tfour;
	    tempe = tempe + (satdata->bstar) * c5 * (sin(xmp) - sinmo);
	    templ = templ + t3cof * tcube + tfour * (t4cof + tsince * t5cof);
	}
	a = aodp * sqr(tempa);
	e = (satdata->eo) - tempe;
	xl = xmp + omega + xnode + xnodp*templ;
	beta = sqrt(1.0 - sqr(e));
	xn = xke / pow(a,1.5);
	/* Long period periodics */
	axn = e * cos(omega);
	temp = 1.0 / (a * sqr(beta));
	xll = temp * xlcof * axn;
	aynl = temp * aycof;
	xlt = xl + xll;
	ayn = e * sin(omega) + aynl;
	/* Solve Kepler's Equation */
	capu = fmod2p(xlt - xnode);
	temp2 = capu;
	i=1;
	do {
	    sinepw = sin(temp2);
	    cosepw = cos(temp2);
	    temp3 = axn * sinepw;
	    temp4 = ayn * cosepw;
	    temp5 = axn * cosepw;
	    temp6 = ayn * sinepw;
	    epw = (capu - temp4 + temp3 - temp2) / (1.0 - temp5 - temp6) + temp2;
	    temp7 = temp2;
	    temp2 = epw;
	    i++;
	} while ((i<=10) && (fabs(epw - temp7) > _e6a));
	/* Short period preliminary quantities */
	ecose = temp5 + temp6;
	esine = temp3 - temp4;
	elsq = sqr(axn) + sqr(ayn);
	temp = 1.0 - elsq;
	pl = a * temp;
	r = a * (1.0 - ecose);
	temp1 = 1.0 / r;
	rdot = xke * sqrt(a) * esine * temp1;
	rfdot = xke * sqrt(pl) * temp1;
	temp2 = a * temp1;
	betal = sqrt(temp);
	temp3 = 1.0 / (1.0 + betal);
	cosu = temp2 * (cosepw - axn + ayn * esine * temp3);
	sinu = temp2 * (sinepw - ayn - axn * esine * temp3);
	u = actan(sinu, cosu);
	sin2u = 2.0 * sinu * cosu;
	cos2u = 2.0 * sqr(cosu) - 1.0;
	temp = 1.0 / pl;
	temp1 = CK2 * temp;
	temp2 = temp1 * temp;
        /* Update for short periodics */
	rk = r * (1.0 - 1.5 * temp2 * betal * x3thm1) + 0.5 * temp1 * x1mth2 * cos2u;
	uk = u - 0.25 * temp2 * x7thm1 * sin2u;
	xnodek = xnode + 1.5 * temp2 * cosio * sin2u;
	xinck = (satdata->xincl) + 1.5 * temp2 * cosio * sinio * cos2u;
	rdotk = rdot - xn * temp1 * x1mth2 * sin2u;
	rfdotk = rfdot + xn * temp1 * (x1mth2 * cos2u + 1.5 * x3thm1);
	/* Orientation vectors */
	MV.v[0] = -sin(xnodek) * cos(xinck);
	MV.v[1] = cos(xnodek) * cos(xinck);
	MV.v[2] = sin(xinck);
	
	NV.v[0] = cos(xnodek);
	NV.v[1] = sin(xnodek);
	NV.v[2] = 0;
	for (i=0;(i<3);i++)
	{
		UV.v[i] = MV.v[i] * sin(uk) + NV.v[i] * cos(uk);
		VV.v[i] = MV.v[i] * cos(uk) - NV.v[i] * sin(uk);
        }

        /* position + velocity */
	for (i=0;(i<3);i++)
	{
		pos->v[i] = rk * UV.v[i];
		vel->v[i] = rdotk * UV.v[i] + rfdotk * VV.v[i];
        }
}



/* ------------------------------------------------------------------- *
 * ------------------------------- sdp4 ------------------------------ *
 * ------------------------------------------------------------------- */

void sdp4 (double tsince, struct vector *pos, struct vector *vel, struct sgp_data *satdata)
{
	struct val_deep_init val_dpinit;
	struct val_deep_sec val_dpsec;
	struct val_deep_per val_dpper;
	struct vector MV, NV, UV, VV;
	double a1=0, a3ovk2=0, ao=0, aodp=0, aycof=0, betao=0, betao2=0;
	double c1=0, c2=0, c4=0, coef=0, coef1=0, cosg=0, cosio=0, del1=0;
	double delo=0, eeta=0, eosq=0, eta=0, etasq=0, omgdot=0;
	double perige=0, pinvsq=0, psisq=0, qoms24=0, s4=0, sing=0;
	double sinio=0, t2cof=0, temp1=0, temp2=0, temp3=0, theta2=0;
	double theta4=0, tsi=0, x1m5th=0, x1mth2=0, x3thm1=0, x7thm1=0;
	double xhdot1=0, xlcof=0, xmdot=0, xnodcf=0, xnodot=0, xnodp=0;
	int i;
	double a,axn,ayn,aynl,beta,betal,capu,cos2u,cosepw;
	double cosu,e,ecose,elsq,em=0,epw,esine,omgadf;
	double pl,r,rdot,rdotk,rfdot,rfdotk,rk,sin2u,sinepw;
	double sinu,temp,temp4,temp5,temp6,tempa;
	double tempe,templ,tsq,u,uk,xinc=0,xinck;
	double xl,xll,xlt,xmam,xmdf,xn,xnoddf,xnode,xnodek;
	double temp7;
	double xke=0, s=0, qo=0, qoms2t=0;
        /* Constants */
	s = _ae + 78 / xkmper;
	qo = _ae + 120 / xkmper;
	xke = sqrt((3600.0 * ge) / cube(xkmper));
	qoms2t = sqr ( sqr (qo - s));
	temp2 = xke / (satdata->xno);
	a1 = pow (temp2 , _tothrd);
	cosio = cos (satdata->xincl);
	theta2 = sqr(cosio);
	x3thm1 = 3.0 * theta2 - 1.0;
	eosq = sqr (satdata->eo);
	betao2 = 1.0 - eosq;
	betao = sqrt (betao2);
	del1 = 1.5 * CK2 * x3thm1 / (sqr(a1) * betao * betao2);
	ao = a1 * ( 1.0 - del1*((1.0/3.0) + del1 * (1.0 + (134.0/81.0) * del1)));
	delo = 1.5 * CK2 * x3thm1 / (sqr(ao) * betao * betao2);
	xnodp = (satdata->xno)/(1 + delo);
	aodp = ao/(1 - delo);
	/* For perigee below 156 km, the values of s and qoms2t are altered.*/
	s4 = s;
	qoms24 = qoms2t;
	perige = (aodp * (1.0 - satdata->eo) - _ae) * xkmper;
	if (perige < 156)
	{
	    s4 = perige - 78.0;
	    if (perige <= 98)
		s4 = 20.0;
	    qoms24 = pow (((120.0 - s4) * _ae / xkmper), 4.0);
	    s4 = s4 / xkmper + _ae;
	}
	pinvsq = 1.0 / ( sqr(aodp) * sqr(betao2) );
	sing = sin(satdata->omegao);
	cosg = cos(satdata->omegao);
	tsi = 1.0 / (aodp - s4);
	eta = aodp * (satdata->eo) * tsi;
	etasq = sqr (eta);
	eeta = (satdata->eo) * eta;
	psisq = fabs( 1.0 - etasq);
	coef = qoms24 * pow( tsi, 4.0);
	coef1 = coef / pow ( psisq, 3.5);
	c2 = coef1 * xnodp * (aodp * (1.0 + 1.5 * etasq + eeta * (4.0 + etasq)) + 0.75 * CK2 * tsi / psisq * x3thm1 * (8.0 + 3.0 * etasq * (8.0 + etasq)));
	c1 = (satdata->bstar) * c2;
	sinio = sin(satdata->xincl);
	a3ovk2 = -XJ3 / CK2 * pow( _ae , 3.0);
	x1mth2 = 1.0 - theta2;
	c4 = 2.0 * xnodp * coef1 * aodp * betao2 * ( eta * (2.0 + 0.5 * etasq) + (satdata->eo) * (0.5 + 2.0 * etasq) - 2.0 * CK2 * tsi / (aodp * psisq) * ( -3.0 * x3thm1 * ( 1.0 - 2.0 * eeta + etasq * (1.5 - 0.5*eeta)) + 0.75 * x1mth2 * (2.0 * etasq - eeta * (1.0 + etasq)) * cos(2.0 * (satdata->omegao))));
	theta4 = sqr(theta2);
	temp1 = 3.0 * CK2 * pinvsq * xnodp;
	temp2 = temp1 * CK2 * pinvsq;
	temp3 = 1.25 * CK4 * pinvsq * pinvsq * xnodp;
	xmdot = xnodp + 0.5 * temp1 * betao * x3thm1 + 0.0625 * temp2 * betao * (13.0 - 78.0 * theta2 + 137.0 * theta4);
	x1m5th = 1.0 - 5.0 * theta2;
	omgdot = -0.5 * temp1 * x1m5th + 0.0625 * temp2 * (7.0 - 114.0 * theta2 + 395.0 * theta4) + temp3 * (3.0 - 36.0 * theta2 + 49.0 * theta4);
	xhdot1 = -temp1 * cosio;
	xnodot = xhdot1 + (0.5 * temp2 * (4.0 - 19.0 * theta2) + 2.0 * temp3 * (3.0 - 7.0 * theta2)) * cosio;
	xnodcf = 3.5 * betao2 * xhdot1 * c1;
	t2cof = 1.5 * c1;
	xlcof = 0.125 * a3ovk2 * sinio * (3.0 + 5.0 * cosio) / (1.0 + cosio);
	aycof = 0.25 * a3ovk2 * sinio;
	x7thm1 = 7.0 * theta2 - 1.0;

	val_dpinit.eosq=eosq;
	val_dpinit.sinio=sinio;
	val_dpinit.cosio=cosio;
	val_dpinit.betao=betao;
	val_dpinit.aodp=aodp;
	val_dpinit.theta2=theta2;
	val_dpinit.sing=sing;
	val_dpinit.cosg=cosg;
	val_dpinit.betao2=betao2;
	val_dpinit.xmdot=xmdot;
	val_dpinit.omgdot=omgdot;
	val_dpinit.xnodott=xnodot;
	val_dpinit.xnodpp=xnodp;
	call_dpinit(&val_dpinit, satdata);
	eosq=val_dpinit.eosq;
	sinio=val_dpinit.sinio;
	cosio=val_dpinit.cosio;
	betao=val_dpinit.betao;
	aodp=val_dpinit.aodp;
	theta2=val_dpinit.theta2;
	sing=val_dpinit.sing;
	cosg=val_dpinit.cosg;
	betao2=val_dpinit.betao2;
	xmdot=val_dpinit.xmdot;
	omgdot=val_dpinit.omgdot;
	xnodot=val_dpinit.xnodott;
	xnodp=val_dpinit.xnodpp;

	/* Update for secular gravity and atmospheric drag. */
	xmdf = satdata->xmo + xmdot * tsince;
	omgadf = satdata->omegao + omgdot * tsince;
	xnoddf = satdata->xnodeo + xnodot * tsince;
	tsq = sqr (tsince);
	xnode = xnoddf + xnodcf * tsq;
	tempa = 1.0 - c1 * tsince;
	tempe = (satdata->bstar) * c4 * tsince;
	templ = t2cof * tsq;
	xn = xnodp;

	val_dpsec.xmdf=xmdf;
	val_dpsec.omgadf=omgadf;
	val_dpsec.xnode=xnode;
	val_dpsec.emm=em;
	val_dpsec.xincc=xinc;
	val_dpsec.xnn=xn;
	val_dpsec.tsince=tsince;
	call_dpsec(&val_dpsec, satdata);
	xmdf=val_dpsec.xmdf;
	omgadf=val_dpsec.omgadf;
	xnode=val_dpsec.xnode;
	em=val_dpsec.emm;
	xinc=val_dpsec.xincc;
	xn=val_dpsec.xnn;
	tsince=val_dpsec.tsince;

	a = pow((xke/xn),_tothrd) * sqr(tempa);
	e = em - tempe;
	xmam = xmdf + xnodp * templ;

	val_dpper.e=e;
	val_dpper.xincc=xinc;
	val_dpper.omgadf=omgadf;
	val_dpper.xnode=xnode;
	val_dpper.xmam=xmam;
	call_dpper(&val_dpper, satdata);
	e=val_dpper.e;
	xinc=val_dpper.xincc;
	omgadf=val_dpper.omgadf;
	xnode=val_dpper.xnode;
	xmam=val_dpper.xmam;

	xl = xmam + omgadf + xnode;
	beta = sqrt(1.0 - sqr(e));
	xn = xke / pow(a,1.5);
        /* Long period periodics */
	axn = e * cos(omgadf);
	temp = 1.0 / (a * sqr(beta));
	xll = temp * xlcof * axn;
	aynl = temp * aycof;
	xlt = xl + xll;
	ayn = e * sin(omgadf) + aynl;
	/* Solve Kepler's Equation */
	capu = fmod2p(xlt - xnode);
	temp2 = capu;
	i=1;
	do {
	    sinepw = sin(temp2);
	    cosepw = cos(temp2);
	    temp3 = axn * sinepw;
	    temp4 = ayn * cosepw;
	    temp5 = axn * cosepw;
	    temp6 = ayn * sinepw;
	    epw = (capu - temp4 + temp3 - temp2) / (1.0 - temp5 - temp6) + temp2;
	    temp7 = temp2;
	    temp2 = epw;
	    i++;
	} while ((i<=10) && (fabs(epw - temp7) > _e6a));

	/* Short period preliminary quantities */
	ecose = temp5 + temp6;
	esine = temp3 - temp4;
	elsq = sqr(axn) + sqr(ayn);
	temp = 1.0 - elsq;
	pl = a * temp;
	r = a * (1.0 - ecose);
	temp1 = 1.0 / r;
	rdot = xke * sqrt(a) * esine * temp1;
	rfdot = xke * sqrt(pl) * temp1;
	temp2 = a * temp1;
	betal = sqrt(temp);
	temp3 = 1.0 / (1.0 + betal);
	cosu = temp2 * (cosepw - axn + ayn * esine * temp3);
	sinu = temp2 * (sinepw - ayn - axn * esine * temp3);
	u = actan(sinu, cosu);
	sin2u = 2.0 * sinu * cosu;
	cos2u = 2.0 * sqr(cosu) - 1.0;
	temp = 1.0 / pl;
	temp1 = CK2 * temp;
	temp2 = temp1 * temp;
        /* Update for short periodics */
	rk = r * (1.0 - 1.5 * temp2 * betal * x3thm1) + 0.5 * temp1 * x1mth2 * cos2u;
	uk = u - 0.25 * temp2 * x7thm1 * sin2u;
	xnodek = xnode + 1.5 * temp2 * cosio * sin2u;
	xinck = xinc + 1.5 * temp2 * cosio * sinio * cos2u;
	rdotk = rdot - xn * temp1 * x1mth2 * sin2u;
	rfdotk = rfdot + xn * temp1 * (x1mth2 * cos2u + 1.5 * x3thm1);
	/* Orientation vectors */
	MV.v[0] = -sin(xnodek) * cos(xinck);
	MV.v[1] = cos(xnodek) * cos(xinck);
	MV.v[2] = sin(xinck);
	
	NV.v[0] = cos(xnodek);
	NV.v[1] = sin(xnodek);
	NV.v[2] = 0;
	for (i=0;(i<3);i++)
	{
		UV.v[i] = MV.v[i] * sin(uk) + NV.v[i] * cos(uk);
		VV.v[i] = MV.v[i] * cos(uk) - NV.v[i] * sin(uk);
        }

        /* position + velocity */
	for (i=0;(i<3);i++)
	{
		pos->v[i] = rk * UV.v[i];
		vel->v[i] = rdotk * UV.v[i] + rfdotk * VV.v[i];
        }
}



// /* ------------------------------------------------------------------- *
//  * ----------------------------- sgp4call ---------------------------- *
//  * ------------------------------------------------------------------- */

// void sgp4call (double time, struct vector *pos, struct vector *vel, struct sgp_data *satdata)
// {
// 	double tsince;
// 	tsince = ( time - (satdata->julian_epoch) ) * _xmnpda;
// 	if ((satdata->ideep)==0)
// 	    sgp4 (tsince, pos, vel, satdata);
// 	else
// 	    sdp4 (tsince, pos, vel, satdata);
// }

