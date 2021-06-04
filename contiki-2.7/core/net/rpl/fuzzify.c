

#include "fuzzify.h"

unsigned int DOWN(unsigned int x1, unsigned int x2, unsigned int X){
	return (TRUE * ((long)(X) - (long)(x2)))/((long)(x1) - (long)(x2));
}

unsigned int UP(unsigned int x1, unsigned int x2, unsigned int X){
	return DOWN(x2, x1, X);
}

unsigned int FIRST_T_NORM(unsigned int b1, unsigned int b2, unsigned int px){
  if (px <= b1) return TRUE;
  if (px > b1 && px <= b2) return DOWN(b1, b2, px);
  return FALSE;
}

unsigned int LAST_T_NORM(unsigned int b1, unsigned int b2, unsigned int px){
  if (px <= b1) return FALSE;
  if (px > b1 && px <= b2) return UP(b1, b2, px);
  return TRUE;
}

unsigned int T_NORM(unsigned int b1, unsigned int b2, unsigned int b3, unsigned int b4, unsigned int px){
  if (px <= b1) return FALSE;
  if (px > b1 && px <= b2) return UP(b1, b2, px);
  if (px > b2 && px <= b3) return TRUE;
  if (px > b3 && px <= b4) return DOWN(b3, b4, px);
  return FALSE;
}


#define min(x,y) ((x)<(y))?(x):(y)
#define max(x,y) ((x)<(y))?(y):(x)

/**********************************************
 *      QoS = FUZZY_COMBINATION(ETX,DELAY,HOPCOUNT)    *
 **********************************************/

unsigned int qos(unsigned int e, unsigned int d, unsigned int hc){

  unsigned int c1 = etx_long(e);
  unsigned int c2 = etx_avg(e);
  unsigned int c3 = etx_short(e);
  unsigned int l1 = dly_high(d);
  unsigned int l2 = dly_avg(d);
  unsigned int l3 = dly_small(d);
  unsigned int p1= hopcount_high(hc);
  unsigned int p2= hopcount_avg(hc);
  unsigned int p3= hopcount_low(hc);
  unsigned int b1 = 0, b2 = 0, b3 = 0, b4 = 0, b5 = 0;

  if(c3 && l3 && p3) b5 = min(c3,l3,p3);
  if ((c3 && l2 && p3) || (c3 && l3 && p2)|| (c3 && l2 && p2)|| (c3 && l3 && p1)|| (c3 && l2 && p1)|| (c2 && l3 && p3)|| (c2 && l3 && p2) || (c2 && l3 && p1)) 
  b4 = max(min(c3,l2,p3),min(c3,l3,p2), min(c3,l2,p2) ,min(c3,l3,p1) ,min(c3,l2,p1) ,min(c2,l3,p3) ,min(c2,l3,p2) ,min(c2,l3,p1));
  if ((c3 && l1 && p3) || (c3 && l1 && p2)|| (c3 && l1 && p1)|| (c2 && l2 && p3)|| (c2 && l2 && p1)|| (c1 && l3 && p3)|| (c1 && l3 && p2) || (c1 && l3 && p1))
  b3 = max(min(c3,l2,p3),min(c3,l1,p2), min(c3,l1,p1) ,min(c2,l2,p3) ,min(c2,l2,p1) ,min(c1,l3,p3) ,min(c1,l3,p2) ,min(c1,l3,p1));
  if ((c2 && l1 && p3) || (c2 && l1 && p2)|| (c2 && l1 && p1)|| (c1 && l2 && p3)|| (c1 && l1 && p3)|| (c1 && l2 && p2)|| (c1 && l1 && p2) || (c1 && l2 && p1)) 
  b2 = max(min(c2,l1,p3),min(c2,l1,p2), min(c2,l1,p1) ,min(c1,l2,p3) ,min(c1,l1,p3) ,min(c1,l2,p2) ,min(c1,l1,p2) ,min(c1,l2,p1));
  if(c1 && l1 && p1) 
  b1 = min(c1,l1,p3);

  return (b1*10 + b2*30 + b3*50 + b4*70 + b5*90)/(b1 + b2 + b3 + b4 + b5);
}


/***************************************************
 *      Quality = FUZZY_COMBINATION(QoS,ENERGY)    *
 ***************************************************/

unsigned int quality(unsigned int q, unsigned int e){

	  unsigned int vsqos = qos_very_slow(q);
	  unsigned int sqos = qos_slow(q);
	  unsigned int aqos = qos_avg(q);
	  unsigned int fqos = qos_fast(q);
	  unsigned int vfqos = qos_very_fast(q);

	  unsigned int leng = energy_low(e);
	  unsigned int meng = energy_medium(e);
	  unsigned int feng = energy_full(e);

	  unsigned int awf = 0, bad = 0, deg = 0, avg = 0, accep = 0, good = 0, exc = 0;

	  if (vsqos && leng) awf += min(vsqos, leng);
	  if (vsqos && meng) bad = min(vsqos, meng) < bad ? bad : min(vsqos, meng);
	  if (vsqos && feng) avg = min(vsqos, feng) < avg ? avg : min(vsqos, feng);
	  if (sqos && leng) bad = min(sqos, leng) < bad ? bad : min(sqos, leng);
	  if (sqos && meng) deg = min(sqos, meng) < deg ? deg : min(sqos, meng);
	  if (sqos && feng) avg = min(sqos, feng) < avg ? avg : min(sqos, feng);
	  if (aqos && leng) bad = min(aqos, leng) < bad ? bad : min(aqos, leng);
	  if (aqos && meng) avg = min(aqos, meng) < avg ? avg : min(aqos, meng);
	  if (aqos && feng) accep = min(aqos, feng) < accep ? accep : min(aqos, feng);
	  if (fqos && leng) deg = min(fqos, leng) < deg ? deg : min(fqos, leng);
	  if (fqos && meng) accep = min(fqos, meng) < accep ? accep : min(fqos, meng);
	  if (fqos && feng) good = min(fqos, feng) < good ? good : min(fqos, feng);
	  if (vfqos && leng) avg = min(vfqos, leng) < avg ? avg : min(vfqos, leng);
	  if (vfqos && meng) good = min(vfqos, meng) < good ? good : min(vfqos, meng);
	  if (vfqos && feng) exc = min(vfqos, feng) < exc ? exc : min(vfqos, feng);

	  return (awf*8 + bad*21 + deg*35 + avg*49 + accep*63 + good*77 + exc*91)/(awf + bad + deg + avg + accep + good + exc);
}

