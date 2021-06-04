

#include "dev/battery-sensor.h"

/* number of mA per mesured current */
#define CURRENT_UNIT 10000
/* current values mA * CURENT_UNIT */
#define C_CPU 18000
#define C_TX  177000
#define C_LST 200000
#define C_LPM 545


/* ratio 10000 milliseconds / minutes */
#define TIME_UNIT 6

/* Current mutliplicator to accelarate current consumption */
#define CURRENT_FACTOR 1

#define MAX_ENERGY 255
/* alpha parameter : initial charge */
#if BATTERY_RAKHMATOV
	#define _ALPHA 120000

#else /* BATTERY_RAKHMATOV */
	#define _ALPHA 120000
#endif	/* BATTERY_RAKHMATOV */


/* precision of charge level */
#define _FACTOR (10*CURRENT_UNIT)
#define ALPHA (_ALPHA*10000)

#define PRECISION 10000

/* period for energy update (ms) */
#define DELTA (1000*BATTERY_CHARGE_PERIOD)
/* Lambda : e^-(BETA^2*(DELTA/60000)) */
/* LAMBDA = Lambda * LAMBDA_FACTOR             */
#define LAMBDA 1353 //967
#define LAMBDA_FACTOR 1000
/* Beta : battery parameter */
/* BETA = 10 * Beta         */
#define BETA 10
/* C_0 = S_m:1->inf (e^-Beta^2 * m^2 * Delta / (Beta^2 * m^2)) */
/* C0 = 1000 * C_0                                             */
#define C0 135 //1337

/* a : average Nu in minute */
/* _a = 1000 * a            */
#define _A 1800 //30
/* square root of a in minutes */
/* Sqrt_a = 1000 * sqrt(a)         */
#define SQRT_A 1341 //173
/* 1/(2 * sqrt(a)) in minutes    */
/* inv2Sqrt_a = 1000 / (2 * sqrt(a)) */
#define INV2SQRT_A 372 //2886


/* square root of Pi *1000 */
#define SQRT_PI 1772
/* Pi root * 1000 */
#define PI2 9869



#define BATTERY_CHARGE_PERIOD 120

PROCESS_NAME(battery_charge_process);

uint8_t battery_charge_value;
uint32_t battery_charge_dec_value;
void set_main_powered(int m);

