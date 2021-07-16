#ifndef BATTERY_H
#define BATTERY_H

/* number of mA per mesured current */
#define CURRENT_UNIT 10000
/* current values mA * CURENT_UNIT */
#define current_cpu 5000
#define current_tx  17400
#define current_rx 18800
#define current_cpu_idle 5

/* ratio 10000 milliseconds / minutes */
#define TIME_UNIT 6

/* Current mutliplicator to accelarate current consumption */
#define CURRENT_FACTOR 1

#define MAX_ENERGY 255
//capacity in ma 
#define capacity 4000
// max_energy 
#define max_energy capacity*3

uint8_t battery_charge_value;
void battery_charge_set();

#endif
