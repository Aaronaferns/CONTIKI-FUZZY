#include "contiki.h"
#include "battery.h"
#define DEBUG DEBUG_NONE
#include "net/uip-debug.h"

extern uint8_t battery_charge_value;
void battery_charge_set()
{
  uint32_t energy_consumption;
  uint32_t battery_charge;
  energest_flush();


  uint32_t cpu = energest_type_time(ENERGEST_TYPE_CPU);
  uint32_t cpu_idle = energest_type_time(ENERGEST_TYPE_LPM);
  uint32_t tx = energest_type_time(ENERGEST_TYPE_TRANSMIT);
  uint32_t rx = energest_type_time(ENERGEST_TYPE_LISTEN);

  uint32_t energy = ((tx * current_tx + rx * current_rx + \
   cpu * current_cpu + cpu_idle * current_cpu_idle) * 3L) /32768;  
  
  energy_consumption=max_energy-energy/CURRENT_UNIT; 
  battery_charge= (energy_consumption*255/max_energy)<<24;
  battery_charge_value=255- (uint8_t)battery_charge;
  
}
