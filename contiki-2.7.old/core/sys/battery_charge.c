
#include "contiki.h"
#include "battery_charge.h"
#define DEBUG DEBUG_NONE
#include "net/uip-debug.h"

/*---------------------------------------------------------------------------*/
PROCESS(battery_charge_process, "battery charge process");
AUTOSTART_PROCESSES(&battery_charge_process);
/*---------------------------------------------------------------------------*/
#def Battery_RAKHMATOV 0
#if BATTERY_RAKHMATOV
	uint32_t AL(uint32_t Nu);

#else /* BATTERY_RAKHMATOV */
	uint64_t battery_long_init = (uint64_t)_ALPHA * (uint64_t)1000 * (uint64_t)60000;
	uint64_t battery_long_value = (uint64_t)_ALPHA * (uint64_t)1000 * (uint64_t)60000;
#endif	/* BATTERY_RAKHMATOV */

static uint8_t main_powered = 0;
static uint32_t last_cpu=0, last_lpm=0, last_transmit=0, last_listen=0;

static uint32_t S_Ik_dk=0, S_In_1_dn_1=0, sigmaLn_1=0;

/* Cumulative sum of current by time in ms */
static uint64_t S_Ik_dk_ms=0;

void set_main_powered(int m){
  main_powered = m;
}


void battery_charge_set(void)
{
  if(main_powered == 1){
    battery_charge_value = MAX_ENERGY;
    return;
  }

  SENSORS_ACTIVATE(battery_sensor);

  energest_flush();

  uint32_t cpu = energest_type_time(ENERGEST_TYPE_CPU) - last_cpu;
  uint32_t lpm = energest_type_time(ENERGEST_TYPE_LPM) - last_lpm;
  uint32_t transmit = energest_type_time(ENERGEST_TYPE_TRANSMIT) - last_transmit;
  uint32_t listen = energest_type_time(ENERGEST_TYPE_LISTEN) - last_listen;

 
  last_cpu = energest_type_time(ENERGEST_TYPE_CPU);
  last_lpm = energest_type_time(ENERGEST_TYPE_LPM);
  last_transmit = energest_type_time(ENERGEST_TYPE_TRANSMIT);
  last_listen = energest_type_time(ENERGEST_TYPE_LISTEN);

  SENSORS_DEACTIVATE(battery_sensor);

  /* Compute times in ms */
  uint32_t cpu_t =      (cpu      * 1000) / RTIMER_ARCH_SECOND;
  //uint32_t lpm_t =      (lpm      * 1000) / RTIMER_ARCH_SECOND;
  uint32_t lpm_t =  (uint32_t)  ((uint64_t) ((uint64_t)lpm      * 1000) / RTIMER_ARCH_SECOND);
  uint32_t transmit_t = (transmit * 1000) / RTIMER_ARCH_SECOND;
  uint32_t listen_t =   (listen   * 1000) / RTIMER_ARCH_SECOND;


  /* Sometimes the lpm is not refreshed */
  /* TODO this is a bug */
  if (lpm_t > ((uint32_t)DELTA + ((uint32_t)DELTA/(uint32_t)10)))
    lpm_t = (uint32_t)DELTA - cpu_t - transmit_t - listen_t;

  /*
  ANNOTATE("# %lu %lu %lu %lu %lu\n",(clock_time() * (uint32_t)1000) / CLOCK_SECOND,
	   cpu_t, lpm_t,
	   transmit_t,
	   listen_t);
  */
  ///*
  /*ANNOTATE("# %lu %lu %lu %lu\n",
	   cpu_t, lpm_t, 
	   transmit_t, 
	   listen_t);*/
  //*/

  //int bs = battery_sensor.value(0);
  //ANNOTATE("#B=%ld\n",(int)bs*(int32_t)45000/(int)4096);
  
  /* Compute current in CURRENT_UNIT of mA.ms  */

  uint32_t p_cpu = (cpu_t       * (uint32_t)C_CPU);
  uint32_t p_tx  = (transmit_t  * (uint32_t)C_TX);
  uint32_t p_lst = (listen_t    * (uint32_t)C_LST);
  uint32_t p_lpm = (lpm_t       * (uint32_t)C_LPM);

  /* Total current in CURRENT_UNIT of mA */
  
  uint32_t I_k = (uint32_t)C_CPU + (uint32_t)C_LPM;
  if (listen_t != 0)
    I_k += (uint32_t)C_LST;
  if (transmit_t != 0)
    I_k += (uint32_t)C_TX;

  //ANNOTATE("I_k %lu\n",I_k);

  /* current in micro A.ms */ 

  uint32_t Ikdk = p_cpu + p_lpm + p_lst + p_tx;

  // ANNOTATE("Ik_dk %lu\n",Ikdk);

   /* Sum of Ik.dk in micro A.ms */

#if BATTERY_RAKHMATOV

  S_Ik_dk_ms += Ikdk;
  
  /* Sum of Ik.dk and Ik-1.dk-1 in micro A.ms */

  S_In_1_dn_1 = S_Ik_dk;

  /* Convert sum current by tim in milli A.mn */ /* I think it's converted in (micro) uA.mn instead --- Remark KAMGUEU*/

  S_Ik_dk = (S_Ik_dk_ms)/((uint32_t)TIME_UNIT*(uint32_t)CURRENT_UNIT);
  //ANNOTATE("S_Ikdk %lu\n",S_Ik_dk);

  
  uint32_t A = 0;
  if (lpm_t > (listen_t + transmit_t))
    A = AL(lpm_t -(listen_t + transmit_t));

   //ANNOTATE("AL() %lu\n",A);

   uint32_t  sigmaLn = (uint32_t)(S_Ik_dk + (uint32_t)(2*A*I_k)/(uint32_t)1000);

   //ANNOTATE("sig term %lu\n",sigmaLn);

  uint32_t lambda;
  uint8_t slbd = 0;
  if (sigmaLn_1 >= S_In_1_dn_1){
    lambda = ((uint32_t)LAMBDA*(sigmaLn_1 - S_In_1_dn_1));
    slbd = 1;
  }
  else{
    lambda = ((uint32_t)LAMBDA * (S_In_1_dn_1 - (sigmaLn_1)));
    slbd=2;
  }
  lambda/=((uint32_t)LAMBDA_FACTOR*(uint32_t)1); 

  //ANNOTATE("lambda term %lu\n",lambda);

  /* slbd == 2 when the lambda term is negative */
  if (slbd == 2){
    if (sigmaLn < lambda)
      sigmaLn = (lambda - sigmaLn);
    else
      sigmaLn = (sigmaLn - lambda);
  }
  else
    sigmaLn = (sigmaLn + lambda);
 
  
  sigmaLn_1 = sigmaLn;

  //ANNOTATE("sigma(Ln) %lu\n", sigmaLn);
  
  if ((uint32_t)ALPHA >= sigmaLn){

    uint32_t est = (uint32_t)ALPHA - sigmaLn;

    //ANNOTATE("EST = %lu\n",est);

    uint32_t rest = (uint32_t)((uint64_t)((uint64_t)est*(uint64_t)((uint64_t)MAX_ENERGY*(uint64_t)PRECISION))/(uint64_t)ALPHA);
    uint32_t pe = rest / (uint32_t)PRECISION;
    uint32_t dec = rest - (pe * (uint32_t)PRECISION); 
    battery_charge_value = (uint8_t)pe;
    battery_charge_dec_value=dec;
    ANNOTATE("#A E=%lu.%04lu\n",pe, dec);
    /*printf("lpm_t=%lu/%lu cpu_t=%lu/%lu tx_t=%lu/%lu rx_t=%lu/%lu\n", lpm,lpm_t, cpu,cpu_t, transmit,transmit_t, listen,listen_t);
    printf("p_lpm=%lu p_cpu=%lu p_tx=%lu p_rx=%lu, ikdk=%lu\n", p_lpm, p_cpu, p_tx, p_lst, Ikdk);*/

  } else{
    ANNOTATE("#A E=0\n");
    battery_charge_value = 0;
    //exit(0);
  }
#else /* BATTERY_RAKHMATOV */

  battery_long_value = battery_long_value - (uint64_t) Ikdk;
  /*
  uint32_t mb, lb;
  mb = (uint32_t)(battery_long_value >> 32);
  lb = (uint32_t)(battery_long_value & 0xFFFFFFFF);
  printf("Battery mb = %lu \n", mb);
  printf("Battery lb = %lu \n",  lb);*/

  uint32_t rest = (uint32_t)((battery_long_value * MAX_ENERGY * PRECISION)/battery_long_init);
  uint32_t pe = rest / (uint32_t)PRECISION;
  uint32_t dec = rest - (pe * (uint32_t)PRECISION);
  battery_charge_value = (uint8_t)pe;
  battery_charge_dec_value=dec;
  ANNOTATE("#A E=%lu.%04lu\n",pe, dec);
  /*printf("lpm_t=%lu/%lu cpu_t=%lu/%lu tx_t=%lu/%lu rx_t=%lu/%lu\n", lpm,lpm_t, cpu,cpu_t, transmit,transmit_t, listen,listen_t);
  printf("p_lpm=%lu p_cpu=%lu p_tx=%lu p_rx=%lu, ikdk=%lu\n", p_lpm, p_cpu, p_tx, p_lst, Ikdk);*/

#endif /* BATTERY_RAKHMATOV */

}


/*---------------------------------------------------------------------------*/
PROCESS_THREAD(battery_charge_process, ev, data)
{
  PROCESS_BEGIN();
  static struct etimer period_timer;
  battery_charge_value=MAX_ENERGY;
  etimer_set(&period_timer, CLOCK_SECOND * BATTERY_CHARGE_PERIOD);
  while(1) {
    PROCESS_WAIT_EVENT();
    if(ev == PROCESS_EVENT_TIMER) {
      if(data == &period_timer) {
	battery_charge_set();
        etimer_reset(&period_timer);
      }
    }
  }

  PROCESS_END();
}
