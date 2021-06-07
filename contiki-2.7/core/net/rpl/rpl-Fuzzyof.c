/**
 * 
 *         Objective Function with multiple metrics combined using Fuzzy Logic  (Fuzzyof)
 *
 *         This implementation uses the estimated number of 
 *         transmissions (ETX), Delay, Hopcount and Node Energy metrics.
 *
 * 
 * 
 **/
#include "fis.h"
#include "sys/energest.h"
#include "net/delay.h"
#include "net/rpl/rpl-private.h"
#include "net/nbr-table.h"

#define DEBUG DEBUG_NONE
#include "net/uip-debug.h"

#if CONTIKI_DELAY
#define DLY_SCALE	100
#define DLY_ALPHA	90
#endif /* CONTIKI_DELAY */

#define HOPCOUNT_MAX 255
#define QUALITY_MAX 100
#define QUALITY_RANK_DIVISOR 10
#define DEFAULT_RANK_INCREMENT  RPL_MIN_HOPRANKINC
#define PARENT_SWITCH_THRESHOLD 2

static void reset(rpl_dag_t *);
static void neighbor_link_callback(rpl_parent_t *, int, int, delay_t);
static rpl_parent_t *best_parent(rpl_parent_t *, rpl_parent_t *);
static rpl_dag_t *best_dag(rpl_dag_t *, rpl_dag_t *);
static rpl_rank_t calculate_rank(rpl_parent_t *, rpl_rank_t);
static void update_metric_container(rpl_instance_t *);

rpl_of_t rpl_Fuzzyof = {
  reset,
  neighbor_link_callback,
  best_parent,
  best_dag,
  calculate_rank,
  update_metric_container,
  1
};

/* Constants for the ETX moving average */
#define ETX_SCALE   100
#define ETX_ALPHA   90

/* Reject parents that have a higher link metric than the following. */
#define MAX_LINK_METRIC			10

/* Reject parents that have a higher path cost than the following. */
#define MAX_PATH_COST			100

/*
 * The rank must differ more than 1/PARENT_SWITCH_THRESHOLD_DIV in order
 * to switch preferred parent.
 */
#define PARENT_SWITCH_THRESHOLD_DIV	2

typedef uint16_t rpl_path_metric_t;

static rpl_path_metric_t
calculate_etx_path_metric(rpl_parent_t *p)
{
  if(p == NULL) {
    return MAX_PATH_COST * RPL_DAG_MC_ETX_DIVISOR;
  }
  return p->mc.obj.etx + (uint16_t)p->link_metric;
}

static uint8_t
calculate_energy_path_metric(rpl_parent_t *p){

	if (p == NULL) return 0;
	return (battery_charge_value >= p->mc.obj.energy.energy_est) ? p->mc.obj.energy.energy_est : battery_charge_value;
}

static rpl_path_metric_t
calculate_hopcount_path_metric(rpl_parent_t *p)
{
	if(p == NULL /*|| (p->mc.obj.hopcount == 0 && p->rank > ROOT_RANK(p->dag->instance))*/) {
	   return HOPCOUNT_MAX;
	} else
		return 1 + p->mc.obj.hopcount;
}

static rpl_path_metric_t
calculate_latency_path_metric(rpl_parent_t *p){

	if(p == NULL /*|| (p->mc.obj.latency == 0 && p->rank > ROOT_RANK(p->dag->instance))*/) {
	   return MAX_DELAY;
	} else {
		uint16_t local_delay = p->delay_metric;
		return p->mc.obj.latency + local_delay;
	}

}

static rpl_path_metric_t
calculate_fuzzy_metric(rpl_parent_t *p){
  return quality(
    qos(
      p->mc.obj.etx/RPL_DAG_MC_ETX_DIVISOR,
      p->mc.obj.latency,
      p->mc.obj.hopcount
    ),
      p->mc.obj.energy.energy_est
  );
}

static void
reset(rpl_dag_t *sag)
{
  PRINTF("RPL: Reset MRHOF\n");
}

static void
neighbor_link_callback(rpl_parent_t *p, int status, int numtx, delay_t delay)
{
  uint16_t recorded_etx = p->link_metric;
  uint16_t packet_etx = numtx * RPL_DAG_MC_ETX_DIVISOR;
  uint16_t new_etx;

  /* Do not penalize the ETX when collisions or transmission errors occur. */
  if(status == MAC_TX_OK || status == MAC_TX_NOACK) {
    if(status == MAC_TX_NOACK) {
      packet_etx = MAX_LINK_METRIC * RPL_DAG_MC_ETX_DIVISOR;
    #if CONTIKI_DELAY
      if(!rimeaddr_cmp(dest,&rimeaddr_null) && list_length(time_list) > 0)
        {
		     item = list_pop(time_list);
		     memb_free(&time_memb,item);
	      }
    #endif /* CONTIKI_DELAY */
    }
    
/* Update the EWMA of the ETX for the neighbor. */
    new_etx = ((uint32_t)recorded_etx * ETX_ALPHA +
               (uint32_t)packet_etx * (ETX_SCALE - ETX_ALPHA)) / ETX_SCALE;

    PRINTF("RPL: ETX changed from %u to %u (packet ETX = %u)\n",
        (unsigned)(recorded_etx / RPL_DAG_MC_ETX_DIVISOR),
        (unsigned)(new_etx  / RPL_DAG_MC_ETX_DIVISOR),
        (unsigned)(packet_etx / RPL_DAG_MC_ETX_DIVISOR));
    p->link_metric = new_etx;

#if CONTIKI_DELAY
    delay_t packet_delay, recorded_delay, new_delay;

		recorded_delay = p->delay_metric;
    packet_delay= delay;
		if(packet_delay > MAX_DELAY) packet_delay = MAX_DELAY;
			/* Update the EWMA of the delay for the neighbor. */
		new_delay = (recorded_delay * DLY_ALPHA + 
                 packet_delay * (DLY_SCALE - DLY_ALPHA)) / DLY_SCALE;

    PRINTF("RPL: delay changed from %u to %u (packet delay = %u)\n",
        (unsigned)(recorded_delay),
        (unsigned)(new_delay),
        (unsigned)(packet_delay));
     p->delay_metric = new_delay;   
#endif /*CONTIKI_DELAY*/
  }
}

static rpl_rank_t
calculate_rank(rpl_parent_t *p, rpl_rank_t base_rank)
{
  rpl_rank_t new_rank;
  rpl_rank_t rank_increase;

  if(p == NULL) {
    if(base_rank == 0) {
      return INFINITE_RANK;
    }
    rank_increase = DEFAULT_RANK_INCREMENT;
  } else {
    rank_increase = p->dag->instance->min_hoprankinc +
	    		((QUALITY_MAX - calculate_fuzzy_metric(p)) * p->dag->instance->min_hoprankinc)/QUALITY_RANK_DIVISOR;
    if(base_rank == 0) {
      base_rank = p->rank;
    }
  }

  if(INFINITE_RANK - base_rank < rank_increase) {
    /* Reached the maximum rank. */
    new_rank = INFINITE_RANK;
  } else {
   /* Calculate the rank based on the new rank information from DIO or
      stored otherwise. */
    new_rank = base_rank + rank_increase;
  }

  return new_rank;
}

static rpl_dag_t *
best_dag(rpl_dag_t *d1, rpl_dag_t *d2)
{
  if(d1->grounded != d2->grounded) {
    return d1->grounded ? d1 : d2;
  }

  if(d1->preference != d2->preference) {
    return d1->preference > d2->preference ? d1 : d2;
  }

  return d1->rank < d2->rank ? d1 : d2;
}

static rpl_parent_t *
best_parent(rpl_parent_t *p1, rpl_parent_t *p2)
{
  rpl_dag_t *dag;
  rpl_path_metric_t min_diff;
  rpl_rank_t r1;
  rpl_rank_t r2;

  dag = p1->dag; /* Both parents are in the same DAG. */

  min_diff = PARENT_SWITCH_THRESHOLD;

  r1 = calculate_rank(p1);
  r2 = calculate_rank(p2);

  /* Maintain stability of the preferred parent in case of similar ranks. */
  if(p1 == dag->preferred_parent || p2 == dag->preferred_parent) {
    if(r1 < r2 + min_diff &&
       r1 > r2 - min_diff) {
      PRINTF("RPL: MRHOF hysteresis: %u <= %u <= %u\n",
             r2 - min_diff,
             r1,
             r2 + min_diff);
      return dag->preferred_parent;
    }
  }

  return r1 < r2 ? p1 : p2;
}


static void
update_metric_container(rpl_instance_t *instance)
{
   rpl_dag_t *dag;

	  instance->mc.flags = RPL_DAG_MC_FLAG_P;
	  instance->mc.aggr = RPL_DAG_MC_AGGR_ADDITIVE;
	  instance->mc.prec = 0;
	  instance->mc.type = RPL_DAG_MC_FUZZY;
	  instance->mc.length = 8;
	  /*instance->mc.length = sizeof(instance->mc.obj);*/

	  dag = instance->current_dag;
	  if (!dag->joined) {
	    /* We should probably do something here */
	    return;
	  }

	  /* If this node is the root, then initialize metrics values accordingly */

	  rpl_path_metric_t energy, etx, hc, latency;
  /* If this node is the root, then initialize metrics values accordingly */
  if(dag->rank == ROOT_RANK(instance)) {
    energy = instance->mc.obj.energy.energy_est = calculate_energy_path_metric(dag->preferred_parent);
    instance->mc.obj.energy.flags = RPL_DAG_MC_ENERGY_TYPE_MAINS << RPL_DAG_MC_ENERGY_TYPE;
    etx = instance->mc.obj.etx = 0;
    hc = instance->mc.obj.hopcount = 0;
    latency = instance->mc.obj.latency = 0;
  } else {
    energy = instance->mc.obj.energy.energy_est = calculate_energy_path_metric(dag->preferred_parent);
    instance->mc.obj.energy.flags = RPL_DAG_MC_ENERGY_TYPE_BATTERY << RPL_DAG_MC_ENERGY_TYPE;
    etx = instance->mc.obj.etx = calculate_etx_path_metric(dag->preferred_parent);
    hc = instance->mc.obj.hopcount = calculate_hopcount_path_metric(dag->preferred_parent);
    latency = instance->mc.obj.latency = calculate_latency_path_metric(dag->preferred_parent);
  }
  printf("UwU energy %u etx %u hc %u latency %u\n", energy, etx, hc, latency);
}
