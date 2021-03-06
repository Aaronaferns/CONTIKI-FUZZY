
#include "net/rpl/rpl-private.h"
#include "net/nbr-table.h"
#include "net/delay.h"
#include "delay_func.h"
#include "lib/list.h"
#include "lib/memb.h"
#include "net/rpl/battery.h"
#include "net/rpl/FIS.h"
#define DLY_SCALE	100
#define DLY_ALPHA	90
extern unsigned long before_trans;
extern unsigned long after_ack;
MEMB(time_memb, struct time_queue, MAX_QUEUED_PACKETS);
LIST(time_list);


#define QUALITY_MAX 100
#define QUALITY_RANK_DIVISOR 10
#define DEFAULT_RANK_INCREMENT  RPL_MIN_HOPRANKINC
#define HOPCOUNT_MAX 50
#define MAX_PACKET_DELAY

#define DEBUG DEBUG_PRINT
#include "net/uip-debug.h"

static void reset(rpl_dag_t *);
static void neighbor_link_callback(rpl_parent_t *, int, int);
static rpl_parent_t *best_parent(rpl_parent_t *, rpl_parent_t *);
static rpl_dag_t *best_dag(rpl_dag_t *, rpl_dag_t *);
static rpl_rank_t calculate_rank(rpl_parent_t *, rpl_rank_t);
static void update_metric_container(rpl_instance_t *);

rpl_of_t rpl_fuzzyof = {
  reset,
  neighbor_link_callback,
  best_parent,
  best_dag,
  calculate_rank,
  update_metric_container,
  4
};

/* Constants for the ETX moving average */
#define ETX_SCALE   100
#define ETX_ALPHA   90

/* Reject parents that have a higher link metric than the following. */
#define MAX_LINK_METRIC			15

/* Reject parents that have a higher path cost than the following. */
#define MAX_PATH_COST			100



/*
 * The rank must differ more than 1/PARENT_SWITCH_THRESHOLD_DIV in order
 * to switch preferred parent.
 */
#define PARENT_SWITCH_THRESHOLD_DIV	2

typedef uint16_t rpl_path_metric_t;


static uint8_t calculate_energy_metric(rpl_parent_t *p)
{
  battery_charge_set();
  uint8_t ener = battery_charge_value;
  PRINTF("BAT: %u\n", ener);
  return ener;
}

static rpl_path_metric_t
calculate_hopcount_metric(rpl_parent_t *p)
{

	if(p == NULL ){
		return HOPCOUNT_MAX;
	}
	return p->mc.obj.hopcount + 1;
}


static rpl_path_metric_t
calculate_delay_metric(rpl_parent_t *p)
{
  if(p == NULL) {
    return MAX_DELAY;
  }
  return p->mc.obj.latency + (uint16_t)p->delay_metric;

}

static rpl_path_metric_t
calculate_path_metric(rpl_parent_t *p)
{
  if(p == NULL) {
    return MAX_PATH_COST * RPL_DAG_MC_ETX_DIVISOR;
  }
return p->mc.obj.etx + (uint16_t)p->link_metric;
}

static uint8_t
calculate_quality_metric(rpl_parent_t *p)
{
  return quality(qos((p->mc.obj.etx)/RPL_DAG_MC_ETX_DIVISOR, p->mc.obj.latency, p->mc.obj.hopcount), p->mc.obj.energy.energy_est);
}

static void
reset(rpl_dag_t *sag)
{
  PRINTF("RPL: Reset MRHOF\n");
}

static void
neighbor_link_callback(rpl_parent_t *p, int status, int numtx){

  struct time_queue *item;
  delay_t packet_delay,  new_delay;
  delay_t recorded_delay= p->delay_metric;

  uint16_t recorded_etx = p->link_metric;
  uint16_t packet_etx = numtx * RPL_DAG_MC_ETX_DIVISOR;
  uint16_t new_etx;

  /* Do not penalize the ETX when collisions or transmission errors occur. */
  if(status == MAC_TX_OK || status == MAC_TX_NOACK) {
    if(status == MAC_TX_OK) {
    if(list_length(time_list) > 0){
	  item = list_pop(time_list);
	  packet_delay = (before_trans - item->before_mac) + (after_ack - before_trans)/2;
	  memb_free(&time_memb,item); 
    } 
    }
    else if(status == MAC_TX_NOACK) {
      packet_etx = MAX_LINK_METRIC * RPL_DAG_MC_ETX_DIVISOR;
      packet_delay= 50;
      if(list_length(time_list) > 0){
		item = list_pop(time_list);
		memb_free(&time_memb,item);
	}
    }
    
    new_etx = ((uint32_t)recorded_etx * ETX_ALPHA +
               (uint32_t)packet_etx * (ETX_SCALE - ETX_ALPHA)) / ETX_SCALE;

    PRINTF("RPL: ETX changed from %u to %u (packet ETX = %u)\n",
        (unsigned)(recorded_etx / RPL_DAG_MC_ETX_DIVISOR),
        (unsigned)(new_etx  / RPL_DAG_MC_ETX_DIVISOR),
        (unsigned)(packet_etx / RPL_DAG_MC_ETX_DIVISOR));
    p->link_metric = new_etx;
    if(packet_delay > MAX_DELAY) packet_delay = MAX_DELAY;
    new_delay = (recorded_delay * DLY_ALPHA + packet_delay * (DLY_SCALE - DLY_ALPHA)) / DLY_SCALE;
    p->delay_metric = new_delay;

    PRINTF("RPL: delay changed from %u to %u (packet delay = %u)\n",
        (unsigned)(recorded_delay),
        (unsigned)(new_delay),
        (unsigned)(packet_delay));        

  }else{
    if(list_length(time_list) > 0){
		item = list_pop(time_list);
		memb_free(&time_memb,item);  
  }
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
	    		((QUALITY_MAX - calculate_quality_metric(p)) * p->dag->instance->min_hoprankinc)/QUALITY_RANK_DIVISOR;
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
  rpl_path_metric_t p1_metric;
  rpl_path_metric_t p2_metric;

  dag = p1->dag; /* Both parents are in the same DAG. */

  min_diff = PARENT_SWITCH_THRESHOLD_DIV;

  p1_metric = calculate_quality_metric(p1);
  p2_metric = calculate_quality_metric(p2);

  /* Maintain stability of the preferred parent in case of similar ranks. */
  if(p1 == dag->preferred_parent || p2 == dag->preferred_parent) {
    if(p1_metric < p2_metric + min_diff &&
       p1_metric > p2_metric - min_diff) {
      PRINTF("RPL: FUZZY OF hysteresis: %u <= %u <= %u\n",
             p2_metric - min_diff,
             p1_metric,
             p2_metric + min_diff);
      return dag->preferred_parent;
    }
  }

  return p1_metric < p2_metric ? p2 : p1;
}


static void
update_metric_container(rpl_instance_t *instance)
{
  rpl_path_metric_t path_metric, delay_metric, hopcount;
  uint8_t quality, energy;
uint8_t qos1;
  rpl_dag_t *dag;
  instance->mc.type = RPL_DAG_MC;
  instance->mc.flags = RPL_DAG_MC_FLAG_P;
  instance->mc.aggr = RPL_DAG_MC_AGGR_ADDITIVE;
  instance->mc.prec = 0;
instance->mc.length = 8;

  dag = instance->current_dag;

  if (!dag->joined) {
    PRINTF("RPL: Cannot update the metric container when not joined\n");
    return;
  }

  if(dag->rank == ROOT_RANK(instance)) {
    path_metric = 0;
    delay_metric = 0;
    hopcount = 0;
    energy= MAX_ENERGY;
    instance->mc.obj.energy.flags = RPL_DAG_MC_ENERGY_TYPE_MAINS << RPL_DAG_MC_ENERGY_TYPE;

  } else {
    path_metric = calculate_path_metric(dag->preferred_parent);
    delay_metric = calculate_delay_metric(dag->preferred_parent);
    hopcount = calculate_hopcount_metric(dag->preferred_parent);
    energy = calculate_energy_metric(dag->preferred_parent);
    instance->mc.obj.energy.flags = RPL_DAG_MC_ENERGY_TYPE_BATTERY << RPL_DAG_MC_ENERGY_TYPE;
  }


   rpl_path_metric_t et, l, h;
   uint8_t e;
  et = instance->mc.obj.etx = path_metric;
  l = instance->mc.obj.latency = delay_metric;
  h = instance->mc.obj.hopcount = hopcount;
  e = instance->mc.obj.energy.energy_est= energy;
  quality= calculate_quality_metric(dag->preferred_parent);
qos1 = qos((dag->preferred_parent->mc.obj.etx)/RPL_DAG_MC_ETX_DIVISOR, dag->preferred_parent->mc.obj.latency, dag->preferred_parent->mc.obj.hopcount);

  PRINTF(" ETX is %u , LATENCY is %u , HC is %u , ENERGY is %u, Quality is %u , qos is %u.\n",
	et / RPL_DAG_MC_ETX_DIVISOR, l, h,
  e, (unsigned)quality, (unsigned) qos1);

}

void neighbor_info_send_mac (rimeaddr_t *dest) {
    struct time_queue *item;
    if(!rimeaddr_cmp(dest,&rimeaddr_null)){
    	item = memb_alloc(&time_memb);
    	if(item != NULL){
    		item->dest = *dest;
    		item->before_mac = (clock_time()*1000)/CLOCK_SECOND;
    		list_add(time_list,item);
    	}
    	else {
    		printf("delay: could not allocate time_memb\n");
    	}
    }
}

void time_memb_init(){
    memb_init(&time_memb);
}

