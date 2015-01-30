/*
 * litmus/sched_acedf.c
 *
 * Implementation of the C-EDF scheduling algorithm.
 *
 * This implementation is based on G-EDF:
 * - CPUs are clustered around L2 or L3 caches.
 * - Clusters topology is automatically detected (this is arch dependent
 *   and is working only on x86 at the moment --- and only with modern
 *   cpus that exports cpuid4 information)
 * - The plugins _does not_ attempt to put tasks in the right cluster i.e.
 *   the programmer needs to be aware of the topology to place tasks
 *   in the desired cluster
 * - default clustering is around L2 cache (cache index = 2)
 *   supported clusters are: L1 (private cache: pedf), L2, L3, ALL (all
 *   online_cpus are placed in a single cluster).
 *
 *   For details on functions, take a look at sched_gsn_edf.c
 *
 * Currently, we do not support changes in the number of online cpus.
 * If the num_online_cpus() dynamically changes, the plugin is broken.
 *
 * This version uses the simple approach and serializes all scheduling
 * decisions by the use of a queue lock. This is probably not the
 * best way to do it, but it should suffice for now.
 */

#include <linux/spinlock.h>
#include <linux/percpu.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <linux/module.h>

#include <litmus/litmus.h>
#include <litmus/jobs.h>
#include <litmus/preempt.h>
#include <litmus/budget.h>
#include <litmus/sched_plugin.h>
#include <litmus/edf_common.h>
#include <litmus/sched_trace.h>
#include <litmus/trace.h>

#include <litmus/clustered.h>

#include <litmus/bheap.h>

#ifdef CONFIG_SCHED_CPU_AFFINITY
#include <litmus/affinity.h>
#endif

/* to configure the cluster size */
#include <litmus/litmus_proc.h>
#include <linux/uaccess.h>

#include <float.h>


/* Reference configuration variable. Determines which cache level is used to
 * group CPUs into clusters.  GLOBAL_CLUSTER, which is the default, means that
 * all CPUs form a single cluster (just like GSN-EDF).
 */
static enum cache_level cluster_config = L3_CLUSTER;
//static enum cache_level cluster_config = GLOBAL_CLUSTER;

struct clusterdomain;

/* cpu_entry_t - maintain the linked and scheduled state
 *
 * A cpu also contains a pointer to the acedf_domain_t cluster
 * that owns it (struct clusterdomain*)
 */
typedef struct  {
	int 			cpu;
	struct clusterdomain*	cluster;	/* owning cluster */
	struct task_struct*	linked;		/* only RT tasks */
	struct task_struct*	scheduled;	/* only RT tasks */
	atomic_t		will_schedule;	/* prevent unneeded IPIs */
	struct bheap_node*	hn;
} cpu_entry_t;

/* one cpu_entry_t per CPU */
DEFINE_PER_CPU(cpu_entry_t, acedf_cpu_entries);

#define set_will_schedule() \
	(atomic_set(&__get_cpu_var(acedf_cpu_entries).will_schedule, 1))
#define clear_will_schedule() \
	(atomic_set(&__get_cpu_var(acedf_cpu_entries).will_schedule, 0))
#define test_will_schedule(cpu) \
	(atomic_read(&per_cpu(acedf_cpu_entries, cpu).will_schedule))

/*
 * In C-EDF there is a acedf domain _per_ cluster
 * The number of clusters is dynamically determined accordingly to the
 * total cpu number and the cluster size
 */
typedef struct clusterdomain {
	int clusterID;
	int representative_CPU;

	/* rt_domain for this cluster */
	rt_domain_t	domain;
	/* cpus in this cluster */
	cpu_entry_t*	*cpus;
	/* map of this cluster cpus */
	cpumask_var_t	cpu_map;
	/* the cpus queue themselves according to priority in here */
	struct bheap_node *heap_node;
	struct bheap      cpu_heap;
	
	/* To migrate between clusters we'll need two locks */
	raw_spinlock_t secondary_lock;
	/* lock for this cluster */
#define cluster_lock domain.ready_lock
} acedf_domain_t;

static raw_spinlock_t global_lock;

#define NUMBER_OF_WITHELD_CPUS 0.5
#define MAX_CPUS_UTIL 0.90 

/* a acedf_domain per cluster; allocation is done at init/activation time */
acedf_domain_t *acedf;

#define remote_cluster(cpu)	((acedf_domain_t *) per_cpu(acedf_cpu_entries, cpu).cluster)
#define task_cpu_cluster(task)	remote_cluster(get_partition(task))

/* Uncomment WANT_ALL_SCHED_EVENTS if you want to see all scheduling
 * decisions in the TRACE() log; uncomment VERBOSE_INIT for verbose
 * information during the initialization of the plugin (e.g., topology) */
//#define WANT_ALL_SCHED_EVENTS

#define VERBOSE_INIT

#define MAX_CLUSTERS 4
#define MAX_TASKS 100
#define ACEDF_MAX_SERVICE_LEVEL 3

//TODO-AARON: The number here limits the number of real time threads. 
//Need to make this more general or fix
//Should make this linked list to be more general and flexible
static struct task_struct* all_tasks_acedf[MAX_TASKS]; 
//static int target_cluster_tasks[MAX_TASKS];


//TODO-AARON: This is part of the hacky fix above used to keep track of the number
//of tasks in the system and add them to the task_struct. 
static int currentNumberTasks_acedf[MAX_CLUSTERS];
static int numberTasks_acedf;

//Mark Beta

struct acedf_ratio_struct {
	double ratio;
	struct task_struct* task; 
};


static struct acedf_ratio_struct all_ratios[MAX_CLUSTERS][MAX_TASKS];

//TODO-AARON: This only works with 100 clusters. 
static double acedf_cluster_total_utilization[MAX_CLUSTERS]; 
static double acedf_cluster_total_QoS[MAX_CLUSTERS];
static int acedf_number_of_clusters;

 
static const lt_t nanosecondsBetweenReweights_acedf = 1000000000/4; //Number of times per second that the system is weighted at a minimum;
static const lt_t nanosecondsBetweenRepartitions_acedf = 1000000000*2 /*/1*/; //Number of times per second that the system is repatitionined at a maximum ;
static lt_t lastReweightTime_acedf[MAX_CLUSTERS]; //time used to keep track of the last time the cluster was reweighted
static lt_t initialStartTime_acedf;//Time the first task starts
static lt_t lastRepartitionTime_acedf; // Time of the last repartitioning of the system 
static lt_t initialStableWindowTime_acedf=((lt_t)10)*((lt_t)1000000000); //For the first 10 seconds, nothing happens. 
static int changeNow_acedf; //if 1, under utilized. if 0, nothing. -1 if over utilized

/* total number of cluster */
static int num_clusters;
/* we do not support cluster of different sizes */
static unsigned int cluster_size;



static int cpu_lower_prio(struct bheap_node *_a, struct bheap_node *_b)
{
	cpu_entry_t *a, *b;
	a = _a->value;
	b = _b->value;
	/* Note that a and b are inverted: we want the lowest-priority CPU at
	 * the top of the heap.
	 */
	return edf_higher_prio(b->linked, a->linked);
}

/* update_cpu_position - Move the cpu entry to the correct place to maintain
 *                       order in the cpu queue. Caller must hold acedf lock.
 */
static void update_cpu_position(cpu_entry_t *entry)
{
	acedf_domain_t *cluster = entry->cluster;

	if (likely(bheap_node_in_heap(entry->hn)))
		bheap_delete(cpu_lower_prio,
				&cluster->cpu_heap,
				entry->hn);

	bheap_insert(cpu_lower_prio, &cluster->cpu_heap, entry->hn);
}

/* caller must hold acedf lock */
static cpu_entry_t* lowest_prio_cpu(acedf_domain_t *cluster)
{
	struct bheap_node* hn;
	hn = bheap_peek(cpu_lower_prio, &cluster->cpu_heap);
	return hn->value;
}


/* link_task_to_cpu - Update the link of a CPU.
 *                    Handles the case where the to-be-linked task is already
 *                    scheduled on a different CPU.
 */
static noinline void link_task_to_cpu(struct task_struct* linked,
				      cpu_entry_t *entry)
{
	cpu_entry_t *sched;
	struct task_struct* tmp;
	int on_cpu;

	BUG_ON(linked && !is_realtime(linked));

	/* Currently linked task is set to be unlinked. */
	if (entry->linked) {
		entry->linked->rt_param.linked_on = NO_CPU;
	}

	/* Link new task to CPU. */
	if (linked) {
		/* handle task is already scheduled somewhere! */
		on_cpu = linked->rt_param.scheduled_on;
		if (on_cpu != NO_CPU) {
			sched = &per_cpu(acedf_cpu_entries, on_cpu);
			/* this should only happen if not linked already */
			BUG_ON(sched->linked == linked);

			/* If we are already scheduled on the CPU to which we
			 * wanted to link, we don't need to do the swap --
			 * we just link ourselves to the CPU and depend on
			 * the caller to get things right.
			 */
			if (entry != sched) {
#ifdef WANT_ALL_SCHED_EVENTS
				TRACE_TASK(linked,
					   "already scheduled on %d, updating link.\n",
					   sched->cpu);
#endif
				tmp = sched->linked;
				linked->rt_param.linked_on = sched->cpu;
				sched->linked = linked;
				update_cpu_position(sched);
				linked = tmp;
			}
		}
		if (linked) /* might be NULL due to swap */
			linked->rt_param.linked_on = entry->cpu;
	}
	entry->linked = linked;
#ifdef WANT_ALL_SCHED_EVENTS
	if (linked)
		TRACE_TASK(linked, "linked to %d.\n", entry->cpu);
	else
		TRACE("NULL linked to %d.\n", entry->cpu);
#endif
	update_cpu_position(entry);
}

/* unlink - Make sure a task is not linked any longer to an entry
 *          where it was linked before. Must hold acedf_lock.
 */
static noinline void unlink(struct task_struct* t)
{
	cpu_entry_t *entry;

	if (t->rt_param.linked_on != NO_CPU) {
		/* unlink */
		entry = &per_cpu(acedf_cpu_entries, t->rt_param.linked_on);
		t->rt_param.linked_on = NO_CPU;
		link_task_to_cpu(NULL, entry);
	} else if (is_queued(t)) {
		/* This is an interesting situation: t is scheduled,
		 * but was just recently unlinked.  It cannot be
		 * linked anywhere else (because then it would have
		 * been relinked to this CPU), thus it must be in some
		 * queue. We must remove it from the list in this
		 * case.
		 *
		 * in C-EDF case is should be somewhere in the queue for
		 * its domain, therefore and we can get the domain using
		 * task_cpu_cluster
		 */
		remove(&(task_cpu_cluster(t))->domain, t);
	}
}


/* preempt - force a CPU to reschedule
 */
static void preempt(cpu_entry_t *entry)
{
	preempt_if_preemptable(entry->scheduled, entry->cpu);
}

/* requeue - Put an unlinked task into gsn-edf domain.
 *           Caller must hold acedf_lock.
 */
static noinline void requeue(struct task_struct* task)
{ 
	acedf_domain_t *cluster;
 	cluster = task_cpu_cluster(task);

 	
	BUG_ON(!task);
	/* sanity check before insertion */
	BUG_ON(is_queued(task));

	if (is_early_releasing(task) || is_released(task, litmus_clock()))
	{
		__add_ready(&cluster->domain, task);
	}
	else {
		/* it has got to wait */
		add_release(&cluster->domain, task);
	}
}

#ifdef CONFIG_SCHED_CPU_AFFINITY
static cpu_entry_t* acedf_get_nearest_available_cpu(
				acedf_domain_t *cluster, cpu_entry_t *start)
{
	cpu_entry_t *affinity;

	get_nearest_available_cpu(affinity, start, acedf_cpu_entries,
#ifdef CONFIG_RELEASE_MASTER
		cluster->domain.release_master
#else
		NO_CPU
#endif
		);

	/* make sure CPU is in our cluster */
	if (affinity && cpu_isset(affinity->cpu, *cluster->cpu_map))
		return(affinity);
	else
		return(NULL);
}
#endif


/* check for any necessary preemptions */
static void check_for_preemptions(acedf_domain_t *cluster)
{
	struct task_struct *task;
	cpu_entry_t *last;

#ifdef CONFIG_PREFER_LOCAL_LINKING
	cpu_entry_t *local;

	/* Before linking to other CPUs, check first whether the local CPU is
	 * idle. */
	local = &__get_cpu_var(acedf_cpu_entries);
	task  = __peek_ready(&cluster->domain);

	if (task && !local->linked
		&& task-> rt_param.task_params.cpu == local->cluster->representative_CPU
#ifdef CONFIG_RELEASE_MASTER
	    && likely(local->cpu != cluster->domain.release_master)
#endif
		) {
		task = __take_ready(&cluster->domain);
#ifdef WANT_ALL_SCHED_EVENTS
		TRACE_TASK(task, "linking to local CPU %d to avoid IPI\n", local->cpu);
#endif		
		link_task_to_cpu(task, local);
		preempt(local);
	} 
	/*else if (task && !local->linked && task-> rt_param.task_params.cpu != local->cluster->representative_CPU ) { // TODO: Erase the else if, this is just for testing purposes 
		TRACE_TASK(task, "is on a different cluster, so, we're not linking to  %d\n", local->cpu);
	} */
	
#endif


	for(last = lowest_prio_cpu(cluster);
	    edf_preemption_needed(&cluster->domain, last->linked);
	    last = lowest_prio_cpu(cluster)) {
		/* preemption necessary */
		task = __take_ready(&cluster->domain);
#ifdef WANT_ALL_SCHED_EVENTS	
		TRACE("check_for_preemptions: attempting to link task %d to %d\n",
		      task->pid, last->cpu);
#endif		      
#ifdef CONFIG_SCHED_CPU_AFFINITY
		{
			cpu_entry_t *affinity =
					acedf_get_nearest_available_cpu(cluster,
						&per_cpu(acedf_cpu_entries, task_cpu(task)));
			if(affinity)
				last = affinity;
			else if(requeue_preempted_job(last->linked))
				requeue(last->linked);
		}
#else
		if (requeue_preempted_job(last->linked))
			requeue(last->linked);
#endif
		link_task_to_cpu(task, last);
		preempt(last);
	}
}

/* acedf_job_arrival: task is either resumed or released */
static noinline void acedf_job_arrival(struct task_struct* task)
{
	acedf_domain_t *cluster = task_cpu_cluster(task);
	BUG_ON(!task);

	requeue(task);
	check_for_preemptions(cluster);
}

static void acedf_release_jobs(rt_domain_t* rt, struct bheap* tasks)
{
	acedf_domain_t* cluster = container_of(rt, acedf_domain_t, domain);
	unsigned long flags;
	unsigned long flags_secondary;

	raw_spin_lock_irqsave(&cluster->cluster_lock, flags);
	raw_spin_lock_irqsave(&cluster->secondary_lock, flags_secondary);

	__merge_ready(&cluster->domain, tasks);
	check_for_preemptions(cluster);

	
	raw_spin_unlock_irqrestore(&cluster->secondary_lock, flags_secondary);
	raw_spin_unlock_irqrestore(&cluster->cluster_lock, flags);
}

//added
/* 	p and i values are the weights that should be given to the proportional and
 *	integrative components for calculating a new estimated execution time 
 * 	NOTE: Must be called before execution_cost is reset*/
static void calculate_estimated_execution_cost_acedf(struct task_struct *t, double p, double i){
	/* update the cumulative estimated execution difference */
	t->rt_param.cumulative_diff_est_actual_exec_cost+=
		t->rt_param.current_diff_est_actual_exec_cost;
		
	/* Update the difference between the estimated and actual execution time*/
	t->rt_param.current_diff_est_actual_exec_cost = 
		 get_exec_time(t) - get_estimated_exec_time(t);
 	//TRACE(",,,,time,%llu,task,%d,actualExecTime,%llu,estimated,%llu\n", litmus_clock(), t->pid,get_exec_time(t),get_estimated_exec_time(t));
 
	t->rt_param.job_params.estimated_exec_time = 
		(lt_t)	(p * t->rt_param.current_diff_est_actual_exec_cost + 
				 i * t->rt_param.cumulative_diff_est_actual_exec_cost);		
	return;
}


static noinline void repartition_tasks_acedf(int clusterID){
	int count;
	int i; 
	struct task_struct *temp;
	//TODO: Adjust all the service levels of all the jobs if a trigger threshold is met.
	// This is how tsk_rt(t)->ctrl_page->service_level;
	const double number_of_cpus_held_back = NUMBER_OF_WITHELD_CPUS; //Note: On my 12 core (24 virtual processors)
	//If this is under 5, then the system crashes. (Thus, the system runs on 19 virtual cores)
	struct task_struct** local_copy = all_tasks_acedf;

	int taskLevel[numberTasks_acedf];
	int taskCluster[numberTasks_acedf];
	double weightLevel[numberTasks_acedf];
	int outerIndex;
	int innerIndex;
	int lowerIndex;
	int upperIndex;
	double valueDensityLowerIndex;
	double valueDensityUpperIndex;
	double estWeightDiffUpper;
	double estWeightDiffLower;
	
	double QoSLowerIndex; 
	double QoSUpperIndex;   
	
	int aConvertValue;
	int bConvertValue;
	//TOOD: Change to be an array... 
	double localTotalUtilization[num_clusters];
	double minTotalUtil;
	int minTotalUilIndex;
	double maxUtilization = cluster_size-number_of_cpus_held_back;
	//This is the max_level 
	//If repartition_trigger is at most 1.0, then it will never have any impact. 
	const int max_level = ACEDF_MAX_SERVICE_LEVEL; //max level should be 3, but crashing. So let's try 2
	const int lowest_level = 0; 
	double calculationFactor;
	int number_of_tasks_on_cluster = 0;
	double min_qos;
	double max_qos;
	int fail_repartition = 0;
	int number_of_all_tasks =0;
	int min_cluster;
	
	
	int lowestIndexEachCluster[num_clusters];
	
	for(count = 0; count < num_clusters; count++){
		number_of_all_tasks+=currentNumberTasks_acedf[count];
		localTotalUtilization[i] = 0;
		lowestIndexEachCluster[count] = 0;
	}
	
	//merges arrays into lowest 
	for(count = 0; count < number_of_all_tasks; count++){
		min_cluster = -1;
		for( innerIndex = 0;innerIndex<num_clusters;innerIndex++){
			if ((min_cluster < 0) && (lowestIndexEachCluster[innerIndex]<currentNumberTasks_acedf[innerIndex])){
				min_cluster = innerIndex;
			} else if ((min_cluster>=0) && all_ratios[innerIndex][lowestIndexEachCluster[innerIndex]].ratio > all_ratios[min_cluster][lowestIndexEachCluster[min_cluster]].ratio){
				min_cluster = innerIndex;
			}
		}
		local_copy[count] = all_ratios[min_cluster][lowestIndexEachCluster[min_cluster]].task;
		
// 		aConvertValue = (int)(10000*all_ratios[min_cluster][lowestIndexEachCluster[min_cluster]].ratio);
// 		TRACE("For Count %d, choose index %d, and ratio %d\n", count, min_cluster, aConvertValue);
		
		lowestIndexEachCluster[min_cluster]++;
		
		

	}
	
	
	//Now need to maximize
	//Step 1 : Set all tasks to base level (level 0)
	//Step 2 : Increase tasks from top to lowest_priorty_cpu()
	//Step 3 : taskLevel


	//Step 1: set all tasks to their base service level (level 0) and assign to processor 
	// most remaining capacity with 
	//This allows for us to get an assessment of how much "extra" capacity we have

	for(outerIndex=0; outerIndex < number_of_all_tasks; outerIndex++) {
		//taskLevel corresponds to the calculated desired index for all elements
		//as sorted by the above. 
		//Since everything starts off at zero, taskLevel starts at zero.

		
	
		// If it is 0, then the estimated weight is the correct amount to to add to
		// localTotalUtiization 
		// FYI aConvertValue is a temp used for tracing
		weightLevel[outerIndex] = get_estimated_weight(local_copy[outerIndex]) / tsk_rt(local_copy[outerIndex])->task_params.service_levels[tsk_rt(local_copy[outerIndex])->ctrl_page->service_level].relative_work;	
		
		//Calculate the total utilization. 
		//localTotalUtilization+=weightLevel[outerIndex];
		//taskCluster
		
		minTotalUtil = localTotalUtilization[0];
		minTotalUilIndex = 0;
		
		//Find the cluster with the minimum amount of utilizliation
		for(innerIndex = 0; innerIndex< num_clusters;innerIndex++){
// 			aConvertValue = 10000*localTotalUtilization[minTotalUilIndex];
// 			bConvertValue = 10000*localTotalUtilization[innerIndex];
// 			TRACE("Old min10000,%d,testingNewMin,%d\n", aConvertValue, bConvertValue);
			if (localTotalUtilization[minTotalUilIndex] > localTotalUtilization[innerIndex]){
				minTotalUilIndex = innerIndex;
			}

		}
		
		//Assign this task to that cluster
		localTotalUtilization[minTotalUilIndex] += weightLevel[outerIndex];
		taskCluster[outerIndex] = minTotalUilIndex;
		//TRACE("Assigning %d to cluster %d\n", outerIndex, minTotalUilIndex);
	}
	
	//Step 1.5 validate that no cluster is overutilized
	fail_repartition = 0;
	for(outerIndex = 0; outerIndex <num_clusters;outerIndex++){
		if (localTotalUtilization[outerIndex] > maxUtilization){
			fail_repartition = 1;
		}
	}

// 	
// 	
// 	//Step 2 : Increase tasks from top to lowest_priorty_cpu()
// 	//Since all tasks are in sorted order at service level 0
// 	//we start from the first task and attempt to increase its service level 
// 	//to the maximum value without overloading the system. 
// 	//Even if we overload on one task, we still keep on increasing other tasks
// 	//because we might be able to increase their levels slightly 
	if(fail_repartition==0){
		TRACE(",,,,,time,%llu,REPARTITIONING\n", litmus_clock());
		for(outerIndex=0; outerIndex < number_of_all_tasks; outerIndex++) {

			//This assumes that all tasks have the same max service level
			//Increase servie level to the maximum possible value. 
			//We start at 1 because every task is already at zero. 	
			taskLevel[outerIndex] = 0;		
			for( innerIndex = 1; innerIndex <= max_level; innerIndex++) {
				//The value the weight would be if we increased the weight of the task
				calculationFactor = weightLevel[outerIndex] * tsk_rt(local_copy[outerIndex])->task_params.service_levels[innerIndex].relative_work;			
			
				//If we increased it, would the system be over utilized?
				if( (calculationFactor<MAX_CPUS_UTIL) &&  ((localTotalUtilization[taskCluster[outerIndex]]-weightLevel[outerIndex]+calculationFactor) < maxUtilization)) {

					//Increasing the weight of the task
					taskLevel[outerIndex] = innerIndex;
				}
			}
		
			//Change the total utilization by subtracting the old weight and adding the new weight
			//We don't actually change the weight here though
			localTotalUtilization[taskCluster[outerIndex]] = localTotalUtilization[taskCluster[outerIndex]]-weightLevel[outerIndex] + weightLevel[outerIndex]* tsk_rt(local_copy[outerIndex])->task_params.service_levels[taskLevel[outerIndex]].relative_work;	
		}
	
	// ***********************************************************************************
	// ***********************************************************************************

	
		//Go through and actually change the weight of each task now that all the work is done.
		for(outerIndex=0; outerIndex < number_of_all_tasks; outerIndex++) {
			local_copy[outerIndex]->rt_param.task_params.target_cpu = taskCluster[outerIndex];
			local_copy[outerIndex]->rt_param.task_params.target_service_level = taskLevel[outerIndex];
			//aConvertValue = (int)(10000*weightLevel[outerIndex]);
			//TRACE("REPART:Index,%d,MinWeight,%d,Cluster,%d,service_level,%d\n",outerIndex, aConvertValue,taskCluster[outerIndex], taskLevel[outerIndex]);
	 	}
	 	


	 	// Change the repartitioning time to now for all clusters. 
	 	for(outerIndex = 0; outerIndex < num_clusters; outerIndex++){
		 	lastReweightTime_acedf[outerIndex] = litmus_clock();			 	 
		 }
	}

	
}

// static noinline void adjust_all_service_levels_acedf(int triggerReweightNow, int clusterID){
// 	int acumulator = 0;
// 	int i = 0;
// 	for (i = 0; i< 100;i++) {
// 		acumulator+=all_tasks_acedf[triggerReweightNow]->rt_param.task_params.cpu;
// 		
// 	}	
// 	TRACE("acumulator %d\n", acumulator); 
// }

//Should only be called by job_completion. Thus, caller holds its' cluster lock
static noinline void adjust_all_service_levels_acedf(int triggerReweightNow, int clusterID){
	int count;
	int i; 
	struct task_struct *temp;
	const double number_of_cpus_held_back = NUMBER_OF_WITHELD_CPUS; 
	
	int taskLevel[numberTasks_acedf];
	double weightLevel[numberTasks_acedf];
	int outerIndex;
	int innerIndex;
	int lowerIndex;
	int upperIndex;
	double valueDensityLowerIndex;
	double valueDensityUpperIndex;
	double estWeightDiffUpper;
	double estWeightDiffLower;
	
	double QoSLowerIndex; 
	double QoSUpperIndex; 
	
	int aConvertValue;
	double localTotalUtilization = 0;
	double maxUtilization = cluster_size-number_of_cpus_held_back;
	//This is the max_level 
	//If repartition_trigger is at most 1.0, then it will never have any impact. 
	//const double repartition_trigger = 1.25; // TODO: Change this to a real value
	const double repartition_trigger = 0.90; // TODO: Change this to a real value
	const int max_level = ACEDF_MAX_SERVICE_LEVEL; //max level should be 3, but crashing. So let's try 2
	const int lowest_level = 0; //lowest service level level should be 3, but crashing. So let's try 2
	double calculationFactor;
	int number_of_tasks_on_cluster = 0;
	double min_qos;
	double max_qos;
	
	int representative_CPU = acedf[clusterID].representative_CPU;
	
	//Improve the conditions here so it's time based not task release based. 
	//changeNow_acedf is used to change only once at the beginning
	//changeNow_acedf=1;
	if( triggerReweightNow!=0 ) { 
		

		//Step 1: set all tasks to their base service level (level 0)
		//This allows for us to get an assessment of how much "extra" capacity we have

		for(count = 0;count < currentNumberTasks_acedf[clusterID]; count++) {

			weightLevel[count] = get_estimated_weight(all_ratios[clusterID][count].task) / tsk_rt(all_ratios[clusterID][count].task)->task_params.service_levels[tsk_rt(all_ratios[clusterID][count].task)->ctrl_page->service_level].relative_work;
			localTotalUtilization+=weightLevel[count];
		}
		

		//sort local_copy based on value density.. Assumes linear relationship
		// The rather complicated formulas in here come from Aaron Block's dissertation
		// specifically formula 6.11 on page 258
		// you can get Aaron's dissertation here: http://cs.unc.edu/~block/aarondiss.pdf

		//Local_copy is now sorted
		
		//Now need to maximize
		//Step 1 : Set all tasks to base level (level 0)
		//Step 2 : Increase tasks from top to lowest_priorty_cpu()
		//Step 3 : taskLevel
		
		//Step 2 : Increase tasks from top to lowest_priorty_cpu()
		//Since all tasks are in sorted order at service level 0
		//we start from the first task and attempt to increase its service level 
		//to the maximum value without overloading the system. 
		//Even if we overload on one task, we still keep on increasing other tasks
		//because we might be able to increase their levels slightly 
		
		for(outerIndex=0; outerIndex < currentNumberTasks_acedf[clusterID]; outerIndex++) {

			//This assumes that all tasks have the same max service level
			//Increase servie level to the maximum possible value. 
			//We start at 1 because every task is already at zero. 		
			taskLevel[outerIndex] = 0;	
			for( innerIndex = 1; innerIndex <= max_level; innerIndex++) {
				//The value the weight would be if we increased the weight of the task
				calculationFactor = weightLevel[outerIndex] * tsk_rt(all_ratios[clusterID][outerIndex].task)->task_params.service_levels[innerIndex].relative_work;
			
				if( (calculationFactor<MAX_CPUS_UTIL) && ((localTotalUtilization-weightLevel[outerIndex]+calculationFactor) < maxUtilization)) {
					//Increasing the weight of the task
					taskLevel[outerIndex] = innerIndex;
				}
			}
			
			//Change the total utilization by subtracting the old weight and adding the new weight
			//We don't actually change the weight here though
			localTotalUtilization = localTotalUtilization-weightLevel[outerIndex] + weightLevel[outerIndex]* tsk_rt(all_ratios[clusterID][outerIndex].task)->task_params.service_levels[taskLevel[outerIndex]].relative_work;
		}

		
		// Go through and actually change the weight of each task now that all the work is done.
		acedf_cluster_total_QoS[clusterID] = 0.0;


		for(outerIndex=0; outerIndex < currentNumberTasks_acedf[clusterID]; outerIndex++) {
			//Change the service level to be the target_service_level
			all_ratios[clusterID][outerIndex].task->rt_param.task_params.target_service_level = taskLevel[outerIndex];
			acedf_cluster_total_QoS[clusterID] += tsk_rt(all_ratios[clusterID][outerIndex].task)->task_params.service_levels[taskLevel[outerIndex]].quality_of_service;
		}
		
// 		
// 		//////////////////////////////////////////
// 		//Now, determine if we need to repartition. 
		min_qos = acedf_cluster_total_QoS[clusterID];
		max_qos = acedf_cluster_total_QoS[clusterID];
		
		for(outerIndex = 0; outerIndex < acedf_number_of_clusters; outerIndex++) {
			if (min_qos > acedf_cluster_total_QoS[outerIndex]){
				min_qos = acedf_cluster_total_QoS[outerIndex];
			}
			
			if (max_qos < acedf_cluster_total_QoS[outerIndex]){
				max_qos = acedf_cluster_total_QoS[outerIndex];
			}
		} 
		

		if ( /*(min_qos > 0) && (max_qos > 0)  && ((max_qos/min_qos) > repartition_trigger)  && */( (lastRepartitionTime_acedf + nanosecondsBetweenRepartitions_acedf) < litmus_clock())) {
			
	
			//We check this value twice to make sure that we aren't trying to repartition twice back to back
			//The first check (pre-locks) gets ride of most of the request
			//This check provents race conditions
	
			/* release your own lock */
			raw_spin_unlock(&acedf[clusterID].secondary_lock);
	
			/* aquire all the locks IN ORDER */
			for(i =0; i< acedf_number_of_clusters; i++ ){
				raw_spin_lock(&acedf[i].secondary_lock);
			}
	


		
			if ((lastRepartitionTime_acedf + nanosecondsBetweenRepartitions_acedf) < litmus_clock()){
				lastRepartitionTime_acedf = litmus_clock();
				repartition_tasks_acedf(clusterID);
			} 
			
			/* release all locks except the one for this cluster */
			for(i =0; i< acedf_number_of_clusters; i++ ){
				if (i !=clusterID ){
					raw_spin_unlock(&acedf[i].secondary_lock);
				}
			}
		}	

	}	
}

static void remove_task_from_cluster(struct task_struct* t, int cluster_id){
	int count = t->rt_param.task_params.sorted_list_index;
	int aconv;
	int bconv;
	int j;
	
	
	j = count;
	while (j < currentNumberTasks_acedf[cluster_id]){
		//aconv= (int)(10000*all_ratios[cluster_id][j].ratio);
		if( (j+1) < currentNumberTasks_acedf[cluster_id]){		
			//bconv= (int)(10000*all_ratios[cluster_id][j+1].ratio);
			//TRACE("For Count %d, Replace %d, with %d\n",count, aconv, bconv);
			
			all_ratios[cluster_id][j] = all_ratios[cluster_id][j+1];
			if(all_ratios[cluster_id][j].task != 0){
				all_ratios[cluster_id][j].task->rt_param.task_params.sorted_list_index = j;
			}
		}
		j++;
	}
//	bconv= (int)(10000*all_ratios[cluster_id][currentNumberTasks_acedf[cluster_id]-1].ratio);
//	TRACE("Killing ratio %d\n", bconv);
	
	all_ratios[cluster_id][currentNumberTasks_acedf[cluster_id]-1].ratio = -1;
	all_ratios[cluster_id][currentNumberTasks_acedf[cluster_id]-1].task = 0;
	t->rt_param.task_params.sorted_list_index =-1;


	
	currentNumberTasks_acedf[cluster_id]--;
	acedf_cluster_total_utilization[cluster_id] -= get_estimated_weight(t);
}

static void add_task_to_cluster(struct task_struct* t, int cluster_id){
	int i = 0;
	const int max_level = ACEDF_MAX_SERVICE_LEVEL;
	const int lowest_level = 0; 
	int j;
	double QoSDifference;
	double weight_range;
	double current_min_weight;
	double ratio; 
	struct acedf_ratio_struct ratio_holder_a;
	struct acedf_ratio_struct ratio_holder_b;
	
	
	
	
	currentNumberTasks_acedf[cluster_id]++;
	acedf_cluster_total_utilization[cluster_id] += get_estimated_weight(t);
	
	
	QoSDifference = tsk_rt(t)->task_params.service_levels[max_level].quality_of_service - tsk_rt(t)->task_params.service_levels[lowest_level].quality_of_service;
	current_min_weight = get_estimated_weight(t)/tsk_rt(t)->task_params.service_levels[tsk_rt(t)->ctrl_page->service_level].relative_work;
	weight_range =  current_min_weight * (tsk_rt(t)->task_params.service_levels[max_level].relative_work - 1 );
	ratio = QoSDifference/weight_range;
	
	i=0;
	while ( (i < currentNumberTasks_acedf[cluster_id])  && (all_ratios[cluster_id][i].ratio >= ratio ) ) {
		i++;
	}
	if (all_ratios[cluster_id][i].ratio<0){
		all_ratios[cluster_id][i].ratio =  ratio;
		all_ratios[cluster_id][i].task = t;
		t->rt_param.task_params.sorted_list_index = i;
	} else {
		j = i;
		ratio_holder_a.ratio = ratio;
		ratio_holder_a.task = t;
		while ((j < currentNumberTasks_acedf[cluster_id]) && (all_ratios[cluster_id][j].ratio>=0)) {
			ratio_holder_b = all_ratios[cluster_id][j];
			all_ratios[cluster_id][j] = ratio_holder_a;
			all_ratios[cluster_id][j].task->rt_param.task_params.sorted_list_index = j;
			ratio_holder_a = ratio_holder_b;
			j++;
		}
		if ((j < currentNumberTasks_acedf[cluster_id]) && (all_ratios[cluster_id][j].ratio<0)) {
			all_ratios[cluster_id][j] = ratio_holder_a;
			all_ratios[cluster_id][j].task->rt_param.task_params.sorted_list_index = j;
		}
	}

}

static void acedf_migrate_to(struct task_struct* task_to_migrate, int target_cluster_id)
{

	
	unsigned long flags;
	acedf_domain_t *oldcluster = task_cpu_cluster(task_to_migrate);
	int oldClusterID  = oldcluster-> clusterID;
	
	remove_task_from_cluster(task_to_migrate, oldClusterID);
	
	//task_to_migrate->rt_param.task_params.cpu = acedf[target_cluster_id].representative_CPU; 
	
	//add_task_to_cluster(task_to_migrate, target_cluster_id);
	add_task_to_cluster(task_to_migrate, oldClusterID);
}


static noinline int job_completion(struct task_struct *t, int forced)
{

	int i,j;
	int cluster_id;

	acedf_domain_t *oldcluster;
	acedf_domain_t *newcluster;


	const double bufferCPUs = NUMBER_OF_WITHELD_CPUS; 
	//If this is under 5, then the system crashes. (Thus, the system runs on 19 virtual cores)
	double old_est_weight;
	double difference_in_weight;
	int triggerReweightNow = 0;
	int largeWeight = 0;
	int totalUtil = 0;
	double maxUtilization;
	unsigned int job_no;
	const int max_level = ACEDF_MAX_SERVICE_LEVEL;
	const int lowest_level = 0; 
	const double PERCENT_CHANGE_TRIGGER = 0.125; // If the task's weight changes by this percentage
	// between job releases, then trigger an immediate reweighting

	int index_in_sorted = t->rt_param.task_params.sorted_list_index;
	double new_ratio;
	struct acedf_ratio_struct ratio_holder_a;
	struct acedf_ratio_struct ratio_holder_b;
	double QoSDifference;
	double weight_range;
	double current_min_weight;
	double old_min_weight; 
	int ratio_measurment;
	int large_value_for_printing;

	BUG_ON(!t);

	sched_trace_task_completion(t, forced);

	cluster_id = task_cpu_cluster(t)->clusterID;
	
	/* set flags */
	tsk_rt(t)->completed = 0;
	
	//added
	old_est_weight = get_estimated_weight(t);
 
	//TODO: replace the (0.10206228,1) in next line with user-set p and i values
	/* The values 0.102 and 0.30345 are the a and b values that are calculated from
	 * Aaron Block's dissertation referenced on pages 293 (the experimental values for
	 * a and c) and the relationship of a,b,c is given on page 253 just below (6.2)
	 */
	calculate_estimated_execution_cost_acedf(t,0.102,0.30345); 
	
	t->rt_param.job_params.estimated_weight = 
		((double) get_estimated_exec_time(t))/get_rt_relative_deadline(t);
	
	/* Update the sorted tasks */
		/* establish the new Ratio  */
	old_min_weight = old_est_weight/tsk_rt(t)->task_params.service_levels[tsk_rt(t)->ctrl_page->service_level].relative_work;
	
	QoSDifference = tsk_rt(t)->task_params.service_levels[max_level].quality_of_service - tsk_rt(t)->task_params.service_levels[lowest_level].quality_of_service;
	current_min_weight = get_estimated_weight(t)/tsk_rt(t)->task_params.service_levels[tsk_rt(t)->ctrl_page->service_level].relative_work;
	weight_range =  current_min_weight * (tsk_rt(t)->task_params.service_levels[max_level].relative_work - 1 );
	new_ratio = QoSDifference/weight_range;

	/* Put into the sorted list array */ 
	if (index_in_sorted<0){
		i=0;
		while ( (i < currentNumberTasks_acedf[cluster_id])  && (all_ratios[cluster_id][i].ratio >= new_ratio ) ) {
			i++;
		}
		if (all_ratios[cluster_id][i].ratio<0){
			all_ratios[cluster_id][i].ratio =  new_ratio;
			all_ratios[cluster_id][i].task = t;
			t->rt_param.task_params.sorted_list_index = i;
		} else {
			j = i;
			ratio_holder_a.ratio = new_ratio;
			ratio_holder_a.task = t;
			while ((j < currentNumberTasks_acedf[cluster_id]) && (all_ratios[cluster_id][j].ratio>=0)) {
				ratio_holder_b = all_ratios[cluster_id][j];
				all_ratios[cluster_id][j] = ratio_holder_a;
				all_ratios[cluster_id][j].task->rt_param.task_params.sorted_list_index = j;
				ratio_holder_a = ratio_holder_b;
				j++;
			}
			if ((j < currentNumberTasks_acedf[cluster_id]) && (all_ratios[cluster_id][j].ratio<0)) {
				all_ratios[cluster_id][j] = ratio_holder_a;
				all_ratios[cluster_id][j].task->rt_param.task_params.sorted_list_index = j;
			}
		}
	} else if (index_in_sorted< currentNumberTasks_acedf[cluster_id]){
		if (all_ratios[cluster_id][index_in_sorted].task == 0){
			TRACE("Something has gone wrong, Task %d has an index to a zero entry\n",t->pid );
		} else if (all_ratios[cluster_id][index_in_sorted].task->pid != t->pid)  {
		 	TRACE("Something has gone wrong, Task %d is pointing to %d but the entry thinks it should be, index is, %d\n",t->pid,all_ratios[cluster_id][index_in_sorted].task->pid,index_in_sorted);
		}else {			
			j = index_in_sorted;
			if(all_ratios[cluster_id][index_in_sorted].ratio > new_ratio)  {		
				while ( ((j+1) < currentNumberTasks_acedf[cluster_id]) && (all_ratios[cluster_id][j+1].ratio > new_ratio)) {
					all_ratios[cluster_id][j] = all_ratios[cluster_id][j+1];
					all_ratios[cluster_id][j].task->rt_param.task_params.sorted_list_index = j;
					j++;
				}
				if (j < currentNumberTasks_acedf[cluster_id]) {
					all_ratios[cluster_id][j].ratio = new_ratio;
					all_ratios[cluster_id][j].task = t;
					all_ratios[cluster_id][j].task->rt_param.task_params.sorted_list_index = j;
				} else {
					TRACE("SOMETHING HAS GONE HORRIBLY WRONG, J can't be bigger than currentNumberTasks_acedf[cluster_id]\n");
				}
			} else {
				while ( ((j-1) >= 0) && (all_ratios[cluster_id][j-1].ratio < new_ratio)) {
					all_ratios[cluster_id][j] = all_ratios[cluster_id][j-1];
					all_ratios[cluster_id][j].task->rt_param.task_params.sorted_list_index = j;
					j--;
				}
				if (j >=0 ) { 
					all_ratios[cluster_id][j].ratio = new_ratio;
					all_ratios[cluster_id][j].task = t;
					all_ratios[cluster_id][j].task->rt_param.task_params.sorted_list_index = j;
				} else {
					TRACE("SOMETHING HAS GONE HORRIBLY WRONG, J can't be smaller than 0\n");
				}
			}
		}
	} else {
		TRACE("SOMETHING HAS GONE WRONG %d can't be larger than or equal to %d", index_in_sorted, currentNumberTasks_acedf[cluster_id]);
	}	
// 	for(j=0; j< currentNumberTasks_acedf[cluster_id];j++){
// 		large_value_for_printing = (int)(10000*all_ratios[cluster_id][j].ratio);
// 		TRACE("The cluster ID:%d, the value is %d\n", j, large_value_for_printing);
// 	}

	
	
	
	/* The change in the estimated_weight of this job must be added to the total 
	 * utilization
	 */ 
	difference_in_weight = get_estimated_weight(t) - old_est_weight;
	acedf_cluster_total_utilization[cluster_id] += difference_in_weight;

	largeWeight = 10000*get_estimated_weight(t);
	totalUtil = 10000*acedf_cluster_total_utilization[cluster_id];
	job_no =  t->rt_param.job_params.job_no;
	TRACE(",,,,,time,%llu,taskID,%d,cluster,%d,estWtTimes10000,%d,serviceLevel,%u,taskQoSTimes1000,%d,TOTALQoSTimes1000,%d,jobNumber,%u,totalUtil10000,%d\n", 
	litmus_clock(), 
	t->pid,
	task_cpu_cluster(t)->clusterID,
	largeWeight,
	tsk_rt(t)->ctrl_page->service_level,
	(int)(tsk_rt(t)->task_params.service_levels[tsk_rt(t)->ctrl_page->service_level].quality_of_service*1000),
	(int)(acedf_cluster_total_QoS[task_cpu_cluster(t)->clusterID]*1000), 
	job_no,
	totalUtil );


		
	//When the jobs are released, then start the initiali timers;	
	if(initialStartTime_acedf ==0){
		initialStartTime_acedf = litmus_clock();
	}
	if(lastReweightTime_acedf[task_cpu_cluster(t)->clusterID] == 0){
		lastReweightTime_acedf[task_cpu_cluster(t)->clusterID] = litmus_clock();
	}
	
	
	triggerReweightNow = 0;
	
	

	maxUtilization = cluster_size - bufferCPUs;

	/* Trigger because the system is too utilized */
	if( acedf_cluster_total_utilization[cluster_id]  > maxUtilization) {
		triggerReweightNow = 1;
		largeWeight = 10000*acedf_cluster_total_utilization[cluster_id];
		//
		//TRACE(",,,,,time,%llu,REWEIGHTOVER10000,%d,\n",litmus_clock(), largeWeight);
		//
	}
	
	//Mark Alpha
	if (get_estimated_weight(t) > MAX_CPUS_UTIL) {
		//
		//TRACE(",,,,,time,%llu,SINGLE_OVER,%d,\n",litmus_clock(), get_estimated_weight(t));
		//
		triggerReweightNow = 1;
	}
	
	//trigger because a task changed its weight by too much
	if( (get_estimated_weight(t) > old_est_weight*(1+PERCENT_CHANGE_TRIGGER)) ||
		(get_estimated_weight(t) < old_est_weight*(1-PERCENT_CHANGE_TRIGGER)))
	{
		triggerReweightNow = 1;
		//
		//TRACE(",,,,,time,%llu,TRIGGER PER TASK\n",litmus_clock());
		//
	}
	
	//trigger because enough time has passed
	if ( (lastReweightTime_acedf[task_cpu_cluster(t)->clusterID] + nanosecondsBetweenReweights_acedf) < litmus_clock()){
		triggerReweightNow = 1;
		lastReweightTime_acedf[task_cpu_cluster(t)->clusterID] = litmus_clock();
		//
		//TRACE(",,,,,time,%llu,TRIGGER TIME\n",litmus_clock());
		//
		
	}

	// if we aren't past the initial window, then don't reweight
	if((initialStartTime_acedf+initialStableWindowTime_acedf) > litmus_clock()){
		triggerReweightNow = 0;
	}
	
	adjust_all_service_levels_acedf(triggerReweightNow,cluster_id);
	
	if (tsk_rt(t)->ctrl_page->service_level != t->rt_param.task_params.target_service_level) {
		tsk_rt(t)->ctrl_page->service_level = t->rt_param.task_params.target_service_level;
	}


	
	/* prepare for next period */
	prepare_for_next_period(t);
	
	if (is_early_releasing(t) || is_released(t, litmus_clock()))
		sched_trace_task_release(t);
	/* unlink */
	unlink(t);
	/* requeue
	 * But don't requeue a blocking task. */
	if (is_running(t)) { 
		
		
		if(t->rt_param.task_params.target_cpu != t->rt_param.task_params.cpu){
			oldcluster = remote_cluster(t->rt_param.task_params.cpu); 
			newcluster = remote_cluster(t->rt_param.task_params.target_cpu);
			//Release the secondar lock
			raw_spin_unlock(&oldcluster->secondary_lock);
			
			//reaquire all the locks
			for(i =0; i< acedf_number_of_clusters; i++ ){
				raw_spin_lock(&acedf[i].secondary_lock);
			}

			
			acedf_migrate_to(t, newcluster->clusterID);
			 
			//release all the locks
			for(i =0; i< acedf_number_of_clusters; i++ ){
				if (i !=oldcluster->clusterID && i !=newcluster->clusterID){
					raw_spin_unlock(&acedf[i].secondary_lock);
				}
			}
						
			acedf_job_arrival(t);
			// We return a cluster ID because later on, we'll need to release the
			// new cluster's lock ID as well. 
			return newcluster->clusterID;
		} else {
			acedf_job_arrival(t);
		}
	}
	return -1;
}







/* Getting schedule() right is a bit tricky. schedule() may not make any
 * assumptions on the state of the current task since it may be called for a
 * number of reasons. The reasons include a scheduler_tick() determined that it
 * was necessary, because sys_exit_np() was called, because some Linux
 * subsystem determined so, or even (in the worst case) because there is a bug
 * hidden somewhere. Thus, we must take extreme care to determine what the
 * current state is.
 *
 * The CPU could currently be scheduling a task (or not), be linked (or not).
 *
 * The following assertions for the scheduled task could hold:
 *
 *      - !is_running(scheduled)        // the job blocks
 *	- scheduled->timeslice == 0	// the job completed (forcefully)
 *	- is_completed()		// the job completed (by syscall)
 * 	- linked != scheduled		// we need to reschedule (for any reason)
 * 	- is_np(scheduled)		// rescheduling must be delayed,
 *					   sys_exit_np must be requested
 *
 * Any of these can occur together.
 */
static struct task_struct* acedf_schedule(struct task_struct * prev)
{
	int schedule_normal_path = -1;
	cpu_entry_t* entry = &__get_cpu_var(acedf_cpu_entries);
	acedf_domain_t *cluster = entry->cluster;
	int out_of_time, sleep, preempt, np, exists, blocks;
	struct task_struct* next = NULL;

#ifdef CONFIG_RELEASE_MASTER
	/* Bail out early if we are the release master.
	 * The release master never schedules any real-time tasks.
	 */
	if (unlikely(cluster->domain.release_master == entry->cpu)) {
		sched_state_task_picked();
		return NULL;
	}
#endif

	raw_spin_lock(&cluster->cluster_lock);
	raw_spin_lock(&cluster->secondary_lock);
	
	clear_will_schedule();

	/* sanity checking */
	BUG_ON(entry->scheduled && entry->scheduled != prev);
	BUG_ON(entry->scheduled && !is_realtime(prev));
	BUG_ON(is_realtime(prev) && !entry->scheduled);

	/* (0) Determine state */
	exists      = entry->scheduled != NULL;
	blocks      = exists && !is_running(entry->scheduled);
	out_of_time = exists &&
				  budget_enforced(entry->scheduled) &&
				  budget_exhausted(entry->scheduled);
	np 	    = exists && is_np(entry->scheduled);
	sleep	    = exists && is_completed(entry->scheduled);
	preempt     = entry->scheduled != entry->linked;

#ifdef WANT_ALL_SCHED_EVENTS
	TRACE_TASK(prev, "invoked acedf_schedule.\n");

	if (exists){
		TRACE_TASK(prev,
			   "blocks:%d out_of_time:%d np:%d sleep:%d preempt:%d "
			   "state:%d sig:%d\n",
			   blocks, out_of_time, np, sleep, preempt,
			   prev->state, signal_pending(prev)); 
	}
	if (entry->linked && preempt){
		TRACE_TASK(prev, "will be preempted by %s/%d\n",
			   entry->linked->comm, entry->linked->pid);
	}
#endif

	/* If a task blocks we have no choice but to reschedule.
	 */
	if (blocks)
		unlink(entry->scheduled);

	/* Request a sys_exit_np() call if we would like to preempt but cannot.
	 * We need to make sure to update the link structure anyway in case
	 * that we are still linked. Multiple calls to request_exit_np() don't
	 * hurt.
	 */
	if (np && (out_of_time || preempt || sleep)) {
		unlink(entry->scheduled);
		request_exit_np(entry->scheduled);
	}

	/* Any task that is preemptable and either exhausts its execution
	 * budget or wants to sleep completes. We may have to reschedule after
	 * this. Don't do a job completion if we block (can't have timers running
	 * for blocked jobs).
	 */
	if (!np && (out_of_time || sleep) && !blocks)
		schedule_normal_path = job_completion(entry->scheduled, !sleep);

	if (schedule_normal_path == -1 ){
		/* Link pending task if we became unlinked. */
		if (!entry->linked)
			link_task_to_cpu(__take_ready(&cluster->domain), entry);

		/* The final scheduling decision. Do we need to switch for some reason?
		 * If linked is different from scheduled, then select linked as next.
		 */
		if ((!np || blocks) &&
			entry->linked != entry->scheduled) {
			/* Schedule a linked job? */
			if (entry->linked) {
				entry->linked->rt_param.scheduled_on = entry->cpu;
				next = entry->linked;
			}
			if (entry->scheduled) {
				/* not gonna be scheduled soon */
				entry->scheduled->rt_param.scheduled_on = NO_CPU;
				//TRACE_TASK(entry->scheduled, "scheduled_on = NO_CPU\n");
			}
		} else
			/* Only override Linux scheduler if we have a real-time task
			 * scheduled that needs to continue.
			 */
			if (exists)
				next = prev;

		sched_state_task_picked();
		//TRACE("Releasing %d lock:Schedule \n",cluster->clusterID );
		raw_spin_unlock(&cluster->secondary_lock);
		raw_spin_unlock(&cluster->cluster_lock);
	} else { // We run this path if we migrate a task between processors
		/* Link pending task if we became unlinked. */
		if (!entry->linked)
			link_task_to_cpu(__take_ready(&cluster->domain), entry);

		/* The final scheduling decision. Do we need to switch for some reason?
		 * If linked is different from scheduled, then select linked as next.
		 */
		if ((!np || blocks) &&
			entry->linked != entry->scheduled) {
			/* Schedule a linked job? */
			if (entry->linked) {
				entry->linked->rt_param.scheduled_on = entry->cpu;
				next = entry->linked;
			}
			if (entry->scheduled) {
				/* not gonna be scheduled soon */
				entry->scheduled->rt_param.scheduled_on = NO_CPU;
				//TRACE_TASK(entry->scheduled, "scheduled_on = NO_CPU\n");
			}
		} else
			/* Only override Linux scheduler if we have a real-time task
			 * scheduled that needs to continue.
			 */
			if (exists)
				next = prev;

		sched_state_task_picked();  
		//TRACE("Releasing %d lock:Schedule \n",cluster->clusterID );
		//TODO change the cluster we are scheduling on 
		raw_spin_unlock(&acedf[schedule_normal_path].secondary_lock);
		raw_spin_unlock(&cluster->secondary_lock);
		raw_spin_unlock(&cluster->cluster_lock);
	}

#ifdef WANT_ALL_SCHED_EVENTS
	TRACE("acedf_lock released, next=0x%p\n", next);

	if (next)
		TRACE_TASK(next, "scheduled at %llu\n", litmus_clock());
	else if (exists && !next)
		TRACE("becomes idle at %llu.\n", litmus_clock());
#endif


	return next;
}


/* _finish_switch - we just finished the switch away from prev
 */
static void acedf_finish_switch(struct task_struct *prev)
{

	cpu_entry_t* 	entry = &__get_cpu_var(acedf_cpu_entries);

	entry->scheduled = is_realtime(current) ? current : NULL;
#ifdef WANT_ALL_SCHED_EVENTS
	TRACE_TASK(prev, "switched away from\n");
#endif
}


/*	Prepare a task for running in RT mode
 */
static void acedf_task_new(struct task_struct * t, int on_rq, int is_scheduled)
{
	unsigned long 		flags;
	unsigned long 		flags_secondary;
	cpu_entry_t* 		entry;
	acedf_domain_t*		cluster;
	int 				cluster_id;


	TRACE("acedf edf: task new %d\n", t->pid);

	/* the cluster doesn't change even if t is scheduled */
	cluster = task_cpu_cluster(t);
	cluster_id  = cluster->clusterID;
	
	TRACE("ACEDF: Task , %d, is initially assigned to %d\n", t->pid, cluster->clusterID);

	raw_spin_lock_irqsave(&cluster->cluster_lock, flags);
	raw_spin_lock_irqsave(&cluster->secondary_lock, flags_secondary);
	
	
	all_tasks_acedf[currentNumberTasks_acedf[cluster_id]] = t;
	all_ratios[cluster_id][currentNumberTasks_acedf[cluster_id]].ratio = -1;
	all_ratios[cluster_id][currentNumberTasks_acedf[cluster_id]].task = 0;
	currentNumberTasks_acedf[cluster_id]++;
	numberTasks_acedf++;
	t->rt_param.task_params.sorted_list_index = -1;
	
	/* TODO: allow for better setting of estimated execution
	 * time.
	 */
	t->rt_param.job_params.estimated_exec_time = 0;
	
	//we don't want tasks migrating initially
	t->rt_param.task_params.target_cpu = t->rt_param.task_params.cpu;
	

	
	/* setup job params */
	release_at(t, litmus_clock());

	if (is_scheduled) {
		entry = &per_cpu(acedf_cpu_entries, task_cpu(t));
		BUG_ON(entry->scheduled);

#ifdef CONFIG_RELEASE_MASTER

		if (entry->cpu != cluster->domain.release_master) {
#endif
			
			entry->scheduled = t;
			tsk_rt(t)->scheduled_on = task_cpu(t);
#ifdef CONFIG_RELEASE_MASTER
		} else {
			/* do not schedule on release master */
			preempt(entry); /* force resched */
			tsk_rt(t)->scheduled_on = NO_CPU;
		}
#endif
	} else {
		t->rt_param.scheduled_on = NO_CPU;
	}
	t->rt_param.linked_on          = NO_CPU;

	//No target service level to begin with 
	t->rt_param.task_params.target_service_level = tsk_rt(t)->ctrl_page->service_level;
	
	if (is_running(t))
		acedf_job_arrival(t);
	
	raw_spin_unlock_irqrestore(&cluster->secondary_lock, flags_secondary);
	raw_spin_unlock_irqrestore(&(cluster->cluster_lock), flags);
}

static void acedf_task_wake_up(struct task_struct *task)
{
	unsigned long flags;
	unsigned long flags_secondary;
	lt_t now;
	acedf_domain_t *cluster;

	//TRACE_TASK(task, "wake_up at %llu\n", litmus_clock());

	cluster = task_cpu_cluster(task);

	raw_spin_lock_irqsave(&cluster->cluster_lock, flags);
	raw_spin_lock_irqsave(&cluster->secondary_lock, flags_secondary);
	
	now = litmus_clock();
	if (is_sporadic(task) && is_tardy(task, now)) {
		/* new sporadic release */
		release_at(task, now);
		sched_trace_task_release(task);
	}
	acedf_job_arrival(task);

	raw_spin_unlock_irqrestore(&cluster->secondary_lock, flags_secondary);
	raw_spin_unlock_irqrestore(&cluster->cluster_lock, flags);
}

static void acedf_task_block(struct task_struct *t)
{
	unsigned long flags;
	unsigned long flags_secondary; 
	acedf_domain_t *cluster;

	TRACE_TASK(t, "block at %llu\n", litmus_clock());

	cluster = task_cpu_cluster(t);

	/* unlink if necessary */
	raw_spin_lock_irqsave(&cluster->cluster_lock, flags);
	raw_spin_lock_irqsave(&cluster->secondary_lock, flags_secondary);

	
	unlink(t);
	
	raw_spin_unlock_irqrestore(&cluster->secondary_lock, flags_secondary);
	raw_spin_unlock_irqrestore(&cluster->cluster_lock, flags);

	BUG_ON(!is_realtime(t));
}


static void acedf_task_exit(struct task_struct * t)
{
	unsigned long flags;
	unsigned long flags_secondary;
	acedf_domain_t *cluster = task_cpu_cluster(t);

	/* unlink if necessary */
	raw_spin_lock_irqsave(&cluster->cluster_lock, flags);
	raw_spin_lock_irqsave(&cluster->secondary_lock, flags_secondary);
	
	
	remove_task_from_cluster(t, cluster->clusterID);
	numberTasks_acedf--;

	
	// Last one out, shut out the lights
	if(numberTasks_acedf ==0){
		changeNow_acedf=0; 
		initialStartTime_acedf = 0;
		lastRepartitionTime_acedf = 0;
		TRACE("No more tasks\n");
	}
	if(currentNumberTasks_acedf[cluster->clusterID] == 0){
		lastReweightTime_acedf[task_cpu_cluster(t)->clusterID] = 0;
		acedf_cluster_total_utilization[cluster->clusterID] = 0;
		acedf_cluster_total_QoS[cluster->clusterID] = 0;
		TRACE("The Cluster %d is empty\n", cluster->clusterID);
	}
	
	unlink(t);
	if (tsk_rt(t)->scheduled_on != NO_CPU) {
		cpu_entry_t *cpu;
		cpu = &per_cpu(acedf_cpu_entries, tsk_rt(t)->scheduled_on);
		cpu->scheduled = NULL;
		tsk_rt(t)->scheduled_on = NO_CPU;
	}
	
	
	
	
	raw_spin_unlock_irqrestore(&cluster->secondary_lock, flags_secondary);
	raw_spin_unlock_irqrestore(&cluster->cluster_lock, flags);

	BUG_ON(!is_realtime(t));
    TRACE_TASK(t, "RIP\n");
}

static long acedf_admit_task(struct task_struct* tsk)
{
	return (remote_cluster(task_cpu(tsk)) == task_cpu_cluster(tsk)) ?
			0 : -EINVAL;
}


#ifdef VERBOSE_INIT
static void print_cluster_topology(cpumask_var_t mask, int cpu)
{
	int chk;
	char buf[255];

	chk = cpulist_scnprintf(buf, 254, mask);
	buf[chk] = '\0';
	printk(KERN_INFO "CPU = %d, shared cpu(s) = %s\n", cpu, buf);

}
#endif

static int clusters_allocated = 0;

static void cleanup_acedf(void)
{
	int i;

	if (clusters_allocated) {
		for (i = 0; i < num_clusters; i++) {
			kfree(acedf[i].cpus);
			kfree(acedf[i].heap_node);
			free_cpumask_var(acedf[i].cpu_map);
		}

		kfree(acedf);
	}
}

static struct domain_proc_info acedf_domain_proc_info;
static long acedf_get_domain_proc_info(struct domain_proc_info **ret)
{
	*ret = &acedf_domain_proc_info;
	return 0;
}

static void acedf_setup_domain_proc(void)
{
	int i, cpu, domain;
#ifdef CONFIG_RELEASE_MASTER
	int release_master = atomic_read(&release_master_cpu);
	/* skip over the domain with the release master if cluster size is 1 */
	int skip_domain = (1 == cluster_size && release_master != NO_CPU) ?
			release_master : NO_CPU;
#else
	int release_master = NO_CPU;
	int skip_domain = NO_CPU;
#endif
	int num_rt_cpus = num_online_cpus() - (release_master != NO_CPU);
	int num_rt_domains = num_clusters - (skip_domain != NO_CPU);
	struct cd_mapping *map;

	memset(&acedf_domain_proc_info, sizeof(acedf_domain_proc_info), 0);
	init_domain_proc_info(&acedf_domain_proc_info, num_rt_cpus, num_rt_domains);
	acedf_domain_proc_info.num_cpus = num_rt_cpus;
	acedf_domain_proc_info.num_domains = num_rt_domains;

	for (cpu = 0, i = 0; cpu < num_online_cpus(); ++cpu) {
		if (cpu == release_master)
			continue;
		map = &acedf_domain_proc_info.cpu_to_domains[i];
		/* pointer math to figure out the domain index */
		domain = remote_cluster(cpu) - acedf;
		map->id = cpu;
		cpumask_set_cpu(domain, map->mask);
		++i;
	}

	for (domain = 0, i = 0; domain < num_clusters; ++domain) {
		if (domain == skip_domain)
			continue;
		map = &acedf_domain_proc_info.domain_to_cpus[i];
		map->id = i;
		cpumask_copy(map->mask, acedf[domain].cpu_map);
		++i;
	}
}

static long acedf_activate_plugin(void)
{
	int i, j, cpu, ccpu, cpu_count;
	cpu_entry_t *entry;
	int chk = 0;



	//added 
	cpumask_var_t mask;
	
	for (i = 0;i < MAX_CLUSTERS;i++){
		lastReweightTime_acedf[i] = 0;
	}
	lastRepartitionTime_acedf = 0;
	initialStartTime_acedf=0;
	changeNow_acedf=0; 
	numberTasks_acedf = 0;
	
	
	for(i =0; i< MAX_CLUSTERS; i++ ){
		acedf_cluster_total_utilization[i]=0;
		acedf_cluster_total_QoS[i] = 0;
	}
	
	

	/* de-allocate old clusters, if any */
	cleanup_acedf();

	printk(KERN_INFO "C-EDF: Activate Plugin, cluster configuration = %d\n",
			cluster_config);
			
	/* initailize the global lock */
	raw_spin_lock_init(&global_lock);
	TRACE("INIT GLOBAL lock:Schedule \n");

	/* need to get cluster_size first */
	if(!zalloc_cpumask_var(&mask, GFP_ATOMIC))
		return -ENOMEM;

	if (cluster_config == GLOBAL_CLUSTER) {
		cluster_size = num_online_cpus();
	} else {
		chk = get_shared_cpu_map(mask, 0, cluster_config);
		if (chk) {
			/* if chk != 0 then it is the max allowed index */
			printk(KERN_INFO "C-EDF: Cluster configuration = %d "
			       "is not supported on this hardware.\n",
			       cluster_config);
			/* User should notice that the configuration failed, so
			 * let's bail out. */
			return -EINVAL;
		}

		cluster_size = cpumask_weight(mask);
	}

	if ((num_online_cpus() % cluster_size) != 0) {
		/* this can't be right, some cpus are left out */
		printk(KERN_ERR "C-EDF: Trying to group %d cpus in %d!\n",
				num_online_cpus(), cluster_size);
		return -1;
	}

	num_clusters = num_online_cpus() / cluster_size;
	acedf_number_of_clusters = num_clusters;
	printk(KERN_INFO "C-EDF: %d cluster(s) of size = %d\n",
			num_clusters, cluster_size);

	/* initialize clusters */
	acedf = kmalloc(num_clusters * sizeof(acedf_domain_t), GFP_ATOMIC);
	for (i = 0; i < num_clusters; i++) {
		
		acedf[i].clusterID = i;
		acedf[i].representative_CPU = -1;
		
		acedf[i].cpus = kmalloc(cluster_size * sizeof(cpu_entry_t),
				GFP_ATOMIC);
		acedf[i].heap_node = kmalloc(
				cluster_size * sizeof(struct bheap_node),
				GFP_ATOMIC);
		raw_spin_lock_init(&acedf[i].secondary_lock);
		bheap_init(&(acedf[i].cpu_heap));
		edf_domain_init(&(acedf[i].domain), NULL, acedf_release_jobs);

		if(!zalloc_cpumask_var(&acedf[i].cpu_map, GFP_ATOMIC))
			return -ENOMEM;
#ifdef CONFIG_RELEASE_MASTER
		acedf[i].domain.release_master = atomic_read(&release_master_cpu);
#endif
	} 



	/* cycle through cluster and add cpus to them */
	for (i = 0; i < num_clusters; i++) {
		currentNumberTasks_acedf[i] = 0;
		
		for_each_online_cpu(cpu) {
			/* check if the cpu is already in a cluster */
			for (j = 0; j < num_clusters; j++)
				if (cpumask_test_cpu(cpu, acedf[j].cpu_map))
					break;
			/* if it is in a cluster go to next cpu */
			if (j < num_clusters &&
					cpumask_test_cpu(cpu, acedf[j].cpu_map))
				continue;

			/* this cpu isn't in any cluster */
			/* get the shared cpus */ 
			if (unlikely(cluster_config == GLOBAL_CLUSTER))
				cpumask_copy(mask, cpu_online_mask);
			else
				get_shared_cpu_map(mask, cpu, cluster_config);

			cpumask_copy(acedf[i].cpu_map, mask);
#ifdef VERBOSE_INIT
			print_cluster_topology(mask, cpu);
#endif
			//TODO: Validate 
		 	if ( acedf[i].representative_CPU == -1 ) {
				//acedf[i].representative_CPU = cpu;
				TRACE("Setting cluster number %d to %d\n", i,cpu);			
			} else {
				TRACE("Cluster number %d already set to %d\n", i,acedf[i].representative_CPU);			
			}
			acedf[i].representative_CPU = cpu;
			
			
			
			/* add cpus to current cluster and init cpu_entry_t */
			cpu_count = 0;
			for_each_cpu(ccpu, acedf[i].cpu_map) {

				entry = &per_cpu(acedf_cpu_entries, ccpu);
				acedf[i].cpus[cpu_count] = entry;
				atomic_set(&entry->will_schedule, 0);
				entry->cpu = ccpu;
				entry->cluster = &acedf[i];
				entry->hn = &(acedf[i].heap_node[cpu_count]);
				bheap_node_init(&entry->hn, entry);

				cpu_count++;

				entry->linked = NULL;
				entry->scheduled = NULL;
#ifdef CONFIG_RELEASE_MASTER
				/* only add CPUs that should schedule jobs */
				if (entry->cpu != entry->cluster->domain.release_master)
#endif
					update_cpu_position(entry);
			}
			/* done with this cluster */
			break;
		}
	}

	clusters_allocated = 1;
	free_cpumask_var(mask);

	acedf_setup_domain_proc();

	return 0;
}

static long acedf_deactivate_plugin(void)
{
	destroy_domain_proc_info(&acedf_domain_proc_info);
	return 0;
}

/*	Plugin object	*/
static struct sched_plugin acedf_plugin __cacheline_aligned_in_smp = {
	.plugin_name		= "AC-EDF",
	.finish_switch		= acedf_finish_switch,
	.task_new		= acedf_task_new,
	.complete_job		= complete_job,
	.task_exit		= acedf_task_exit,
	.schedule		= acedf_schedule,
	.task_wake_up		= acedf_task_wake_up,
	.task_block		= acedf_task_block,
	.admit_task		= acedf_admit_task,
	.activate_plugin	= acedf_activate_plugin,
	.deactivate_plugin	= acedf_deactivate_plugin,
	.get_domain_proc_info	= acedf_get_domain_proc_info,
};

static struct proc_dir_entry *cluster_file = NULL, *acedf_dir = NULL;

static int __init init_acedf(void)
{
	int err, fs;

	err = register_sched_plugin(&acedf_plugin);
	if (!err) {
		fs = make_plugin_proc_dir(&acedf_plugin, &acedf_dir);
		if (!fs)
			cluster_file = create_cluster_file(acedf_dir, &cluster_config);
		else
			printk(KERN_ERR "Could not allocate C-EDF procfs dir.\n");
	}
	return err;
}

static void clean_acedf(void)
{
	cleanup_acedf();
	if (cluster_file)
		remove_proc_entry("cluster", acedf_dir);
	if (acedf_dir)
		remove_plugin_proc_dir(&acedf_plugin);
}

module_init(init_acedf);
module_exit(clean_acedf);
