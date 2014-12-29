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
	/* lock for this cluster */
#define cluster_lock domain.ready_lock
} acedf_domain_t;

/* a acedf_domain per cluster; allocation is done at init/activation time */
acedf_domain_t *acedf;

#define remote_cluster(cpu)	((acedf_domain_t *) per_cpu(acedf_cpu_entries, cpu).cluster)
#define task_cpu_cluster(task)	remote_cluster(get_partition(task))

/* Uncomment WANT_ALL_SCHED_EVENTS if you want to see all scheduling
 * decisions in the TRACE() log; uncomment VERBOSE_INIT for verbose
 * information during the initialization of the plugin (e.g., topology)
#define WANT_ALL_SCHED_EVENTS
 */
#define VERBOSE_INIT

//TODO-AARON: The number here limits the number of real time threads. 
//Need to make this more general or fix
//Should make this linked list to be more general and flexible
static struct task_struct* all_tasks_acedf[100]; 


//TODO-AARON: This is part of the hacky fix above used to keep track of the number
//of tasks in the system and add them to the task_struct. 
static int currentNumberTasks_acedf;

static double agsnedf_total_utilization_acedf;

static const lt_t nanosecondsBetweenReweights_acedf = 1000000000/20; //Number of times per second that the system is weighted at a minimum;
static lt_t lastReweightTime_acedf; //time used to keep track of the last time the task was reweighted
static lt_t initialStartTime_acedf;//Time the first task starts
static lt_t initialStableWindowTime_acedf=((lt_t)10)*((lt_t)1000000000); //For the first 30 seconds, nothing happens. 
static int changeNow_acedf; //if 1, under utilized. if 0, nothing. -1 if over utilized


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
				TRACE_TASK(linked,
					   "already scheduled on %d, updating link.\n",
					   sched->cpu);
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
	//acedf_domain_t *oldcluster = task_cpu_cluster(task);
	
	
	///****** ADD CHECK HERE FOR VALIDATING acedf[i].representativeID
	//TODO: remove this code, it's here for an experiment
// 	TRACE("ACEDF**&&**: Task , %d, Intially on %d\n", task->pid, task_cpu_cluster(task)->clusterID);
// 	if (oldcluster->clusterID==0){
// 		task->rt_param.task_params.cpu = cluster;
// 	} else {
// 		task->rt_param.task_params.cpu = 0;
// 	}
 	cluster = task_cpu_cluster(task);
// 	
 	TRACE("ACEDF**&&**: Task , %d, Now on on %d\n", task->pid, task_cpu_cluster(task)->clusterID);
	BUG_ON(!task);
	/* sanity check before insertion */
	BUG_ON(is_queued(task));

	if (is_early_releasing(task) || is_released(task, litmus_clock()))
		__add_ready(&cluster->domain, task);
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
#ifdef CONFIG_RELEASE_MASTER
	    && likely(local->cpu != cluster->domain.release_master)
#endif
		) {
		task = __take_ready(&cluster->domain);
		TRACE_TASK(task, "linking to local CPU %d to avoid IPI\n", local->cpu);
		link_task_to_cpu(task, local);
		preempt(local);
	}
#endif


	for(last = lowest_prio_cpu(cluster);
	    edf_preemption_needed(&cluster->domain, last->linked);
	    last = lowest_prio_cpu(cluster)) {
		/* preemption necessary */
		task = __take_ready(&cluster->domain);
		TRACE("check_for_preemptions: attempting to link task %d to %d\n",
		      task->pid, last->cpu);
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

	raw_spin_lock_irqsave(&cluster->cluster_lock, flags);

	__merge_ready(&cluster->domain, tasks);
	check_for_preemptions(cluster);

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
 
	t->rt_param.job_params.estimated_exec_time = 
		(lt_t)	(p * t->rt_param.current_diff_est_actual_exec_cost + 
				 i * t->rt_param.cumulative_diff_est_actual_exec_cost);		
	return;
}

//added
//Should only be called by job_completion. Thus, caller holds adgnedf_lock
static noinline void adjust_all_service_levels_acedf(int triggerNow){
	int count;
	struct task_struct *temp;
	//TODO: Adjust all the service levels of all the jobs if a trigger threshold is met.
	// This is how tsk_rt(t)->ctrl_page->service_level;
	const int number_of_cpus_held_back = 5; //Note: On my 12 core (24 virtual processors)
	//If this is under 5, then the system crashes. (Thus, the system runs on 19 virtual cores)
	struct task_struct* local_copy[currentNumberTasks_acedf];
	int taskLevel[currentNumberTasks_acedf];
	double weightLevel[currentNumberTasks_acedf];
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
	double maxUtilization = num_online_cpus()-number_of_cpus_held_back;
	//This is the max_level 
	const int max_level = 2; //max level should be 3, but crashing. So let's try 2
	const int lowest_level = 0; //lowest service level level should be 3, but crashing. So let's try 2
	double calculationFactor;
	
	//Improve the conditions here so it's time based not task release based. 
	if( triggerNow!=0 ) { 
		//Copying array to local copy
		for(count = 0;count < currentNumberTasks_acedf; count++) {
			local_copy[count] = all_tasks_acedf[count];
		}
	

		//sort local_copy based on value density.. Assumes linear relationship
		// The rather complicated formulas in here come from Aaron Block's dissertation
		// specifically formula 6.11 on page 258
		// you can get Aaron's dissertation here: http://cs.unc.edu/~block/aarondiss.pdf
		
		for(outerIndex = 1; outerIndex <= currentNumberTasks_acedf - 1 ; outerIndex++) {
			innerIndex = outerIndex;
		
			upperIndex = innerIndex;
			lowerIndex = upperIndex-1;
			QoSUpperIndex = tsk_rt(local_copy[upperIndex])->task_params.service_levels[max_level].quality_of_service - tsk_rt(local_copy[upperIndex])->task_params.service_levels[lowest_level].quality_of_service;
			QoSLowerIndex = tsk_rt(local_copy[lowerIndex])->task_params.service_levels[max_level].quality_of_service - tsk_rt(local_copy[lowerIndex])->task_params.service_levels[lowest_level].quality_of_service;

			estWeightDiffUpper = (get_estimated_weight(local_copy[upperIndex])/tsk_rt(local_copy[upperIndex])->task_params.service_levels[tsk_rt(local_copy[upperIndex])->ctrl_page->service_level].relative_work) * 
			(tsk_rt(local_copy[upperIndex])->task_params.service_levels[max_level].relative_work - 1 );
			if (estWeightDiffUpper <= 0 ) {
				valueDensityUpperIndex = DBL_MAX;
			} else {
				valueDensityUpperIndex =  QoSUpperIndex / estWeightDiffUpper;
			}
		
			estWeightDiffLower = (get_estimated_weight(local_copy[lowerIndex])/tsk_rt(local_copy[lowerIndex])->task_params.service_levels[tsk_rt(local_copy[lowerIndex])->ctrl_page->service_level].relative_work) * 
			(tsk_rt(local_copy[lowerIndex])->task_params.service_levels[max_level].relative_work - 1 );
			if (estWeightDiffLower <= 0 ) {
				valueDensityLowerIndex = DBL_MAX;
			} else {
				valueDensityLowerIndex =  QoSLowerIndex / estWeightDiffLower;
			}


			while (innerIndex > 0 && ( valueDensityUpperIndex > valueDensityLowerIndex)){

				temp = local_copy[upperIndex];
				local_copy[upperIndex] = local_copy[lowerIndex];
				local_copy[lowerIndex] = temp;
			
				innerIndex--;
				if (innerIndex > 0 ) {
					upperIndex = innerIndex;
					lowerIndex = upperIndex-1;
			
					QoSUpperIndex = tsk_rt(local_copy[upperIndex])->task_params.service_levels[max_level].quality_of_service - tsk_rt(local_copy[upperIndex])->task_params.service_levels[lowest_level].quality_of_service;
					QoSLowerIndex = tsk_rt(local_copy[lowerIndex])->task_params.service_levels[max_level].quality_of_service - tsk_rt(local_copy[lowerIndex])->task_params.service_levels[lowest_level].quality_of_service;

					estWeightDiffUpper = (get_estimated_weight(local_copy[upperIndex])/tsk_rt(local_copy[upperIndex])->task_params.service_levels[tsk_rt(local_copy[upperIndex])->ctrl_page->service_level].relative_work) * 
					(tsk_rt(local_copy[upperIndex])->task_params.service_levels[max_level].relative_work - 1 );
					if (estWeightDiffUpper <= 0 ) {
						valueDensityUpperIndex = DBL_MAX;
					} else {
						valueDensityUpperIndex =  QoSUpperIndex / estWeightDiffUpper;
					}
		
					estWeightDiffLower = (get_estimated_weight(local_copy[lowerIndex])/tsk_rt(local_copy[lowerIndex])->task_params.service_levels[tsk_rt(local_copy[lowerIndex])->ctrl_page->service_level].relative_work) * 
					(tsk_rt(local_copy[lowerIndex])->task_params.service_levels[max_level].relative_work - 1 );
					if (estWeightDiffLower <= 0 ) {
						valueDensityLowerIndex = DBL_MAX;
					} else {
						valueDensityLowerIndex =  QoSLowerIndex / estWeightDiffLower;
					}
				}
			}
		}
		//Let's validate sorted. 
		//TODO: Remove this. 
		for(lowerIndex =0; lowerIndex< currentNumberTasks_acedf;lowerIndex++){
			QoSLowerIndex = tsk_rt(local_copy[lowerIndex])->task_params.service_levels[max_level].quality_of_service - tsk_rt(local_copy[lowerIndex])->task_params.service_levels[lowest_level].quality_of_service;
			estWeightDiffLower = (get_estimated_weight(local_copy[lowerIndex])/tsk_rt(local_copy[lowerIndex])->task_params.service_levels[tsk_rt(local_copy[lowerIndex])->ctrl_page->service_level].relative_work) * 
					(tsk_rt(local_copy[lowerIndex])->task_params.service_levels[max_level].relative_work - 1 );
			if (estWeightDiffLower <= 0 ) {
				valueDensityLowerIndex = DBL_MAX;
			} else {
				valueDensityLowerIndex =  QoSLowerIndex / estWeightDiffLower;
			}

			TRACE("Task %i, value Density %d, QoS Diff %d, weightDiff %d, valu\n", lowerIndex,(int)(valueDensityLowerIndex*1000), (int)(QoSLowerIndex*1000), (int)(estWeightDiffLower*1000));
			
					
		}
		//Local_copy is now sorted
		
		//Now need to maximize
		//Step 1 : Set all tasks to base level (level 0)
		//Step 2 : Increase tasks from top to lowest_priorty_cpu()
		//Step 3 : taskLevel

		localTotalUtilization = 0;
	
		//Step 1: set all tasks to their base service level (level 0)
		//This allows for us to get an assessment of how much "extra" capacity we have
		
		for(outerIndex=0; outerIndex < currentNumberTasks_acedf; outerIndex++) {
			//taskLevel corresponds to the calculated desired index for all elements
			//as sorted by the above. 
			//Since everything starts off at zero, taskLevel starts at zero.
			taskLevel[outerIndex] = 0;
			
		
			// If it is 0, then the estimated weight is the correct amount to to add to
			// localTotalUtiization 
			// FYI aConvertValue is a temp used for tracing
			if (tsk_rt(local_copy[outerIndex])->ctrl_page->service_level==0) {
				weightLevel[outerIndex] = get_estimated_weight(local_copy[outerIndex]);
				
				//aConvertValue is just a temp used for tracing
				aConvertValue = 10000*get_estimated_weight(local_copy[outerIndex]);
				TRACE("-LEVEL 0: Task %d, weight %d\n", outerIndex, aConvertValue);
				
			} else {
				//If a task has a service level that isn't currently zero, then
				//we need to calculate it's weight if it were at service level zero. 
				weightLevel[outerIndex] = get_estimated_weight(local_copy[outerIndex]) / tsk_rt(local_copy[outerIndex])->task_params.service_levels[tsk_rt(local_copy[outerIndex])->ctrl_page->service_level].relative_work;
				
				aConvertValue = 10000*get_estimated_weight(local_copy[outerIndex]);
				TRACE("-LEVEL %d: Task %d, weight %d\n", tsk_rt(local_copy[outerIndex])->ctrl_page->service_level, outerIndex, aConvertValue);
			}
			
			aConvertValue = 10000*weightLevel[outerIndex];
			TRACE("Task %d, base weight %d\n", outerIndex, aConvertValue);
			
			//Calculate the total utilization. 
			localTotalUtilization+=weightLevel[outerIndex];
			
			
			aConvertValue = 10000*localTotalUtilization;
			TRACE("Total Utilization %d\n", aConvertValue);
		}
		
		//Step 2 : Increase tasks from top to lowest_priorty_cpu()
		//Since all tasks are in sorted order at service level 0
		//we start from the first task and attempt to increase its service level 
		//to the maximum value without overloading the system. 
		//Even if we overload on one task, we still keep on increasing other tasks
		//because we might be able to increase their levels slightly 
		
		for(outerIndex=0; outerIndex < currentNumberTasks_acedf; outerIndex++) {

			//This assumes that all tasks have the same max service level
			//Increase servie level to the maximum possible value. 
			//We start at 1 because every task is already at zero. 			
			for( innerIndex = 1; innerIndex <= max_level; innerIndex++) {
				//The value the weight would be if we increased the weight of the task
				calculationFactor = weightLevel[outerIndex] * tsk_rt(local_copy[outerIndex])->task_params.service_levels[innerIndex].relative_work;
				
				aConvertValue = 10000*calculationFactor;
				TRACE("Temp Level %d, Task %d, Calculation Factor %d\n", innerIndex, outerIndex, aConvertValue);
				
				//If we increased it, would the system be over utilized?
				if( (localTotalUtilization-weightLevel[outerIndex]+calculationFactor) < maxUtilization) {
					TRACE("Increase task %d, to Level %d\n", outerIndex, innerIndex);
					//Increasing the weight of the task
					taskLevel[outerIndex] = innerIndex;
				}
			}
			
			//Change the total utilization by subtracting the old weight and adding the new weight
			//We don't actually change the weight here though
			localTotalUtilization = localTotalUtilization-weightLevel[outerIndex] + weightLevel[outerIndex]* tsk_rt(local_copy[outerIndex])->task_params.service_levels[taskLevel[outerIndex]].relative_work;
			aConvertValue = weightLevel[outerIndex];
			TRACE("Task %d, subtracted Weight %d\n", outerIndex, aConvertValue);
			
			aConvertValue = 10000*weightLevel[outerIndex]* tsk_rt(local_copy[outerIndex])->task_params.service_levels[taskLevel[outerIndex]].relative_work;
			TRACE("Task %d, added Weight %d\n", outerIndex, aConvertValue);
			
			aConvertValue = 10000*localTotalUtilization;
			TRACE("Task %d, the total Utilization %d\n", outerIndex, aConvertValue);
// 			for(innerIndex = tsk_rt(local_copy[outerIndex])->ctrl_page->service_level; innerIndex < max_level; innerIndex) { 
// 				
// 			}
		
		}
		
		// Go through and actually change the weight of each task now that all the work is done.
		for(outerIndex=0; outerIndex < currentNumberTasks_acedf; outerIndex++) {
			tsk_rt(local_copy[outerIndex])->ctrl_page->service_level = taskLevel[outerIndex];
			TRACE("&&The service level for %d is %d\n", outerIndex, taskLevel[outerIndex]);
		}
	}	
}

/* caller holds acedf_lock */
static noinline void job_completion(struct task_struct *t, int forced)
{

	//TODO: Change this if I want to trigger a weight change when we have fewer than 2 CPUS left
	const int bufferCPUs = 5; //Note: On my 12 core (24 virtual processors)
	//If this is under 5, then the system crashes. (Thus, the system runs on 19 virtual cores)
	double old_est_weight;
	double difference_in_weight;
	int triggerNow = 0;
	double maxUtilization;
	const double PERCENT_CHANGE_TRIGGER = 0.125; // If the task's weight changes by this percentage
	// between job releases, then trigger an immediate reweighting


	BUG_ON(!t);

	sched_trace_task_completion(t, forced);

	TRACE_TASK(t, "job_completion().\n");
	TRACE("ACEDF: Task , %d, complete on %d\n", t->pid, task_cpu_cluster(t)->clusterID);

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
	
	//TODO: Make a flag for the scheduler base on whether service levels are adjusted
	//based on time or trigger level. 
	
	/* The change in the estimated_weight of this job must be added to the total 
	 * utilization
	 */
 
	difference_in_weight = get_estimated_weight(t) - old_est_weight;

	agsnedf_total_utilization_acedf += difference_in_weight;
	
	
	//When the jobs are released, then start the initiali timers;	
	if(initialStartTime_acedf ==0){
		initialStartTime_acedf = litmus_clock();
		TRACE("Starting NOW %llu\n", initialStartTime_acedf);
	}
	if(lastReweightTime_acedf == 0){
		lastReweightTime_acedf = litmus_clock();
		TRACE("RewightingWindow now %llu\n", lastReweightTime_acedf);
	}
	
	
	triggerNow = 0;
	
	
	maxUtilization = num_online_cpus()-bufferCPUs;
	TRACE("The utilization times 100 is %d\n", (int)(agsnedf_total_utilization_acedf*100));
	//Trigger because the system is too utilized
	if( agsnedf_total_utilization_acedf > maxUtilization) {
		TRACE("TRIGGER\n");
		triggerNow = 1;
	}
	
	//trigger because a task changed its weight by too much
	if( (get_estimated_weight(t) > old_est_weight*(1+PERCENT_CHANGE_TRIGGER)) ||
		(get_estimated_weight(t) < old_est_weight*(1-PERCENT_CHANGE_TRIGGER)))
	{
		TRACE("Individual task trigger\n");
		triggerNow = 1;
	}
	
	//trigger because enough time has passed
	if ( (lastReweightTime_acedf + nanosecondsBetweenReweights_acedf) < litmus_clock()){
		triggerNow = 1;
		lastReweightTime_acedf = litmus_clock();
		TRACE("Reweighting NOW at %llu\n", lastReweightTime_acedf);
	}

	// if we aren't past the initial window, then don't reweight
	if((initialStartTime_acedf+initialStableWindowTime_acedf) > litmus_clock()){
		if(triggerNow==1){
			TRACE("Still to early. Nothing until %llu, now %llu\n", initialStartTime_acedf+initialStableWindowTime_acedf, litmus_clock());
		}
		triggerNow = 0;
	}
	
	adjust_all_service_levels_acedf(triggerNow);
	
	//This line is also incorect. No idea why it would let me use it
	//t->rt_param.job_params.current_service_level+=30;
	TRACE("***TASK SERVICE LEVEL :%u\n",tsk_rt(t)->ctrl_page->service_level);
	TRACE("A-GSN-EDF-Period: %llu\n", tsk_rt(t)->task_params.period);
	TRACE("A-GSN-EDF-Deadline: %llu\n", tsk_rt(t)->task_params.relative_deadline);
	
	
	/* prepare for next period */
	prepare_for_next_period(t);
	if (is_early_releasing(t) || is_released(t, litmus_clock()))
		sched_trace_task_release(t);
	/* unlink */
	unlink(t);
	/* requeue
	 * But don't requeue a blocking task. */
	if (is_running(t))
		acedf_job_arrival(t);
	TRACE("ACEDF**&&**: Task , %d, complete on %d\n", t->pid, task_cpu_cluster(t)->clusterID);
}


//TODO: *****
//TODO: This needs to be changed from moving a task from one partition to another
//To one cluster from another. 
//TODO*****
/* called with preemptions disabled */
// static void acedf_migrate_to(struct task_struct* task_to_migrate, acedf_domain_t* target_cluster)
// {
// 	struct task_struct* t = task_to_migrate;
// 	acedf_domain_t* from;
// 
// 	if (task_cpu_cluster(t) == target_cluster)
// 		return;
// 
// 	/* make sure target_cpu makes sense */
// 	//Makes no sense in current context
// 	//BUG_ON(!cpu_online(target_cpu));
// 
// 	local_irq_disable();
// 
// 	from = task_cpu_cluster(t);
// 	raw_spin_lock(&from->cluster_lock);
// 
// 	/* Scheduled task should not be in any ready or release queue.  Check
// 	 * this while holding the lock to avoid RT mode transitions.*/
// 	BUG_ON(is_realtime(t) && is_queued(t));
// 
// 	/* switch clusters */
// 	
// 	//to move a task from one cluster to anthother. If it is not queued, then all you
// 	//need to do is change its CPU from one in a cluster into another in a different one
// 	tsk_rt(t)->task_params.cpu = target_cluster->representative_CPU;
// 	
// 
// 	raw_spin_unlock(&from->cluster_lock);
// 
// 	/* Don't trace scheduler costs as part of
// 	 * locking overhead. Scheduling costs are accounted for
// 	 * explicitly. */
// 	TS_LOCK_SUSPEND;
// 
// 	local_irq_enable();
// 	preempt_enable_no_resched();
// 
// 	/* deschedule to be migrated */
// 	schedule();
// 
// 	/* we are now on the target processor */
// 	preempt_disable();
// 
// 	/* start recording costs again */
// 	TS_LOCK_RESUME;
//  
// 	//Commented out because it makes no sense in current context
// 	//BUG_ON(smp_processor_id() != target_cpu && is_realtime(t));
// }



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
#endif

	if (exists)
		TRACE_TASK(prev,
			   "blocks:%d out_of_time:%d np:%d sleep:%d preempt:%d "
			   "state:%d sig:%d\n",
			   blocks, out_of_time, np, sleep, preempt,
			   prev->state, signal_pending(prev));
	if (entry->linked && preempt)
		TRACE_TASK(prev, "will be preempted by %s/%d\n",
			   entry->linked->comm, entry->linked->pid);


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
		job_completion(entry->scheduled, !sleep);

	/* Link pending task if we became unlinked.
	 */
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
			TRACE_TASK(entry->scheduled, "scheduled_on = NO_CPU\n");
		}
	} else
		/* Only override Linux scheduler if we have a real-time task
		 * scheduled that needs to continue.
		 */
		if (exists)
			next = prev;

	sched_state_task_picked();
	raw_spin_unlock(&cluster->cluster_lock);

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
	cpu_entry_t* 		entry;
	acedf_domain_t*		cluster;
	int localNumber; 

	TRACE("acedf edf: task new %d\n", t->pid);

	/* the cluster doesn't change even if t is scheduled */
	cluster = task_cpu_cluster(t);
	
	TRACE("ACEDF: Task , %d, is initially assigned to %d\n", t->pid, cluster->clusterID);

	raw_spin_lock_irqsave(&cluster->cluster_lock, flags);

	localNumber = currentNumberTasks_acedf; 
	all_tasks_acedf[currentNumberTasks_acedf] = t;
	currentNumberTasks_acedf++;
	
	/* TODO: allow for better setting of estimated execution
	 * time.
	 */
	t->rt_param.job_params.estimated_exec_time = 0;
	
	
	/* setup job params */
	release_at(t, litmus_clock());

	if (is_scheduled) {
		entry = &per_cpu(acedf_cpu_entries, task_cpu(t));
		BUG_ON(entry->scheduled);

#ifdef CONFIG_RELEASE_MASTER
		//TODO: Remove the following line, I'm putting fo clarity
		TRACE("Yup: Release master configured\n");
		if (entry->cpu != cluster->domain.release_master) {
#endif
			//TODO: Remove the following line, I'm putting fo clarity
			TRACE("Nope: release master unconfugred\n");
			
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

	if (is_running(t))
		acedf_job_arrival(t);
	raw_spin_unlock_irqrestore(&(cluster->cluster_lock), flags);
}

static void acedf_task_wake_up(struct task_struct *task)
{
	unsigned long flags;
	lt_t now;
	acedf_domain_t *cluster;

	TRACE_TASK(task, "wake_up at %llu\n", litmus_clock());

	cluster = task_cpu_cluster(task);

	raw_spin_lock_irqsave(&cluster->cluster_lock, flags);
	now = litmus_clock();
	if (is_sporadic(task) && is_tardy(task, now)) {
		/* new sporadic release */
		release_at(task, now);
		sched_trace_task_release(task);
	}
	acedf_job_arrival(task);
	raw_spin_unlock_irqrestore(&cluster->cluster_lock, flags);
}

static void acedf_task_block(struct task_struct *t)
{
	unsigned long flags;
	acedf_domain_t *cluster;

	TRACE_TASK(t, "block at %llu\n", litmus_clock());

	cluster = task_cpu_cluster(t);

	/* unlink if necessary */
	raw_spin_lock_irqsave(&cluster->cluster_lock, flags);
	unlink(t);
	raw_spin_unlock_irqrestore(&cluster->cluster_lock, flags);

	BUG_ON(!is_realtime(t));
}


static void acedf_task_exit(struct task_struct * t)
{
	unsigned long flags;
	acedf_domain_t *cluster = task_cpu_cluster(t);

	/* unlink if necessary */
	raw_spin_lock_irqsave(&cluster->cluster_lock, flags);
	unlink(t);
	if (tsk_rt(t)->scheduled_on != NO_CPU) {
		cpu_entry_t *cpu;
		cpu = &per_cpu(acedf_cpu_entries, tsk_rt(t)->scheduled_on);
		cpu->scheduled = NULL;
		tsk_rt(t)->scheduled_on = NO_CPU;
	}
	raw_spin_unlock_irqrestore(&cluster->cluster_lock, flags);

	BUG_ON(!is_realtime(t));
        TRACE_TASK(t, "RIP\n");
}

static long acedf_admit_task(struct task_struct* tsk)
{
	return (remote_cluster(task_cpu(tsk)) == task_cpu_cluster(tsk)) ?
			0 : -EINVAL;
}

/* total number of cluster */
static int num_clusters;
/* we do not support cluster of different sizes */
static unsigned int cluster_size;

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
	agsnedf_total_utilization_acedf = 0;
	lastReweightTime_acedf=0;
	initialStartTime_acedf=0;
	changeNow_acedf=0; 
	currentNumberTasks_acedf = 0;
	
	

	/* de-allocate old clusters, if any */
	cleanup_acedf();

	printk(KERN_INFO "C-EDF: Activate Plugin, cluster configuration = %d\n",
			cluster_config);

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
	printk(KERN_INFO "C-EDF: %d cluster(s) of size = %d\n",
			num_clusters, cluster_size);

	/* initialize clusters */
	acedf = kmalloc(num_clusters * sizeof(acedf_domain_t), GFP_ATOMIC);
	for (i = 0; i < num_clusters; i++) {
		
		acedf[i].clusterID = i;
		//acedf[i].representative_CPU = -1;
		
		acedf[i].cpus = kmalloc(cluster_size * sizeof(cpu_entry_t),
				GFP_ATOMIC);
		acedf[i].heap_node = kmalloc(
				cluster_size * sizeof(struct bheap_node),
				GFP_ATOMIC);
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
		// 	if ( acedf[i].representative_CPU == -1 ) {
// 				//acedf[i].representative_CPU = cpu;
// 				TRACE("Setting cluster number %d to %d\n", i,cpu);			
// 			} else {
// 				TRACE("Cluster number %d already set to %d\n", i,acedf[i].representative_CPU);			
// 			}
// 			acedf[i].representative_CPU = cpu;
			
			
			
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
