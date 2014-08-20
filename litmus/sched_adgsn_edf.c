//TODO: Remove any bugs that might occur by ovverunns
//TODO: Clean up code to get rid of locking from "coppying" GSN-EDF
/*
 * litmus/sched_adgsn_edf.c
 *
 * Implementation of the AGSN-EDF scheduling algorithm.
 * This file is a modification of the GSN-EDF scheduling algorithim
 *
 * This version uses the simple approach and serializes all scheduling
 * decisions by the use of a queue lock. This is probably not the
 * best way to do it, but it should suffice for now.
 */

#include <linux/spinlock.h>
#include <linux/percpu.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <litmus/litmus.h>
#include <litmus/jobs.h>
#include <litmus/sched_plugin.h>
#include <litmus/edf_common.h>
#include <litmus/sched_trace.h>
#include <litmus/trace.h>

#include <litmus/preempt.h>
#include <litmus/budget.h>

#include <litmus/bheap.h>

#ifdef CONFIG_SCHED_CPU_AFFINITY
#include <litmus/affinity.h>
#endif

/* to set up domain/cpu mappings */
#include <litmus/litmus_proc.h>

#include <linux/module.h>

#include <float.h>

/* Overview of adgsn-EDF operations.
 *
 * For a detailed explanation of adgsn-EDF have a look at the FMLP paper. This
 * description only covers how the individual operations are implemented in
 * LITMUS.
 *
 * link_task_to_cpu(T, cpu) 	- Low-level operation to update the linkage
 *                                structure (NOT the actually scheduled
 *                                task). If there is another linked task To
 *                                already it will set To->linked_on = NO_CPU
 *                                (thereby removing its association with this
 *                                CPU). However, it will not requeue the
 *                                previously linked task (if any). It will set
 *                                T's state to not completed and check whether
 *                                it is already running somewhere else. If T
 *                                is scheduled somewhere else it will link
 *                                it to that CPU instead (and pull the linked
 *                                task to cpu). T may be NULL.
 *
 * unlink(T)			- Unlink removes T from all scheduler data
 *                                structures. If it is linked to some CPU it
 *                                will link NULL to that CPU. If it is
 *                                currently queued in the adgsnedf queue it will
 *                                be removed from the rt_domain. It is safe to
 *                                call unlink(T) if T is not linked. T may not
 *                                be NULL.
 *
 * requeue(T)			- Requeue will insert T into the appropriate
 *                                queue. If the system is in real-time mode and
 *                                the T is released already, it will go into the
 *                                ready queue. If the system is not in
 *                                real-time mode is T, then T will go into the
 *                                release queue. If T's release time is in the
 *                                future, it will go into the release
 *                                queue. That means that T's release time/job
 *                                no/etc. has to be updated before requeu(T) is
 *                                called. It is not safe to call requeue(T)
 *                                when T is already queued. T may not be NULL.
 *
 * adgsnedf_job_arrival(T)	- This is the catch all function when T enters
 *                                the system after either a suspension or at a
 *                                job release. It will queue T (which means it
 *                                is not safe to call adgsnedf_job_arrival(T) if
 *                                T is already queued) and then check whether a
 *                                preemption is necessary. If a preemption is
 *                                necessary it will update the linkage
 *                                accordingly and cause scheduled to be called
 *                                (either with an IPI or need_resched). It is
 *                                safe to call adgsnedf_job_arrival(T) if T's
 *                                next job has not been actually released yet
 *                                (releast time in the future). T will be put
 *                                on the release queue in that case.
 *
 * job_completion(T)		- Take care of everything that needs to be done
 *                                to prepare T for its next release and place
 *                                it in the right queue with
 *                                adgsnedf_job_arrival().
 *
 *
 * When we now that T is linked to CPU then link_task_to_cpu(NULL, CPU) is
 * equivalent to unlink(T). Note that if you unlink a task from a CPU none of
 * the functions will automatically propagate pending task from the ready queue
 * to a linked task. This is the job of the calling function ( by means of
 * __take_ready).
 */


/* cpu_entry_t - maintain the linked and scheduled state
 */
 
typedef struct  {
	int 			cpu;
	struct task_struct*	linked;		/* only RT tasks */
	struct task_struct*	scheduled;	/* only RT tasks */
	struct bheap_node*	hn;
} cpu_entry_t;
DEFINE_PER_CPU(cpu_entry_t, adgsnedf_cpu_entries);

cpu_entry_t* adgsnedf_cpus[NR_CPUS];

/* the cpus queue themselves according to priority in here */
static struct bheap_node adgsnedf_heap_node[NR_CPUS];
static struct bheap      adgsnedf_cpu_heap;


//TODO-AARON: The number here limits the number of real time threads. 
//Need to make this more general or fix
//Should make this linked list to be more general and flexible
static struct task_struct* all_tasks[100]; 

//TODO-AARON: This is part of the hacky fix above used to keep track of the number
//of tasks in the system and add them to the task_struct. 
static int currentNumberTasks;

static double agsnedf_total_utilization;
static int taskSinceLastReweight; //Keeps track of number of task completions since
// last rewieght
static int removeMeTaskCounter; //Temporary, for building. This is used
//to keep track of the number of tasks that have enacted a reweighting event
static int changeNow; //if 1, under utilized. if 0, nothing. -1 if over utilized


static rt_domain_t adgsnedf;
#define adgsnedf_lock (adgsnedf.ready_lock)


/* Uncomment this if you want to see all scheduling decisions in the
 * TRACE() log.
#define WANT_ALL_SCHED_EVENTS
 */

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
 *                       order in the cpu queue. Caller must hold adgsnedf lock.
 */
static void update_cpu_position(cpu_entry_t *entry)
{
	if (likely(bheap_node_in_heap(entry->hn)))
		bheap_delete(cpu_lower_prio, &adgsnedf_cpu_heap, entry->hn);
	bheap_insert(cpu_lower_prio, &adgsnedf_cpu_heap, entry->hn);
}

/* caller must hold adgsnedf lock */
static cpu_entry_t* lowest_prio_cpu(void)
{
	struct bheap_node* hn;
	hn = bheap_peek(cpu_lower_prio, &adgsnedf_cpu_heap);
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
			sched = &per_cpu(adgsnedf_cpu_entries, on_cpu);
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
 *          where it was linked before. Must hold adgsnedf_lock.
 */
static noinline void unlink(struct task_struct* t)
{
	cpu_entry_t *entry;

	if (t->rt_param.linked_on != NO_CPU) {
		/* unlink */
		entry = &per_cpu(adgsnedf_cpu_entries, t->rt_param.linked_on);
		t->rt_param.linked_on = NO_CPU;
		link_task_to_cpu(NULL, entry);
	} else if (is_queued(t)) {
		/* This is an interesting situation: t is scheduled,
		 * but was just recently unlinked.  It cannot be
		 * linked anywhere else (because then it would have
		 * been relinked to this CPU), thus it must be in some
		 * queue. We must remove it from the list in this
		 * case.
		 */
		remove(&adgsnedf, t);
	}
}


/* preempt - force a CPU to reschedule
 */
static void preempt(cpu_entry_t *entry)
{
	preempt_if_preemptable(entry->scheduled, entry->cpu);
}

/* requeue - Put an unlinked task into adgsn-edf domain.
 *           Caller must hold adgsnedf_lock.
 */
static noinline void requeue(struct task_struct* task)
{
	BUG_ON(!task);
	/* sanity check before insertion */
	BUG_ON(is_queued(task));

	if (is_early_releasing(task) || is_released(task, litmus_clock()))
		__add_ready(&adgsnedf, task);
	else {
		/* it has got to wait */
		add_release(&adgsnedf, task);
	}
}

#ifdef CONFIG_SCHED_CPU_AFFINITY
static cpu_entry_t* adgsnedf_get_nearest_available_cpu(cpu_entry_t *start)
{
	cpu_entry_t *affinity;

	get_nearest_available_cpu(affinity, start, adgsnedf_cpu_entries,
#ifdef CONFIG_RELEASE_MASTER
			adgsnedf.release_master
#else
			NO_CPU
#endif
			);

	return(affinity);
}
#endif

/* check for any necessary preemptions */
static void check_for_preemptions(void)
{
	struct task_struct *task;
	cpu_entry_t *last;


#ifdef CONFIG_PREFER_LOCAL_LINKING
	cpu_entry_t *local;

	/* Before linking to other CPUs, check first whether the local CPU is
	 * idle. */
	local = &__get_cpu_var(adgsnedf_cpu_entries);
	task  = __peek_ready(&adgsnedf);

	if (task && !local->linked
#ifdef CONFIG_RELEASE_MASTER
	    && likely(local->cpu != adgsnedf.release_master)
#endif
		) {
		task = __take_ready(&adgsnedf);
		TRACE_TASK(task, "linking to local CPU %d to avoid IPI\n", local->cpu);
		link_task_to_cpu(task, local);
		preempt(local);
	}
#endif

	for (last = lowest_prio_cpu();
	     edf_preemption_needed(&adgsnedf, last->linked);
	     last = lowest_prio_cpu()) {
		/* preemption necessary */
		task = __take_ready(&adgsnedf);
		TRACE("check_for_preemptions: attempting to link task %d to %d\n",
		      task->pid, last->cpu);

#ifdef CONFIG_SCHED_CPU_AFFINITY
		{
			cpu_entry_t *affinity =
					adgsnedf_get_nearest_available_cpu(
						&per_cpu(adgsnedf_cpu_entries, task_cpu(task)));
			if (affinity)
				last = affinity;
			else if (requeue_preempted_job(last->linked))
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

/* adgsnedf_job_arrival: task is either resumed or released */
static noinline void adgsnedf_job_arrival(struct task_struct* task)
{
	BUG_ON(!task);

	requeue(task);
	check_for_preemptions();
}

static void adgsnedf_release_jobs(rt_domain_t* rt, struct bheap* tasks)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&adgsnedf_lock, flags);

	__merge_ready(rt, tasks);
	check_for_preemptions();

	raw_spin_unlock_irqrestore(&adgsnedf_lock, flags);
}

/* 	p and i values are the weights that should be given to the proportional and
 *	integrative components for calculating a new estimated execution time 
 * 	NOTE: Must be called before execution_cost is reset*/
static void calculate_estimated_execution_cost(struct task_struct *t, double p, double i){
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
 
  
//Should only be called by job_completion. Thus, caller holds adgnedf_lock
static noinline void adjust_all_service_levels(int triggerNow){
	int count;
	struct task_struct *temp;
	//TODO: Adjust all the service levels of all the jobs if a trigger threshold is met.
	// This is how tsk_rt(t)->ctrl_page->service_level;
	const int number_of_cpus_held_back = 2;
	struct task_struct* local_copy[currentNumberTasks];
	int taskLevel[currentNumberTasks];
	double weightLevel[currentNumberTasks];
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
	
	int aConvertValue, bConvertValue,cConvertValue;
	double localTotalUtilization = 0;
	double maxUtilization = num_online_cpus()-number_of_cpus_held_back;
	//This is the max_level 
	const int max_level = 2; //max level should be 3, but crashing. So let's try 2
	const int lowest_level = 0; //lowest service level level should be 3, but crashing. So let's try 2
	
	//
	double calculationFactor;
	
	taskSinceLastReweight++; //Keep track of the number of tasks that have occured since
	//last rewieghting
	
	if( (triggerNow!=0) || (taskSinceLastReweight > 300) ) { //TODO: Pick a better number here
		taskSinceLastReweight = 0;
		//Copying array to local copy
		for(count = 0;count < currentNumberTasks; count++) {
			local_copy[count] = all_tasks[count];
		}
	

		//sort local_copy based on value density.. Assumes linear relationship
		// The rather complicated formulas in here come from Aaron Block's dissertation
		// specifically formula 6.11 on page 258
		// you can get Aaron's dissertation here: http://cs.unc.edu/~block/aarondiss.pdf
		
		for(outerIndex = 1; outerIndex <= currentNumberTasks - 1 ; outerIndex++) {
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
		for(lowerIndex =0; lowerIndex< currentNumberTasks;lowerIndex++){
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
		
		for(outerIndex=0; outerIndex < currentNumberTasks; outerIndex++) {
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
		
		for(outerIndex=0; outerIndex < currentNumberTasks; outerIndex++) {

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
		for(outerIndex=0; outerIndex < currentNumberTasks; outerIndex++) {
			tsk_rt(local_copy[outerIndex])->ctrl_page->service_level = taskLevel[outerIndex];
			TRACE("&&The service level for %d is %d\n", outerIndex, taskLevel[outerIndex]);
		}
	}	
}



 
 
 

/* caller holds adgsnedf_lock */
static noinline void job_completion(struct task_struct *t, int forced)
{
	//TODO: Change this if I want to trigger a weight change when we have fewer than 2 CPUS left
	const int bufferCPUs = 2 ;
	double old_est_weight;
	double difference_in_weight;
	int triggerNow = 0;
	double maxUtilization;
	BUG_ON(!t);

	sched_trace_task_completion(t, forced);

	TRACE_TASK(t, "job_completion().\n");

	/* set flags */
	tsk_rt(t)->completed = 0;
	
	old_est_weight = get_estimated_weight(t);
 
	//TODO: replace the (0.10206228,1) in next line with user-set p and i values
	/* The values 0.102 and 0.30345 are the a and b values that are calculated from
	 * Aaron Block's dissertation referenced on pages 293 (the experimental values for
	 * a and c) and the relationship of a,b,c is given on page 253 just below (6.2)
	 */
	calculate_estimated_execution_cost(t,0.102,0.30345);
	
	t->rt_param.job_params.estimated_weight = 
		((double) get_estimated_exec_time(t))/get_rt_relative_deadline(t);
	
	//TODO: Make a flag for the scheduler base on whether service levels are adjusted
	//based on time or trigger level. 
	
	/* The change in the estimated_weight of this job must be added to the total 
	 * utilization
	 */
 
	difference_in_weight = get_estimated_weight(t) - old_est_weight;

	agsnedf_total_utilization += difference_in_weight;
	
	triggerNow = 0;
	
	maxUtilization = num_online_cpus()-bufferCPUs;
	TRACE("The utilization times 100 is %d\n", (int)(maxUtilization*100));
	if( agsnedf_total_utilization > maxUtilization) {
		triggerNow = 1;
	}
	adjust_all_service_levels(triggerNow);
	
	// **** TESTING ******
	// AARON; Note, I want to make sure that the service level will get data sent back and forth
	//TRACE("TASK SERVICE LEVEL :%u\n",tsk_rt(t)->ctrl_page->service_level);
	
	//Constantly rotate between service levels
/*	
	if ( changeNow>0 ) {
		if (tsk_rt(t)->ctrl_page->service_level < 2) {
			tsk_rt(t)->ctrl_page->service_level+=1;
			//Change period
// 			tsk_rt(t)->task_params.period = tsk_rt(t)->task_params.service_levels[tsk_rt(t)->ctrl_page->service_level].service_level_period;
// 			tsk_rt(t)->task_params.relative_deadline = tsk_rt(t)->task_params.period;
		}
		removeMeTaskCounter++;
	} else if ( changeNow < 0 ) {
		if (tsk_rt(t)->ctrl_page->service_level > 0) {
			tsk_rt(t)->ctrl_page->service_level -= 1;
			//Change period
// 			tsk_rt(t)->task_params.period = tsk_rt(t)->task_params.service_levels[tsk_rt(t)->ctrl_page->service_level].service_level_period;
// 			tsk_rt(t)->task_params.relative_deadline = tsk_rt(t)->task_params.period;
		}
		removeMeTaskCounter++;
	} 
	// there are 32 tasks, so, once all 32 have gone. No more changing
	if (removeMeTaskCounter >=32) {
		removeMeTaskCounter = 0;
		changeNow = 0;
	}
*/
	
	//This line is also incorect. No idea why it would let me use it
	//t->rt_param.job_params.current_service_level+=30;
	TRACE("***TASK SERVICE LEVEL :%u\n",tsk_rt(t)->ctrl_page->service_level);
	TRACE("A-GSN-EDF-Period: %llu\n", tsk_rt(t)->task_params.period);
	TRACE("A-GSN-EDF-Deadline: %llu\n", tsk_rt(t)->task_params.relative_deadline);
	
	/* prepare for next period */
	prepare_for_next_period(t);
	TRACE("ADGSN-EDF : %d\n", tsk_rt(t)->ctrl_page->service_level);
	if (is_early_releasing(t) || is_released(t, litmus_clock()))
		sched_trace_task_release(t);
	/* unlink */
	unlink(t);
	/* requeue
	 * But don't requeue a blocking task. */
	if (is_running(t))
		adgsnedf_job_arrival(t);
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
static struct task_struct* adgsnedf_schedule(struct task_struct * prev)
{
	cpu_entry_t* entry = &__get_cpu_var(adgsnedf_cpu_entries);
	int out_of_time, sleep, preempt, np, exists, blocks;
	struct task_struct* next = NULL;

#ifdef CONFIG_RELEASE_MASTER
	/* Bail out early if we are the release master.
	 * The release master never schedules any real-time tasks.
	 */
	if (unlikely(adgsnedf.release_master == entry->cpu)) {
		sched_state_task_picked();
		return NULL;
	}
#endif

	raw_spin_lock(&adgsnedf_lock);

	/* sanity checking */
	BUG_ON(entry->scheduled && entry->scheduled != prev);
	BUG_ON(entry->scheduled && !is_realtime(prev));
	BUG_ON(is_realtime(prev) && !entry->scheduled);

	/* (0) Determine state */
	exists      = entry->scheduled != NULL;
	blocks      = exists && !is_running(entry->scheduled);
	out_of_time = exists && budget_enforced(entry->scheduled)
		&& budget_exhausted(entry->scheduled);
	np 	    = exists && is_np(entry->scheduled);
	sleep	    = exists && is_completed(entry->scheduled);
	preempt     = entry->scheduled != entry->linked;

#ifdef WANT_ALL_SCHED_EVENTS
	TRACE_TASK(prev, "invoked adgsnedf_schedule.\n");
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
		link_task_to_cpu(__take_ready(&adgsnedf), entry);

	/* The final scheduling decision. Do we need to switch for some reason?
	 * If linked is different from scheduled, then select linked as next.
	 */
	if ((!np || blocks) &&
	    entry->linked != entry->scheduled) {
		/* Schedule a linked job? */
		if (entry->linked) {
			entry->linked->rt_param.scheduled_on = entry->cpu;
			next = entry->linked;
			TRACE_TASK(next, "scheduled_on = P%d\n", smp_processor_id());
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

	raw_spin_unlock(&adgsnedf_lock);

#ifdef WANT_ALL_SCHED_EVENTS
	TRACE("adgsnedf_lock released, next=0x%p\n", next);

	if (next)
		TRACE_TASK(next, "scheduled at %llu\n", litmus_clock());
	else if (exists && !next)
		TRACE("becomes idle at %llu.\n", litmus_clock());
#endif


	return next;
}


/* _finish_switch - we just finished the switch away from prev
 */
static void adgsnedf_finish_switch(struct task_struct *prev)
{
	cpu_entry_t* 	entry = &__get_cpu_var(adgsnedf_cpu_entries);

	entry->scheduled = is_realtime(current) ? current : NULL;
#ifdef WANT_ALL_SCHED_EVENTS
	TRACE_TASK(prev, "switched away from\n");
#endif
}


/*	Prepare a task for running in RT mode
 */
static void adgsnedf_task_new(struct task_struct * t, int on_rq, int is_scheduled)
{
	unsigned long 		flags;
	cpu_entry_t* 		entry;
	int localNumber; 



	TRACE("adgsn edf: task new %d\n", t->pid);

	raw_spin_lock_irqsave(&adgsnedf_lock, flags);

	localNumber = currentNumberTasks; 
	all_tasks[currentNumberTasks] = t;
	currentNumberTasks++;


	/* TODO: allow for better setting of estimated execution
	 * time.
	 */
	t->rt_param.job_params.estimated_exec_time = 0;
	
	/* setup job params */
	release_at(t, litmus_clock());

	if (is_scheduled) {
		entry = &per_cpu(adgsnedf_cpu_entries, task_cpu(t));
		BUG_ON(entry->scheduled);

#ifdef CONFIG_RELEASE_MASTER
		if (entry->cpu != adgsnedf.release_master) {
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

	if (is_running(t))
		adgsnedf_job_arrival(t);
	raw_spin_unlock_irqrestore(&adgsnedf_lock, flags);
	TRACE("Adding task number %d\n", localNumber);
}

static void adgsnedf_task_wake_up(struct task_struct *task)
{
	unsigned long flags;
	lt_t now;

	TRACE_TASK(task, "wake_up at %llu\n", litmus_clock());

	raw_spin_lock_irqsave(&adgsnedf_lock, flags);
	now = litmus_clock();
	if (is_sporadic(task) && is_tardy(task, now)) {
		/* new sporadic release */
		release_at(task, now);
		sched_trace_task_release(task);
	}
	adgsnedf_job_arrival(task);
	raw_spin_unlock_irqrestore(&adgsnedf_lock, flags);
}

static void adgsnedf_task_block(struct task_struct *t)
{
	unsigned long flags;

	TRACE_TASK(t, "block at %llu\n", litmus_clock());

	/* unlink if necessary */
	raw_spin_lock_irqsave(&adgsnedf_lock, flags);
	unlink(t);
	raw_spin_unlock_irqrestore(&adgsnedf_lock, flags);

	BUG_ON(!is_realtime(t));
}


static void adgsnedf_task_exit(struct task_struct * t)
{
	unsigned long flags;

	/* unlink if necessary */
	raw_spin_lock_irqsave(&adgsnedf_lock, flags);
	
	/* When a task exits, we need to remove it's weight from the total utilization
	 * There is a possibility that the estimated weight may drift from the total util
	 * in this case, just set the total utilization to 0. */
	if (agsnedf_total_utilization<get_estimated_weight(t)){
		agsnedf_total_utilization=0;
	} else {
		agsnedf_total_utilization-=get_estimated_weight(t);
	}
	
	unlink(t);
	if (tsk_rt(t)->scheduled_on != NO_CPU) {
		adgsnedf_cpus[tsk_rt(t)->scheduled_on]->scheduled = NULL;
		tsk_rt(t)->scheduled_on = NO_CPU;
	}
	raw_spin_unlock_irqrestore(&adgsnedf_lock, flags);

	BUG_ON(!is_realtime(t));
        TRACE_TASK(t, "RIP\n");
}


static long adgsnedf_admit_task(struct task_struct* tsk)
{
	return 0;
}

#ifdef CONFIG_LITMUS_LOCKING

#include <litmus/fdso.h>

/* called with IRQs off */
static void set_priority_inheritance(struct task_struct* t, struct task_struct* prio_inh)
{
	int linked_on;
	int check_preempt = 0;

	raw_spin_lock(&adgsnedf_lock);

	TRACE_TASK(t, "inherits priority from %s/%d\n", prio_inh->comm, prio_inh->pid);
	tsk_rt(t)->inh_task = prio_inh;

	linked_on  = tsk_rt(t)->linked_on;

	/* If it is scheduled, then we need to reorder the CPU heap. */
	if (linked_on != NO_CPU) {
		TRACE_TASK(t, "%s: linked  on %d\n",
			   __FUNCTION__, linked_on);
		/* Holder is scheduled; need to re-order CPUs.
		 * We can't use heap_decrease() here since
		 * the cpu_heap is ordered in reverse direction, so
		 * it is actually an increase. */
		bheap_delete(cpu_lower_prio, &adgsnedf_cpu_heap,
			    adgsnedf_cpus[linked_on]->hn);
		bheap_insert(cpu_lower_prio, &adgsnedf_cpu_heap,
			    adgsnedf_cpus[linked_on]->hn);
	} else {
		/* holder may be queued: first stop queue changes */
		raw_spin_lock(&adgsnedf.release_lock);
		if (is_queued(t)) {
			TRACE_TASK(t, "%s: is queued\n",
				   __FUNCTION__);
			/* We need to update the position of holder in some
			 * heap. Note that this could be a release heap if we
			 * budget enforcement is used and this job overran. */
			check_preempt =
				!bheap_decrease(edf_ready_order,
					       tsk_rt(t)->heap_node);
		} else {
			/* Nothing to do: if it is not queued and not linked
			 * then it is either sleeping or currently being moved
			 * by other code (e.g., a timer interrupt handler) that
			 * will use the correct priority when enqueuing the
			 * task. */
			TRACE_TASK(t, "%s: is NOT queued => Done.\n",
				   __FUNCTION__);
		}
		raw_spin_unlock(&adgsnedf.release_lock);

		/* If holder was enqueued in a release heap, then the following
		 * preemption check is pointless, but we can't easily detect
		 * that case. If you want to fix this, then consider that
		 * simply adding a state flag requires O(n) time to update when
		 * releasing n tasks, which conflicts with the goal to have
		 * O(log n) merges. */
		if (check_preempt) {
			/* heap_decrease() hit the top level of the heap: make
			 * sure preemption checks get the right task, not the
			 * potentially stale cache. */
			bheap_uncache_min(edf_ready_order,
					 &adgsnedf.ready_queue);
			check_for_preemptions();
		}
	}

	raw_spin_unlock(&adgsnedf_lock);
}

/* called with IRQs off */
static void clear_priority_inheritance(struct task_struct* t)
{
	raw_spin_lock(&adgsnedf_lock);

	/* A job only stops inheriting a priority when it releases a
	 * resource. Thus we can make the following assumption.*/
	BUG_ON(tsk_rt(t)->scheduled_on == NO_CPU);

	TRACE_TASK(t, "priority restored\n");
	tsk_rt(t)->inh_task = NULL;

	/* Check if rescheduling is necessary. We can't use heap_decrease()
	 * since the priority was effectively lowered. */
	unlink(t);
	adgsnedf_job_arrival(t);

	raw_spin_unlock(&adgsnedf_lock);
}


/* ******************** FMLP support ********************** */

/* struct for semaphore with priority inheritance */
struct fmlp_semaphore {
	struct litmus_lock litmus_lock;

	/* current resource holder */
	struct task_struct *owner;

	/* highest-priority waiter */
	struct task_struct *hp_waiter;

	/* FIFO queue of waiting tasks */
	wait_queue_head_t wait;
};

static inline struct fmlp_semaphore* fmlp_from_lock(struct litmus_lock* lock)
{
	return container_of(lock, struct fmlp_semaphore, litmus_lock);
}

/* caller is responsible for locking */
struct task_struct* find_hp_waiter_adgsn_edf(struct fmlp_semaphore *sem,
				   struct task_struct* skip)
{
	struct list_head	*pos;
	struct task_struct 	*queued, *found = NULL;

	list_for_each(pos, &sem->wait.task_list) {
		queued  = (struct task_struct*) list_entry(pos, wait_queue_t,
							   task_list)->private;

		/* Compare task prios, find high prio task. */
		if (queued != skip && edf_higher_prio(queued, found))
			found = queued;
	}
	return found;
}

int adgsnedf_fmlp_lock(struct litmus_lock* l)
{
	struct task_struct* t = current;
	struct fmlp_semaphore *sem = fmlp_from_lock(l);
	wait_queue_t wait;
	unsigned long flags;

	if (!is_realtime(t))
		return -EPERM;

	/* prevent nested lock acquisition --- not supported by FMLP */
	if (tsk_rt(t)->num_locks_held)
		return -EBUSY;

	spin_lock_irqsave(&sem->wait.lock, flags);

	if (sem->owner) {
		/* resource is not free => must suspend and wait */

		init_waitqueue_entry(&wait, t);

		/* FIXME: interruptible would be nice some day */
		set_task_state(t, TASK_UNINTERRUPTIBLE);

		__add_wait_queue_tail_exclusive(&sem->wait, &wait);

		/* check if we need to activate priority inheritance */
		if (edf_higher_prio(t, sem->hp_waiter)) {
			sem->hp_waiter = t;
			if (edf_higher_prio(t, sem->owner))
				set_priority_inheritance(sem->owner, sem->hp_waiter);
		}

		TS_LOCK_SUSPEND;

		/* release lock before sleeping */
		spin_unlock_irqrestore(&sem->wait.lock, flags);

		/* We depend on the FIFO order.  Thus, we don't need to recheck
		 * when we wake up; we are guaranteed to have the lock since
		 * there is only one wake up per release.
		 */

		schedule();

		TS_LOCK_RESUME;

		/* Since we hold the lock, no other task will change
		 * ->owner. We can thus check it without acquiring the spin
		 * lock. */
		BUG_ON(sem->owner != t);
	} else {
		/* it's ours now */
		sem->owner = t;

		spin_unlock_irqrestore(&sem->wait.lock, flags);
	}

	tsk_rt(t)->num_locks_held++;

	return 0;
}

int adgsnedf_fmlp_unlock(struct litmus_lock* l)
{
	struct task_struct *t = current, *next;
	struct fmlp_semaphore *sem = fmlp_from_lock(l);
	unsigned long flags;
	int err = 0;

	spin_lock_irqsave(&sem->wait.lock, flags);

	if (sem->owner != t) {
		err = -EINVAL;
		goto out;
	}

	tsk_rt(t)->num_locks_held--;

	/* check if there are jobs waiting for this resource */
	next = __waitqueue_remove_first(&sem->wait);
	if (next) {
		/* next becomes the resouce holder */
		sem->owner = next;
		TRACE_CUR("lock ownership passed to %s/%d\n", next->comm, next->pid);

		/* determine new hp_waiter if necessary */
		if (next == sem->hp_waiter) {
			TRACE_TASK(next, "was highest-prio waiter\n");
			/* next has the highest priority --- it doesn't need to
			 * inherit.  However, we need to make sure that the
			 * next-highest priority in the queue is reflected in
			 * hp_waiter. */
			sem->hp_waiter = find_hp_waiter_adgsn_edf(sem, next);
			if (sem->hp_waiter)
				TRACE_TASK(sem->hp_waiter, "is new highest-prio waiter\n");
			else
				TRACE("no further waiters\n");
		} else {
			/* Well, if next is not the highest-priority waiter,
			 * then it ought to inherit the highest-priority
			 * waiter's priority. */
			set_priority_inheritance(next, sem->hp_waiter);
		}

		/* wake up next */
		wake_up_process(next);
	} else
		/* becomes available */
		sem->owner = NULL;

	/* we lose the benefit of priority inheritance (if any) */
	if (tsk_rt(t)->inh_task)
		clear_priority_inheritance(t);

out:
	spin_unlock_irqrestore(&sem->wait.lock, flags);

	return err;
}

int adgsnedf_fmlp_close(struct litmus_lock* l)
{
	struct task_struct *t = current;
	struct fmlp_semaphore *sem = fmlp_from_lock(l);
	unsigned long flags;

	int owner;

	spin_lock_irqsave(&sem->wait.lock, flags);

	owner = sem->owner == t;

	spin_unlock_irqrestore(&sem->wait.lock, flags);

	if (owner)
		adgsnedf_fmlp_unlock(l);

	return 0;
}

void adgsnedf_fmlp_free(struct litmus_lock* lock)
{
	kfree(fmlp_from_lock(lock));
}

static struct litmus_lock_ops adgsnedf_fmlp_lock_ops = {
	.close  = adgsnedf_fmlp_close,
	.lock   = adgsnedf_fmlp_lock,
	.unlock = adgsnedf_fmlp_unlock,
	.deallocate = adgsnedf_fmlp_free,
};

static struct litmus_lock* adgsnedf_new_fmlp(void)
{
	struct fmlp_semaphore* sem;

	sem = kmalloc(sizeof(*sem), GFP_KERNEL);
	if (!sem)
		return NULL;

	sem->owner   = NULL;
	sem->hp_waiter = NULL;
	init_waitqueue_head(&sem->wait);
	sem->litmus_lock.ops = &adgsnedf_fmlp_lock_ops;

	return &sem->litmus_lock;
}

/* **** lock constructor **** */


static long adgsnedf_allocate_lock(struct litmus_lock **lock, int type,
				 void* __user unused)
{
	int err = -ENXIO;

	/* adgsn-EDF currently only supports the FMLP for global resources. */
	switch (type) {

	case FMLP_SEM:
		/* Flexible Multiprocessor Locking Protocol */
		*lock = adgsnedf_new_fmlp();
		if (*lock)
			err = 0;
		else
			err = -ENOMEM;
		break;

	};

	return err;
}

#endif

static struct domain_proc_info adgsnedf_domain_proc_info;
static long adgsnedf_get_domain_proc_info(struct domain_proc_info **ret)
{
	*ret = &adgsnedf_domain_proc_info;
	return 0;
}

static void adgsnedf_setup_domain_proc(void)
{
	int i, cpu;
	int release_master =
#ifdef CONFIG_RELEASE_MASTER
			atomic_read(&release_master_cpu);
#else
		NO_CPU;
#endif
	int num_rt_cpus = num_online_cpus() - (release_master != NO_CPU);
	struct cd_mapping *map;

	memset(&adgsnedf_domain_proc_info, sizeof(adgsnedf_domain_proc_info), 0);
	init_domain_proc_info(&adgsnedf_domain_proc_info, num_rt_cpus, 1);
	adgsnedf_domain_proc_info.num_cpus = num_rt_cpus;
	adgsnedf_domain_proc_info.num_domains = 1;

	adgsnedf_domain_proc_info.domain_to_cpus[0].id = 0;
	for (cpu = 0, i = 0; cpu < num_online_cpus(); ++cpu) {
		if (cpu == release_master)
			continue;
		map = &adgsnedf_domain_proc_info.cpu_to_domains[i];
		map->id = cpu;
		cpumask_set_cpu(0, map->mask);
		++i;

		/* add cpu to the domain */
		cpumask_set_cpu(cpu,
			adgsnedf_domain_proc_info.domain_to_cpus[0].mask);
	}
}

static long adgsnedf_activate_plugin(void)
{
	int cpu;
	cpu_entry_t *entry;
	
	agsnedf_total_utilization = 0;
	taskSinceLastReweight = 0;
	removeMeTaskCounter=0; //TODO: REMOVE Later
	changeNow=0; 
	currentNumberTasks = 0;


	bheap_init(&adgsnedf_cpu_heap);
#ifdef CONFIG_RELEASE_MASTER
	adgsnedf.release_master = atomic_read(&release_master_cpu);
#endif

	for_each_online_cpu(cpu) {
		entry = &per_cpu(adgsnedf_cpu_entries, cpu);
		bheap_node_init(&entry->hn, entry);
		entry->linked    = NULL;
		entry->scheduled = NULL;
#ifdef CONFIG_RELEASE_MASTER
		if (cpu != adgsnedf.release_master) {
#endif
			TRACE("adgsn-EDF: Initializing CPU #%d.\n", cpu);
			update_cpu_position(entry);
#ifdef CONFIG_RELEASE_MASTER
		} else {
			TRACE("adgsn-EDF: CPU %d is release master.\n", cpu);
		}
#endif
	}

	adgsnedf_setup_domain_proc();
	return 0;
}

static long adgsnedf_deactivate_plugin(void)
{
	destroy_domain_proc_info(&adgsnedf_domain_proc_info);
	return 0;
}

/*	Plugin object	*/
static struct sched_plugin adgsn_edf_plugin __cacheline_aligned_in_smp = {
	.plugin_name		= "AD-adgsn-EDF",
	.finish_switch		= adgsnedf_finish_switch,
	.task_new		= adgsnedf_task_new,
	.complete_job		= complete_job,
	.task_exit		= adgsnedf_task_exit,
	.schedule		= adgsnedf_schedule,
	.task_wake_up		= adgsnedf_task_wake_up,
	.task_block		= adgsnedf_task_block,
	.admit_task		= adgsnedf_admit_task,
	.activate_plugin	= adgsnedf_activate_plugin,
	.deactivate_plugin	= adgsnedf_deactivate_plugin,
	.get_domain_proc_info	= adgsnedf_get_domain_proc_info,
#ifdef CONFIG_LITMUS_LOCKING
	.allocate_lock		= adgsnedf_allocate_lock,
#endif
};


static int __init init_adgsn_edf(void)
{
	int cpu;
	cpu_entry_t *entry;

	bheap_init(&adgsnedf_cpu_heap);
	/* initialize CPU state */
	for (cpu = 0; cpu < NR_CPUS; cpu++)  {
		entry = &per_cpu(adgsnedf_cpu_entries, cpu);
		adgsnedf_cpus[cpu] = entry;
		entry->cpu 	 = cpu;
		entry->hn        = &adgsnedf_heap_node[cpu];
		bheap_node_init(&entry->hn, entry);
	}
	edf_domain_init(&adgsnedf, NULL, adgsnedf_release_jobs);
	return register_sched_plugin(&adgsn_edf_plugin);
}


module_init(init_adgsn_edf);
