/* litmus/jobs.c - common job control code
 */

#include <linux/sched.h>

#include <litmus/litmus.h>
#include <litmus/jobs.h>

static inline void setup_release(struct task_struct *t, lt_t release)
{
	int relative_work;
	int control1;
	int control2;
	int serviceLevel = tsk_rt(t)->ctrl_page->service_level;
	/* prepare next release */
	t->rt_param.job_params.release = release;
		
	/* If the task has multiple service levels (i.e., adaptive), then the deadline is 
	 * determined  by the current service level. 
	 */
	if (get_service_levels(t))  {
		struct rt_service_level current_service_level = 
			get_service_levels(t)[serviceLevel];
		//tsk_rt(t)->job_params.current_service_level
		//tsk_rt(t)->ctrl_page->service_level
		lt_t relative_deadline = current_service_level.service_level_period;
		//TODO: Remove printing
		relative_work = (int)(1000*current_service_level.relative_work);
		control1 = serviceLevel;
		control2 = current_service_level.service_level_number;
		TRACE_TASK(t, "service level %d, level's service: %d,  relative work %d\n", control1, control2, relative_work);
		TRACE_TASK(t, "Changing relative_deadline from %llu to %llu. Other %llu\n",t->rt_param.task_params.relative_deadline, relative_deadline, get_rt_relative_deadline(t));
		TRACE("Setup_release 2 : %d\n", tsk_rt(t)->ctrl_page->service_level);
		t->rt_param.task_params.relative_deadline = relative_deadline;
		t->rt_param.task_params.period = relative_deadline;
		
		/* There are two "current service levels" on in job_params on in the 
		 * task's control_page. The control_page struct is the one the user-space
		 * task is aware of. The job_params is used to denote the service level
		 * the next job should be released at. We update the control_page one here
		 */
	
		//TODO: see if this two pronged service level is working
		//If it is't cut it
		//tsk_rt(t)->ctrl_page->service_level = get_current_survice_level(t);
		TRACE("Setup_release 3 : %d\n", tsk_rt(t)->ctrl_page->service_level);
	} else {
		TRACE("No service levels defined\n");
	}
	TRACE("Setup_release 4 : %d\n", tsk_rt(t)->ctrl_page->service_level);
	t->rt_param.job_params.deadline = release + get_rt_relative_deadline(t);
// 	TRACE_TASK(t, "Relative Deadline %llu\n",
// 			   get_rt_relative_deadline(t));
// 	TRACE_TASK(t, "Absolute Deadline %llu\n",
// 			   t->rt_param.job_params.deadline);
	t->rt_param.job_params.exec_time = 0;

	TRACE("Setup_release 5 : %d\n", tsk_rt(t)->ctrl_page->service_level);
	/* update job sequence number */
	t->rt_param.job_params.job_no++;
}

void prepare_for_next_period(struct task_struct *t)
{
	BUG_ON(!t);

	/* Record lateness before we set up the next job's
	 * release and deadline. Lateness may be negative.
	 */
	t->rt_param.job_params.lateness =
		(long long)litmus_clock() -
		(long long)t->rt_param.job_params.deadline;

	if (tsk_rt(t)->sporadic_release) {
		TRACE_TASK(t, "sporadic release at %llu\n",
			   tsk_rt(t)->sporadic_release_time);
		/* sporadic release */
		setup_release(t, tsk_rt(t)->sporadic_release_time);
		tsk_rt(t)->sporadic_release = 0;
	} else {
		/* periodic release => add period */
		setup_release(t, get_release(t) + get_rt_period(t));
	}
}

void release_at(struct task_struct *t, lt_t start)
{
	BUG_ON(!t);
	setup_release(t, start);
	tsk_rt(t)->completed = 0;
}

long default_wait_for_release_at(lt_t release_time)
{
	struct task_struct *t = current;
	unsigned long flags;

	local_irq_save(flags);
	tsk_rt(t)->sporadic_release_time = release_time;
	smp_wmb();
	tsk_rt(t)->sporadic_release = 1;
	local_irq_restore(flags);

	return complete_job();
}


/*
 *	Deactivate current task until the beginning of the next period.
 */
long complete_job(void)
{
	/* Mark that we do not excute anymore */
	tsk_rt(current)->completed = 1;
	/* call schedule, this will return when a new job arrives
	 * it also takes care of preparing for the next release
	 */
	schedule();
	return 0;
}
