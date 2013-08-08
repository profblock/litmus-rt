/* litmus/jobs.c - common job control code
 */

#include <linux/sched.h>

#include <litmus/litmus.h>
#include <litmus/jobs.h>

static inline void setup_release(struct task_struct *t, lt_t release)
{
	/* prepare next release */
	t->rt_param.job_params.release = release;
	/* If the task has multiple service levels (i.e., adaptive), then the deadline is 
	 * determined  by the current service level. 
	 */
	if (get_service_levels(t))  {
		struct rt_service_level current_service_level = 
			get_service_levels(t)[get_current_survice_level(t)];
		lt_t relative_deadline = current_service_level.service_level_period;
		
		t->rt_param.job_params.deadline = release + relative_deadline;
		
		t->rt_param.job_params.estimated_weight = 
			((double) get_estimated_exec_time(t))/relative_deadline;
		
		/* There are two "current service levels" on in job_params on in the 
		 * task's control_page. The control_page struct is the one the user-space
		 * task is aware of. The job_params is used to denote the service level
		 * the next job should be released at. We update the control_page one here
		 */
		 
		 tsk_rt(t)->ctrl_page->service_level = get_current_survice_level(t);
		 
		
	} else {
		t->rt_param.job_params.deadline = release + get_rt_relative_deadline(t);
	}
	t->rt_param.job_params.exec_time = 0;
	

	/* update job sequence number */
	t->rt_param.job_params.job_no++;

	/* don't confuse Linux */
	t->rt.time_slice = 1;
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
		
	setup_release(t, get_release(t) + get_rt_period(t));
}

void release_at(struct task_struct *t, lt_t start)
{
	BUG_ON(!t);
	setup_release(t, start);
	tsk_rt(t)->completed = 0;
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
