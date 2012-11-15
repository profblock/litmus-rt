/*
 * LITMUS^RT kernel style scheduling tracepoints
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM litmus

#if !defined(_SCHED_TASK_TRACEPOINT_H) || defined(TRACE_HEADER_MULTI_READ)
#define _SCHED_TASK_TRACEPOINT_H

#include <linux/tracepoint.h>

#include <litmus/litmus.h>
#include <litmus/rt_param.h>

/*
 * Tracing task admission
 */
TRACE_EVENT(litmus_task_param,

	TP_PROTO(struct task_struct *t),

	TP_ARGS(t),

	TP_STRUCT__entry(
		__field( pid_t,		pid	)
		__field( unsigned int,	job	)
		__field( lt_t,		wcet	)
		__field( lt_t,		period	)
		__field( lt_t,		phase	)
		__field( int,		partition )
	),

	TP_fast_assign(
		__entry->pid	= t ? t->pid : 0;
		__entry->job	= t ? t->rt_param.job_params.job_no : 0;
		__entry->wcet	= get_exec_cost(t);
		__entry->period	= get_rt_period(t);
		__entry->phase	= get_rt_phase(t);
		__entry->partition = get_partition(t);
	),

	TP_printk("period(%d, %Lu).\nwcet(%d, %Lu).\n",
		__entry->pid, __entry->period,
		__entry->pid, __entry->wcet)
);

/*
 * Tracing jobs release
 */
TRACE_EVENT(litmus_task_release,

	TP_PROTO(struct task_struct *t),

	TP_ARGS(t),

	TP_STRUCT__entry(
		__field( pid_t,		pid	)
		__field( unsigned int,	job	)
		__field( lt_t,		release	)
		__field( lt_t,		deadline	)
	),

	TP_fast_assign(
		__entry->pid	= t ? t->pid : 0;
		__entry->job	= t ? t->rt_param.job_params.job_no : 0;
		__entry->release	= get_release(t);
		__entry->deadline	= get_deadline(t);
	),

	TP_printk("release(job(%u, %u)): %Lu\ndeadline(job(%u, %u)): %Lu\n",
			__entry->pid, __entry->job, __entry->release,
			__entry->pid, __entry->job, __entry->deadline)
);

/*
 * Tracepoint for switching to new task
 */
TRACE_EVENT(litmus_switch_to,

	TP_PROTO(struct task_struct *t),

	TP_ARGS(t),

	TP_STRUCT__entry(
		__field( pid_t,		pid	)
		__field( unsigned int,	job	)
		__field( lt_t,		when	)
		__field( lt_t,		exec_time	)
	),

	TP_fast_assign(
		__entry->pid	= is_realtime(t) ? t->pid : 0;
		__entry->job	= is_realtime(t) ? t->rt_param.job_params.job_no : 0;
		__entry->when		= litmus_clock();
		__entry->exec_time	= get_exec_time(t);
	),

	TP_printk("switch_to(job(%u, %u)): %Lu (exec: %Lu)\n",
			__entry->pid, __entry->job,
			__entry->when, __entry->exec_time)
);

/*
 * Tracepoint for switching away previous task
 */
TRACE_EVENT(litmus_switch_away,

	TP_PROTO(struct task_struct *t),

	TP_ARGS(t),

	TP_STRUCT__entry(
		__field( pid_t,		pid	)
		__field( unsigned int,	job	)
		__field( lt_t,		when	)
		__field( lt_t,		exec_time	)
	),

	TP_fast_assign(
		__entry->pid	= is_realtime(t) ? t->pid : 0;
		__entry->job	= is_realtime(t) ? t->rt_param.job_params.job_no : 0;
		__entry->when		= litmus_clock();
		__entry->exec_time	= get_exec_time(t);
	),

	TP_printk("switch_away(job(%u, %u)): %Lu (exec: %Lu)\n",
			__entry->pid, __entry->job,
			__entry->when, __entry->exec_time)
);

/*
 * Tracing jobs completion
 */
TRACE_EVENT(litmus_task_completion,

	TP_PROTO(struct task_struct *t, unsigned long forced),

	TP_ARGS(t, forced),

	TP_STRUCT__entry(
		__field( pid_t,		pid	)
		__field( unsigned int,	job	)
		__field( lt_t,		when	)
		__field( unsigned long,	forced	)
	),

	TP_fast_assign(
		__entry->pid	= t ? t->pid : 0;
		__entry->job	= t ? t->rt_param.job_params.job_no : 0;
		__entry->when	= litmus_clock();
		__entry->forced	= forced;
	),

	TP_printk("completed(job(%u, %u)): %Lu (forced: %lu)\n",
			__entry->pid, __entry->job,
			__entry->when, __entry->forced)
);

/*
 * Trace blocking tasks.
 */
TRACE_EVENT(litmus_task_block,

	TP_PROTO(struct task_struct *t),

	TP_ARGS(t),

	TP_STRUCT__entry(
		__field( pid_t,		pid	)
		__field( lt_t,		when	)
	),

	TP_fast_assign(
		__entry->pid	= t ? t->pid : 0;
		__entry->when	= litmus_clock();
	),

	TP_printk("(%u) blocks: %Lu\n", __entry->pid, __entry->when)
);

/*
 * Tracing jobs resume
 */
TRACE_EVENT(litmus_task_resume,

	TP_PROTO(struct task_struct *t),

	TP_ARGS(t),

	TP_STRUCT__entry(
		__field( pid_t,		pid	)
		__field( unsigned int,	job	)
		__field( lt_t,		when	)
	),

	TP_fast_assign(
		__entry->pid	= t ? t->pid : 0;
		__entry->job	= t ? t->rt_param.job_params.job_no : 0;
		__entry->when	= litmus_clock();
	),

	TP_printk("resume(job(%u, %u)): %Lu\n",
			__entry->pid, __entry->job, __entry->when)
);

/*
 * Trace synchronous release
 */
TRACE_EVENT(litmus_sys_release,

	TP_PROTO(lt_t *start),

	TP_ARGS(start),

	TP_STRUCT__entry(
		__field( lt_t,		rel	)
		__field( lt_t,		when	)
	),

	TP_fast_assign(
		__entry->rel	= *start;
		__entry->when	= litmus_clock();
	),

	TP_printk("SynRelease(%Lu) at %Lu\n", __entry->rel, __entry->when)
);

/*
 * Containers
 */
TRACE_EVENT(litmus_container_param,

	TP_PROTO(int cid, const char *name),

	TP_ARGS(cid, name),

	TP_STRUCT__entry(
		__field( int,  cid )
		__array( char,	name,	TASK_COMM_LEN	)
	),

	TP_fast_assign(
	       memcpy(__entry->name, name, TASK_COMM_LEN);
	       __entry->cid = cid;
	),

	TP_printk("container, name: %s, id: %d\n", __entry->name, __entry->cid)
);

TRACE_EVENT(litmus_server_param,

	TP_PROTO(int sid, int cid, unsigned long long wcet, unsigned long long period),

	TP_ARGS(sid, cid, wcet, period),

	TP_STRUCT__entry(
		__field( int, sid )
		__field( int, cid )
		__field( unsigned long long, wcet )
		__field( unsigned long long, period )
	),

	TP_fast_assign(
	       __entry->cid = cid;
	       __entry->sid = sid;
	       __entry->wcet = wcet;
	       __entry->period = period;
	),

	TP_printk("server(%llu, %llu), sid: %llu, cont: %llu\n",
	          __entry->wcet, __entry->period, __entry->sid, __entry->cid)
);

TRACE_EVENT(litmus_server_switch_to,

	TP_PROTO(int sid, unsigned int job, int tid, unsigned int tjob, int cpu),

        TP_ARGS(sid, job, tid, tjob, cpu),

	TP_STRUCT__entry(
		__field( int, sid)
		__field( unsigned int, job)
		__field( int, tid)
		__field( unsigned int, tjob)
		__field( int, cpu)
	),

	TP_fast_assign(
		__entry->sid = sid;
		__entry->tid = tid;
		__entry->job = job;
		__entry->tjob = tjob;
		__entry->cpu = cpu;
	),

	TP_printk("switch_to(server(%d, %u)): (%d, %d) on %d\n",
		  __entry->sid, __entry->job, __entry->tid, __entry->tjob, __entry->cpu)
);

TRACE_EVENT(litmus_server_switch_away,

	    TP_PROTO(int sid, unsigned int job, int tid, unsigned int tjob, int cpu),

	    TP_ARGS(sid, job, tid, tjob, cpu),

	TP_STRUCT__entry(
		__field( int, sid)
		__field( unsigned int, job)
		__field( int, tid)
		__field( unsigned int, tjob)
		__field( int, cpu)
	),

	TP_fast_assign(
		__entry->sid = sid;
		__entry->tid = tid;
		__entry->job = job;
		__entry->tjob = tjob;
		__entry->cpu = cpu;
	),

	TP_printk("switch_away(server(%d, %u)): (%d, %d) on %d\n",
		  __entry->sid, __entry->job, __entry->tid, __entry->tjob, __entry->cpu)
);

TRACE_EVENT(litmus_server_release,

	TP_PROTO(int sid, unsigned int job,
		 unsigned long long release,
		 unsigned long long deadline),

	TP_ARGS(sid, job, release, deadline),

	TP_STRUCT__entry(
		__field( int, sid)
		__field( unsigned int, job)
		__field( unsigned long long, release)
		__field( unsigned long long, deadline)
	),

	TP_fast_assign(
		__entry->sid = sid;
		__entry->job = job;
		__entry->release = release;
		__entry->deadline = deadline;
	),

	TP_printk("release(server(%d, %u)), release: %llu, deadline: %llu\n",
		  __entry->sid, __entry->job, __entry->release, __entry->deadline)
);

TRACE_EVENT(litmus_server_completion,

	TP_PROTO(int sid, int job),

	TP_ARGS(sid, job),

	TP_STRUCT__entry(
		__field( int, sid)
		__field( unsigned int, job)
	),

	TP_fast_assign(
		__entry->sid = sid;
		__entry->job = job;
	),

	TP_printk("completion(server(%d, %d))\n", __entry->sid, __entry->job)
);

TRACE_EVENT(litmus_server_resume,

        TP_PROTO(int sid),

        TP_ARGS(sid),

	TP_STRUCT__entry(
		__field( int, sid)
	),

	TP_fast_assign(
		__entry->sid = sid;
	),

        TP_printk("resume(server(%d))\n", __entry->sid)
);

TRACE_EVENT(litmus_server_block,

	TP_PROTO(int sid),

	TP_ARGS(sid),

	TP_STRUCT__entry(
		__field( int, sid)
	),

	TP_fast_assign(
		__entry->sid = sid;
	),

	TP_printk("block(server(%d))\n", __entry->sid)
);

#endif /* _SCHED_TASK_TRACEPOINT_H */

/* Must stay outside the protection */
#include <trace/define_trace.h>
