#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <litmus/litmus.h>
#include <litmus/rt_param.h>
#include <litmus/cap_dbf.h>

static struct dbf sum;
static struct dbf tmp;

void cap_dbf_init(struct cap_dbf *cap,
	struct cap_dbf *parent, struct task_struct *tsk, unsigned int flags)
{
	/* TODO: if cap is already created, do cleanup here */
	INIT_LIST_HEAD(&cap->children);
	cap->parent = parent;
	cap->flags = flags;
	cap->owner = tsk;
}

struct cap_dbf *cap_dbf_create(lt_t e, lt_t p, lt_t d, int id,
	struct cap_dbf *parent, struct task_struct *tsk, unsigned int flags)
{
	int ret;
	struct cap_dbf *c;

	c = kmalloc(sizeof(struct cap_dbf), GFP_ATOMIC);
	if (!c)
		return NULL;

	ret = dbf_init_rtparams(&c->dbf, e, p, d);
	if (ret < 0)
		return NULL;

	cap_dbf_init(c, parent, tsk, flags);
	c->cid = id;

	return c;
}

int do_schedulability_check(struct cap_dbf *parent, struct cap_dbf *new)
{
	int i = 0;
	slope_t util;
	struct cap_dbf *c;
	long long interval = 0;	/* can be negative */
	unsigned int imax;
	struct point q;

	/* check that utilization does not exceed 100% */
	util = new->dbf.slope;
	list_for_each_entry(c, &parent->children, list)
		util += c->dbf.slope;
	if (util > PRECISION)
		return -1;

	/* Do a QPA check for EDF schedulability */

	/* calculate the highest point to check (L) */
	dbf_init_dup(&sum, &new->dbf);
	list_for_each_entry(c, &parent->children, list) {
		dbf_add(&tmp, &sum, &c->dbf);
		dbf_init_dup(&sum, &tmp);
	}

	/* find the interval to use (be pessimistic) */
	interval = dbf_find_interval(&sum, P_OPT_CEIL);

	/* if interval is negative, we are schedulable */
	if (interval < 0)
		return 1;

	/* for each, do QPA till you hit demand < dmin */
	imax = dbf_highest_point_index(&sum, interval);
	if (imax == 0)
		goto istrue;

	for (i = imax; i >= 0; i--) {
		struct point *p;

		p = dbf_get_point(&sum, i);

		while (p->y < p->x && p->x != 0) {
			dbf_point_at(&q, &sum, p->y, P_OPT_CEIL);
			p = &q;
		}

		if (p->x == 0)
			goto istrue;
		else if (p->x > p->y)
			goto isfalse;
		else if (p->x == p->y)
			continue;
	}

isfalse:
	return -1;
istrue:
	return 0;
}

int cap_dbf_split(struct cap_dbf *parent, struct cap_dbf *child)
{
	struct dbf tmp;
	struct dbf sum;
	struct cap_dbf *c;

	/* 2 cases: top level or leaf level */
	if (parent->flags & CAPABILITY_TOP_LEVEL) {
		int ret;
		ret = do_schedulability_check(parent, child);
		if (ret < 0) {
			pr_info("top level split failed. QPA check failed\n");
			return ret;
		}
	} else {
		/*
		 * we only check the child DBFs since we want the DBF and task
		 * trees completely independent
		 */
		pr_info("non top-level split requested\n");
		dbf_init_dup(&sum, &child->dbf);
		list_for_each_entry(c, &parent->children, list) {
			dbf_add(&tmp, &sum, &c->dbf);
			dbf_init_dup(&sum, &tmp);
		}

		if (!dbf_less_than(&sum, &parent->dbf)) {
			pr_info("failed to admit task. DBF violates parent\n");
			dbf_clear(&sum);
			return -1;
		}

		dbf_clear(&sum);
	}

	pr_info("successfully admitted task\n");

	list_add(&child->list, &parent->children);

	return 0;
}

int cap_dbf_destroy(struct cap_dbf *cap)
{
	int i;
	/* TODO */
	/* Remove RT capability of all children. ie. call destroy on them */
	if (cap->owner) {
		for (i = 0; i < NR_CPUS; i++)
			set_cap_provider(cap->owner, i, NULL);
		/* TODO: Revoke RT capabilities of owner task */
	}
	kfree(cap);
	return 0;
}

void cap_dbf_assign(struct cap_dbf *cap, struct task_struct *tsk)
{
#if 0
	TODO:
	if (tsk->cap) {
		cap_dbf_destroy(tsk->cap);
		kfree(tsk->cap);
	}
#endif
	set_rt_capability(tsk, cap);
	cap->owner = tsk;
}

struct cap_dbf *cap_dbf_find(struct cap_dbf *parent, int id)
{
	struct cap_dbf *t;
	struct cap_dbf *res;

	if (parent->cid == id)
		return parent;

	list_for_each_entry(t, &parent->children, list) {
		res = cap_dbf_find(t, id);
		if (res)
			return res;
	}

	return NULL;
}
