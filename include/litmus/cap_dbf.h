#ifndef LINUX_DBF_CAP_H
#define LINUX_DBF_CAP_H

#include <linux/list.h>

#include <litmus/dbf.h>

#define CAPABILITY_TOP_LEVEL	(1 << 0)
#define CAPABILITY_LEAF_LEVEL	(1 << 1)

struct cap_dbf {
	struct dbf dbf;			/* The DBF of this capability (NULL for root) */
	struct list_head children;	/* List of child capabilities (splits) */
	struct cap_dbf *parent;		/* The parent capability */
	struct task_struct *owner;	/* Task owning this capability */
	struct list_head list;
	int flags;
};

/* init a static capability */
void cap_dbf_init(struct cap_dbf *cap,
	struct cap_dbf *parent, struct task_struct *tsk, unsigned int flags);
/* create a new capability */
struct cap_dbf *cap_dbf_create(lt_t e, lt_t p, lt_t d, struct cap_dbf *parent,
	struct task_struct *tsk, unsigned int flags);
/* Split a capability by adding child to the parent (err if inadmissioble */
int cap_dbf_split(struct cap_dbf *parent, struct cap_dbf *child);
/* Destroy a capability and return the resources to the parent */
int cap_dbf_destroy(struct cap_dbf *cap);
void cap_dbf_assign(struct cap_dbf *cap, struct task_struct *tsk);

#endif
