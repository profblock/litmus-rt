#include <linux/kernel.h>

#include <litmus/dbf.h>

#include <linux/string.h>
#include <linux/list.h>

#define list_tail(list, type, member)           \
        list_entry((list)->prev, type, member)

void dbf_init(struct dbf *dbf, lt_t e, lt_t p, lt_t d)
{
	dbf_clear(dbf);
	dbf->slope = -1;
	dbf->npoints = 0;
	dbf->e = e;
	dbf->p = p;
	dbf->d = d;
}

void dbf_init_dup(struct dbf *dbf, struct dbf *to_copy)
{
	memcpy(dbf, to_copy, sizeof(struct dbf));
}

int dbf_init_rtparams(struct dbf *dbf, lt_t e, lt_t p, lt_t d)
{
	int i;
	lt_t x;
	lt_t y;
	slope_t slope;
	slope_t pslope;
	unsigned int type;

	pr_emerg("new DBF e=%llu, p=%llu, d=%llu\n", e, p, d);

	dbf_init(dbf, e, p, d);
	if (dbf_append_point(dbf, 0, 0, POINT_TYPE_STEP, 0) < 0) {
		pr_info("failed to append point 0,0\n");
		return -1;
	}

	/* This is rounded down (pessimistic) which is what we want */
	slope = (e*PRECISION)/p;

	for (i = 0; i < DBF_NPOINTS - 1; i++) {
		/* treat the last point of the DBF differently */
		if (i == (DBF_NPOINTS - 1)) {
			type = POINT_TYPE_CONT;
			pslope = slope;
		} else {
			type = POINT_TYPE_STEP;
			pslope = 0;
		}

		x = d + (i * p);
		y = e * (i + 1);
		if (dbf_append_point(dbf, x, y, type, pslope) < 0)
			goto err_append;
	}

	dbf_set_slope(dbf, slope);

	return 0;

err_append:
	pr_info("error appending point %d to DBF\n", i);
	while (--i >= 0)
		dbf_delete_point(dbf, i);
	return -1;
}

int dbf_append_point(struct dbf *dbf, lt_t x, lt_t y,
	unsigned int flags, slope_t slope)
{
	struct point *p;

	if (dbf->npoints >= DBF_NPOINTS) {
		pr_info("DBF has too many points. npoints = %d\n", dbf->npoints);
		return -1;
	}

	p = &dbf->points[dbf->npoints++];

	memset(p, 0, sizeof(struct point));
	p->x = x;
	p->y = y;
	p->flags = flags;
	p->slope = slope;

	/*
	 * TODO: when using arbitrary DBFs, change the slope of the previously
	 * last point when adding a new one
	 *
	 * TODO: perhaps some error checking? eg. no point appended should make the
	 * DBF non-decreasing.
	 */

	return 0;
}

void dbf_set_slope(struct dbf *dbf, slope_t slope)
{
	dbf->slope = slope;
}

void dbf_delete_point(struct dbf *dbf, unsigned int idx)
{
	unsigned int i = 0;

	if (idx >= dbf->npoints)
		return;

	for (i = idx; i < dbf->npoints - 1; i++)
		memcpy(&dbf->points[i], &dbf->points[i+1], sizeof(struct point));
	
	dbf->npoints--;
}

struct point *dbf_get_point(struct dbf *dbf, unsigned int idx)
{
	if (idx >= dbf->npoints)
		return NULL;

	return &dbf->points[idx];
}

void dbf_clear(struct dbf *dbf)
{
	memset(dbf, 0, sizeof(struct dbf));
}

void dbf_dump(struct dbf *dbf)
{
	int i;
	struct point *p;

	if (!dbf)
		return;

	for (i = 0; i < dbf->npoints; i++) {
		p = &dbf->points[i];
		pr_info("POINT(%llu, %llu, type=%s, slope=%d)\n", p->x, p->y,
			p->flags ? "cont" : "step", p->slope);
	}

	pr_info("SLOPE(%d)\n", dbf->slope);
}

int dbf_point_at(struct point *r, struct dbf *dbf, lt_t x, int opts)
{
	int i;
	struct point *p;
	struct point *p1 = NULL;
	struct point *p2 = NULL;
	slope_t add;

	if (!r)
		return -1;

	/* get the last point before the requested coordinate */
	for (i = 0; i < dbf->npoints; i++) {
		p = &dbf->points[i];
		p2 = p;
		if (p->x > x)
			break;
		/* not possible for p1 to be NULL because of (0,0) */
		p1 = p;
	}
	
	memset(r, 0, sizeof(struct point));

	/*
	 * since we add point (0,0), p1 and p2 can never be NULL
	 * Also x coordinate is unsigned so no negative numbers
	 * At this point, either coordinate x lies between p1 and p2
	 * OR p1 == p2 (ie. we must use slope of the point).
	 */
	r->x = x;
	/* trivial: we found the exact point */
	if (p1->x == x) {
		r->y = p1->y;
		r->slope = p1->slope;
		r->flags = p1->flags;
		return 0;
	}

	if (p1 == p2) {
		/* we need to use the slope to calculate the point (use p1) */
		/* here we can choose to round up or down */
		add = (opts == P_OPT_CEIL) ? PRECISION : 0;
		r->y = p1->y + (p1->slope * (x - p1->x) + add)/PRECISION;
		r->flags = p1->flags;
		r->slope = p1->slope;
	} else {
		/*
		 * this assert will prevent people from appending to DBFs in a
		 * way that makes them non-decreasing.
		 */
		if (p1->flags == POINT_TYPE_STEP) {
			/* r is a point between two points of a step function */
			r->y = p1->y;
			r->flags = POINT_TYPE_STEP;
			r->slope = 0;
		} else {
			/* r is a point between points of a continuous function */
			add = (opts == P_OPT_CEIL) ? PRECISION : 0;
			r->y = p1->y + (p1->slope * (x - p1->x) + add)/PRECISION;
			r->flags = POINT_TYPE_CONT;
			r->slope = p1->slope;
		}
	}

	return 0;
}

int dbf_add(struct dbf *res, struct dbf *a, struct dbf *b)
{
	unsigned int i = 0;
	unsigned int j = 0;

	dbf_clear(res);
	dbf_init(res, 0, 0, 0);

	while (i < a->npoints && j < b->npoints && res->npoints < DBF_NPOINTS) {
		struct point *pa;
		struct point *pb;
		struct point x;

		pa = dbf_get_point(a, i);
		pb = dbf_get_point(b, j);

		if (pa->x < pb->x) {
			dbf_point_at(&x, b, pa->x, 0);
			dbf_append_point(res, pa->x, pa->y + x.y, x.flags | pa->flags,
				x.slope + pa->slope);
			i++;
		} else {
			/* pa->x >= pb->x */
			dbf_point_at(&x, a, pb->x, 0);
			dbf_append_point(res, pb->x, pb->y + x.y, x.flags | pb->flags,
				x.slope + pb->slope);
			j++;
			if (pa->x == pb->x)
				i++;
		}
	}

	/* now blindly add the remaining points in the two DBFs*/
	while (i < a->npoints && res->npoints < DBF_NPOINTS) {
		struct point *pa;
		struct point x;
		pa = dbf_get_point(a, i);
		dbf_point_at(&x, b, pa->x, 0);
		dbf_append_point(res, pa->x, pa->y + x.y, x.flags | pa->flags,
			x.slope + pa->slope);
		i++;
	}

	while (j < b->npoints && res->npoints < DBF_NPOINTS) {
		struct point *pb;
		struct point x;
		pb = dbf_get_point(b, j);
		dbf_point_at(&x, a, pb->x, 0);	
		dbf_append_point(res, pb->x, pb->y + x.y, x.flags | pb->flags,
			x.slope + pb->slope);
		j++;
	}

	res->slope = a->slope + b->slope;

	return 0;
}

int dbf_less_than(struct dbf *a, struct dbf *b)
{
	unsigned int i;

	if (a->slope > b->slope)
		return 0;

	/* every point in a should be below b AND vice versa */
	for (i = 0; i < a->npoints; i++) {
		struct point *pa;
		struct point pb;

		pa = dbf_get_point(a, i);
		/* DBF b is floored while DBF a is ceiled */
		dbf_point_at(&pb, b, pa->x, P_OPT_FLOOR);

		if (pa->y > pb.y)
			return 0;
	}

	for (i = 0; i < b->npoints; i++) {
		struct point *pb;
		struct point pa;

		pb = dbf_get_point(b, i);
		dbf_point_at(&pa, a, pb->x, P_OPT_CEIL);

		if (pb->y < pa.y)
			return 0;
	}

	return 1;
}

long long dbf_find_interval(struct dbf *dbf, int opts)
{
	struct point *p;
	lt_t x;

	p = &dbf->points[dbf->npoints-1];
	
	/* x = (y1 - m*x1)/(m + 1) */
	x = ((p->y*PRECISION - p->slope*p->x)/(p->slope + PRECISION));

	if (!(x % PRECISION))
		return x/PRECISION;

	if (opts == P_OPT_CEIL)
		return ((x + PRECISION)/PRECISION);
	else
		return x / PRECISION;
}


int dbf_highest_point_index(struct dbf *dbf, lt_t x)
{
	int i = 0;
	struct point *p;

	for (i = 0; i < dbf->npoints; i++) {
		p = &dbf->points[i];
		if (p->x > x)
			return i - 1;
		else if (p->x == x)
			return i;	/* TODO: assume no DBF has 2 same points */
	}

	return -1;
}
