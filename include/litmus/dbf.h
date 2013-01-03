#ifndef LITMUS_HSCHED_H
#define LITMUS_HSCHED_H

#include <linux/list.h>

#define DBF_NPOINTS	100		/* user-configurable number of points */
#define PRECISION	1000		/* minimum slope is 0.001 */

/*
 * IMPORTANT NOTE: ALL FUNCTIONS RELATED TO DBF's ASSUME CORRECT AND
 * VALID POINTERS. ALL CHECKS ARE DONE IN cap_dbf.c FOR POINTER VALIDITY.
 * THE USER ONLY TOUCHED cap_dbf.c FUNCTIONS.
 */

typedef int	slope_t;

/*
 * Define a point whose transition to the next point is a step
 * If set to 0, the next point is a continuous slope.
 */
#define POINT_TYPE_STEP	0
#define POINT_TYPE_CONT	1

/*
 * When retrieving points on a DBF, choose whether to be optimisitc or
 * pessimistic in our calculations
 */
#define P_OPT_FLOOR	1
#define P_OPT_CEIL	0

/* Structure defining a point and the region to the next one */
struct point {
	lt_t x;			/* X-coordinate of point */
	lt_t y;			/* Y-coordinate of point */
	unsigned int flags;		/* flags. currently just the point type */
	slope_t slope;			/* slope of line (see PRECISION) */
};

/* Structure defining a DBF */
struct dbf {
	struct point points[DBF_NPOINTS];
	unsigned int npoints;		/* number of points in DBF */
	slope_t slope;			/* final approximated slope of DBF */
	lt_t e;			/* E, P and D, stored here for convenince */
	lt_t p;
	lt_t d;
};

/* initialize an empty DBF structure */
void dbf_init(struct dbf *dbf, lt_t e, lt_t p, lt_t d);
/* initialize an empty DBF as a copy of another */
void dbf_init_dup(struct dbf *dbf, struct dbf *to_copy);
/* initialize a DBF structure using real-time task parameters. */
int dbf_init_rtparams(struct dbf *dbf, lt_t e, lt_t p, lt_t d);

/* append a single point to the end of a DBF structure */
int dbf_append_point(struct dbf *dbf, lt_t x, lt_t y,
	unsigned int flags, slope_t slope);
/* set the approximation slope (Utilization) to a DBF structure */
void dbf_set_slope(struct dbf *dbf, slope_t slope);

/* get a point from the DBFs point array (by index) */
struct point *dbf_get_point(struct dbf *dbf, unsigned int idx);
/* get demand at coordinate x for a given DBF */
int dbf_point_at(struct point *r, struct dbf *dbf, lt_t x, int opts);
/* find point of intersection of the DBF with x=y line */
long long dbf_find_interval(struct dbf *dbf, int opts);
/* find the highest index where p->x < x */
int dbf_highest_point_index(struct dbf *dbf, lt_t x);

/* print a DBF structure */
void dbf_dump(struct dbf *dbf);

/* add two DBF's and return the resulting DBF */
int dbf_add(struct dbf *res, struct dbf *a, struct dbf *b);
/* check if every point in DBF 'b' lies below DBF 'a' and vice versa */
int dbf_less_than(struct dbf *a, struct dbf *b);

/* clear a single point in a DBF structure (by index) */
void dbf_delete_point(struct dbf *dbf, int unsigned idx);
/* clear a DBF structure (only the points inside) */
void dbf_clear(struct dbf *dbf);

#endif
