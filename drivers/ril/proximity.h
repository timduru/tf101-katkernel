#ifndef _PROXIMITY_H
#define _PROXIMITY_H

#ifdef CONFIG_PROX
extern int prox_lds6202_enable(void);
extern int prox_lds6202_disable(void);
#endif

int init_proximity(void);
void free_proximity(void);
void ril_request_proxi(int state);

#endif

