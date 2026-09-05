#ifndef POWER_LIMIT_H_
#define POWER_LIMIT_H_

/* Initialize the runtime state owned by the power-limit manager. */
void PowerLimit_Init(void);

/* Update power and current limits from measurements and operating rules. */
void PowerLimit_Update(void);

#endif /* POWER_LIMIT_H_ */
