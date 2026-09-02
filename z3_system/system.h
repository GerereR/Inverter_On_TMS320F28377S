#ifndef SYSTEM_H_
#define SYSTEM_H_

/* System startup and peripheral initialization entry point. */
void System_Init(void);
void System_EnterSafeOutput(void);

/* Shared floating-point limiter for control and supervisory algorithms. */
float System_Clamp(float value, float minimum, float maximum);

#endif /* SYSTEM_H_ */
