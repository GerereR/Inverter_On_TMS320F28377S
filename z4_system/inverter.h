#ifndef INVERTER_H_
#define INVERTER_H_

void Inverter_Init(void);
void Inverter_EnterSafeOutput(void);

/* Returns zero when the value or range is invalid. */
float Inverter_Clamp(float value, float minimum, float maximum);

#endif /* INVERTER_H_ */
