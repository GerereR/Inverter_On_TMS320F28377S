#ifndef INVERTER_H_
#define INVERTER_H_

void Inverter_Init(void);
void Inverter_EnterSafeOutput(void);
float Inverter_Clamp(float value, float minimum, float maximum);

#endif /* INVERTER_H_ */
