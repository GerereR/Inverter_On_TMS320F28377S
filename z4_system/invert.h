#ifndef INVERT_H_
#define INVERT_H_

void Invert_Init(void);
void Invert_EnterSafeOutput(void);
float Invert_Clamp(float value, float minimum, float maximum);

#endif /* INVERT_H_ */
