#ifndef REACTIVE_CTRL_H_
#define REACTIVE_CTRL_H_

/* Initialize the runtime state owned by the reactive-power controller. */
void ReactiveCtrl_Init(void);

/* Update reactive-power commands and capacitor compensation in the future. */
void ReactiveCtrl_Update(void);

#endif /* REACTIVE_CTRL_H_ */
