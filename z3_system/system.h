#ifndef SYSTEM_H_
#define SYSTEM_H_

/* System startup and peripheral initialization entry point. */
void System_Init(void);
void System_EnterSafeOutput(void);

/* Shared floating-point limiter for control and supervisory algorithms. */
float System_Clamp(float value, float minimum, float maximum);

/* Grid-relay self-test (scheme A: grid-vs-inverter voltage difference).
 * Init() resets the sequence and opens all relays. SelfTest(deltaMs) advances
 * the timing, drives the relay GPIOs and evaluates the adhesion/failure check.
 * Results are reported through gRelayData.selfTestPassed / gRelayData.fault. */
void System_RelaySelfTestInit(void);
void System_RelaySelfTest(Uint16 deltaMs);

#endif /* SYSTEM_H_ */
