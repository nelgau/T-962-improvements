#ifndef REFLOW_H_
#define REFLOW_H_

typedef enum eReflowMode {
	REFLOW_INITIAL=0,
	REFLOW_STANDBY,
	REFLOW_BAKE,
	REFLOW_REFLOW,
	REFLOW_STANDBYFAN
} ReflowMode_t;

#define SETPOINT_MIN 30
#define SETPOINT_MAX 260

// 36 hours max timer
#define BAKE_TIMER_MAX (60 * 60 * 36)

void Reflow_Init(void);
void Reflow_SetMode(ReflowMode_t themode);
ReflowMode_t Reflow_GetMode(void);
void Reflow_LoadSetpoint(void);
uint16_t Reflow_GetSetpoint(void);
void Reflow_SetSetpoint(uint16_t thesetpoint);
int16_t Reflow_GetActualTemp(void);
uint8_t Reflow_IsDone(void);
int Reflow_IsPreheating(void);
void Reflow_SetBakeTimer(int seconds);
int Reflow_GetBakeTimer(void);
int Reflow_GetTimeLeft(void);
int32_t Reflow_Run(uint32_t thetime, float meastemp, uint8_t* pheat, uint8_t* pfan, int32_t manualsetpoint);
void Reflow_SetLogLevel(int);

#endif /* REFLOW_H_ */
