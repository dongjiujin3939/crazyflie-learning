#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

#include "controller.h" // 包含控制结构的定义

void pid_Init();
void pid_Update();
bool velocityPidController_Test();
void velocityPidController_Init();
void velocityPidController_Update(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensorData, const state_t *state, const uint32_t);

#endif // SPEED_CONTROLLER_H