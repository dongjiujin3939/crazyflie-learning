#include "stabilizer_types.h"
#include "controller.h"
#include "sensors.h"
#include <stdlib.h>
// #include "pid.h"
// #include "platform_defaults.h"
// #include "num.h"

typedef struct {
    float kp;
    float ki;
    float kd;
    float integrator;
    float previousError;
    float output;
    float integratorMax;
    float integratorMin;
} PID_t;

void pid_Init(PID_t *pid, float kp, float ki, float kd,float integratorMax, float integratorMin) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integrator = 0.0;
    pid->previousError = 0.0;
    pid->output = 0.0;
    pid->integratorMax = integratorMax;
    pid->integratorMin = integratorMin;
}

float pid_Update(PID_t *pid, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;
    pid->integrator += error * dt;

    if(pid->integrator > pid->integratorMax){
        pid->integrator = pid->integratorMax;
    } else if(pid->integrator < pid->integratorMin){
        pid->integrator = pid->integratorMin;
    } // 积分限幅
    float derivative = (error - pid->previousError) / dt;
    pid->output = pid->kp * error + pid->ki * pid->integrator + pid->kd * derivative;
    pid->previousError = error;
    return pid->output;
}

static PID_t velocityPidRoll;
static PID_t velocityPidPitch;
static PID_t velocityPidYaw;

void velocityPidController_Init(void) {
    pid_Init(&velocityPidRoll, 3.0, 3.0, 0.0,100.0,-100.0);  
    pid_Init(&velocityPidPitch, 3.0, 3.0, 0.0,100.0,-100.0); 
    pid_Init(&velocityPidYaw, 3.0, 3.0, 0.0, 100.0,-100.0);
}
bool velocityPidController_Test(){
    // bool pass = true;
    return true;
}


void velocityPidController_Update(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensorData, const state_t *state, const uint32_t tick) {
    // 假设 dt 是由 tick 或其他参数计算得出的
    // float dt = 0.001f; // 这个值需要根据 `tick` 或其他机制计算
    static uint32_t lastTick = 0;
    uint32_t currentTick = tick;
    float dt = (currentTick - lastTick) * 0.001f ;
    lastTick = currentTick;

    static float previousGyroX = 0.0f, previousGyroY = 0.0f, previousGyroZ = 0.0f;
    float alpha = 0.8f;  // 滤波系数
    float filteredGyroX = alpha * previousGyroX + (1 - alpha) * sensorData->gyro.x;
    float filteredGyroY = alpha * previousGyroY + (1 - alpha) * sensorData->gyro.y;
    float filteredGyroZ = alpha * previousGyroZ + (1 - alpha) * sensorData->gyro.z;

    previousGyroX = filteredGyroX;
    previousGyroY = filteredGyroY;
    previousGyroZ = filteredGyroZ;
    // 使用 setpoint 中的速度目标和 sensorData 中的当前传感器数据更新 PID 控制器
    float rollControl = pid_Update(&velocityPidRoll, setpoint->velocity.x, sensorData->gyro.x, dt);
    float pitchControl = pid_Update(&velocityPidPitch, setpoint->velocity.y, sensorData->gyro.y, dt);
    float yawControl = pid_Update(&velocityPidYaw, setpoint->velocity.z, sensorData->gyro.z, dt);  // 新增: Yaw 控制

    // 将计算的控制信号写入 control 结构
    control->roll = rollControl;
    control->pitch = pitchControl;
    control->yaw = yawControl; // 新增: 将 Yaw 控制信号写入 control 结构

    control->thrust = setpoint->thrust;
    
    // if (sensorData->batteryLevel < BATTERY_LOW_THRESHOLD) 
    // {
    //     // 如果电池电量低，可能需要采取措施，例如减小推力或发出警告
    //     control->thrust = MIN(control->thrust, MAX_SAFE_THRUST); 
    // }

    // // 其他异常情况处理（如传感器失效）
    // if (!sensorData->gyro.isValid) {
    //     // 如果陀螺仪数据无效，停止控制或采取其他措施
    //     control->roll = 0.0f;
    //     control->pitch = 0.0f;
    //     control->yaw = 0.0f;
    //     control->thrust = 0.0f;
    // }
}
