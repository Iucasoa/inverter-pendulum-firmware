#include "pid.h"
#include <math.h>

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd,
              float dt, float limite_torque)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->dt = dt;
    pid->limite_torque = limite_torque;
    /* Coeficiente do filtro passa-baixas de 1ª ordem na derivada:
     * alpha_d = 1 - exp(-2*pi*fc*dt)
     */
    pid->alpha_d = 1.0f - expf(-2.0f * PI_F * DERIV_FILT_HZ * dt);
    PID_Reset(pid);
}

/* K > 0, z > 0  →  Kp, Ki, Kd todos positivos */
void PID_InitKz(PID_t *pid, float K, float z,
                float dt, float limite_torque)
{
    PID_Init(pid,
             2.0f * K * z,   /* Kp */
             K * z * z,     /* Ki */
             K,             /* Kd */
             dt, limite_torque);
}

void PID_Reset(PID_t *pid)
{
    pid->integral       = 0.0f;
    pid->erro_anterior  = 0.0f;
    pid->deriv_filtrada = 0.0f;
    pid->torque_saida   = 0.0f;
}

float PID_Calcular(PID_t *pid, float setpoint_graus,
                   float angulo_encoder_graus)
{
    /* 1) Erro em rad (mesma unidade da FT) */
    float e = (setpoint_graus - angulo_encoder_graus) * DEG2RAD;

    /* 2) Termo P e Termo D com filtro passa-baixas de 1ª ordem
     *    (elimina os espasmos de quantização de 0.15°/pulso no motor de passo)
     */
    float P = pid->Kp * e;
    float deriv_bruta = (e - pid->erro_anterior) / pid->dt;
    pid->deriv_filtrada += pid->alpha_d * (deriv_bruta - pid->deriv_filtrada);
    float D = pid->Kd * pid->deriv_filtrada;

    /* 3) Termo I com anti-windup condicional + vazamento leve (leaky integrator)
     *    para evitar que o erro acumulado na subida arremesse a haste ao chegar em 180°
     */
    float i_novo = (pid->integral * 0.995f) + e * pid->dt;
    float I      = pid->Ki * i_novo;

    float u = P + I + D;

    if (pid->limite_torque > 0.0f) {
        if (u > pid->limite_torque) {
            u = pid->limite_torque;
            if (e < 0.0f) pid->integral = i_novo;   /* só integra se ajuda */
        } else if (u < -pid->limite_torque) {
            u = -pid->limite_torque;
            if (e > 0.0f) pid->integral = i_novo;
        } else {
            pid->integral = i_novo;
        }
    } else {
        pid->integral = i_novo;
    }

    pid->erro_anterior = e;
    pid->torque_saida  = u;
    return u;
}
