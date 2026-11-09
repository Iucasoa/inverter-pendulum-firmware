#ifndef SWINGUP_H
#define SWINGUP_H

/* ============================================================
 * SWING-UP POR BOMBEAMENTO DE ENERGIA (pêndulo rotacional / Furuta)
 *
 * Ideia: o braço do motor oscila em sincronia com a haste, injetando
 * energia a cada meia-oscilação, até a haste chegar ao topo com
 * velocidade ~0. Lá o PID do LGR (pid.c) assume.
 *
 * Energia normalizada da haste (0 = parada embaixo, 2 = parada no topo):
 *     E = 0.5*(thd/w0)^2 + (1 - cos th)
 * Lei de bombeamento (Astrom-Furuta), com saturação suave:
 *     u = A * sat(kE*(Eref - E)) * tanh(thd*cos(th)/s0)
 *     aceleração do braço = -sinal_motor*u - kp*braco - kd*vel_braco
 * O termo kp/kd é uma "mola" que mantém o braço perto do centro,
 * e uma guarda de frenagem garante que o braço não passe de lim_braco
 * (protege o cabo).
 *
 * Fases:
 *   CHUTE      -> tranco curto no braço (vai e volta) para tirar a haste do repouso
 *   MEDINDO    -> (só na 1ª vez) segura o braço e cronometra a haste oscilando
 *                 livre; corrige pela amplitude e define w0 automaticamente
 *   BOMBEANDO  -> injeta energia até a haste chegar ao topo
 *                 (se a haste ficar parada > 1 s, volta para CHUTE)
 *
 * Identificação do sentido: no tranco, com a haste parada embaixo, o braço
 * acelera para + e o firmware vê para que lado a haste se move nos primeiros
 * 40 ms. Isso define o sentido do bombeio direto pela física, sem depender
 * de sinal_motor.
 *
 * Se a haste já estava balançando (não dá para identificar no tranco):
 *  - atrito só tira energia, então toda meia-oscilação que GANHA altura revela
 *    o sentido certo (compara o ganho com o "trabalho" acel*thd*cos(th) do braço);
 *  - se os picos param de crescer por 6 meias-oscilações, tenta o outro sentido;
 *  - se o pico da oscilação ENCOLHE
 * (para < 75% do anterior duas vezes, ou despenca para < 2°), ou a haste para
 * 2 vezes, o bombeio está tirando energia em vez de colocar -> inverte
 * sinal_bombeio sozinho.
 * ============================================================ */

#include <stdint.h>
#include "pid.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SW_CHUTE = 0,
    SW_MEDINDO,
    SW_BOMBEANDO
} SwingFase_t;

/* Parâmetros (preenchidos no .ino a partir dos #defines) */
typedef struct {
    float dt;               /* período de controle (s)                              */
    float cpr;              /* contagens do encoder por volta                       */
    float w0_padrao;        /* rad/s - usado se a medição automática falhar         */
    float acel_bombeio;     /* rad/s^2 - aceleração do braço para bombear energia   */
    float ganho_energia;    /* kE (adimensional)                                    */
    float s0;               /* rad/s - suavização de sinal(thd*cos th)              */
    float kp_braco;         /* 1/s^2 - mola de centralização do braço               */
    float kd_braco;         /* 1/s   - amortecimento do braço                       */
    float lim_braco_rad;    /* rad   - o braço freia para não passar disso          */
    float acel_freio;       /* rad/s^2 - aceleração de frenagem da guarda           */
    float acel_max;         /* rad/s^2 - limite absoluto do motor                   */
    float zona_captura_rad; /* rad   - distância do topo em que o PID assume        */
    float margem_passo;     /* aumento de Eref por meia-oscilação que não chega     */
    float margem_max;       /* margem máxima de energia                             */
    float chute_rad;        /* rad   - amplitude do tranco inicial                  */
    float chute_t;          /* s     - duração da ida do tranco                     */
    float vel_filt_hz;      /* Hz    - corte do filtro da velocidade da haste       */
} SwingParams_t;

typedef struct {
    SwingParams_t p;
    SwingFase_t fase;
    float t_fase;           /* tempo na fase atual (s)                     */

    /* Sensores derivados do encoder */
    int   sensor_ok;
    long  count_ant;
    float alfa_vel;
    float th;               /* [0, 2pi)  0 = embaixo                       */
    float th_w;             /* (-pi, pi] 0 = embaixo                       */
    float th_w_ant;
    float thd;              /* velocidade da haste filtrada (rad/s)        */
    float energia;          /* energia normalizada (2 = topo)              */
    int8_t sinal_thd;       /* sentido atual da haste (com histerese)      */
    int   virou;            /* 1 no ciclo em que a haste inverteu sentido  */
    float pico_meia;        /* maior |th_w| na meia-oscilação atual        */
    float pico_ultimo;      /* pico da meia-oscilação que acabou de fechar */

    /* Período natural */
    float w0;
    int   w0_medido;
    int   n_cruz;
    float t_cruz0, t_cruz_ult;
    float amp_soma;
    int   amp_n;
    int   med_tentada;      /* já tentou medir o período desde o Init      */
    float braco_ref;        /* posição do braço no início do tranco        */
    float t_parado;         /* tempo com a haste parada durante o bombeio  */

    /* Bombeamento */
    float margem;           /* Eref = 2*(1 + margem)                       */
    int8_t sinal_bombeio;   /* +1/-1: sentido do bombeio (corrigido sozinho) */
    int   n_queda;          /* meias-oscilações seguidas perdendo altura   */
    int   n_paradas;        /* vezes que a haste parou durante o bombeio   */
    float ep_ant;           /* pico (rad) da meia-oscilação anterior       */
    int   evento_inversao;  /* 1 quando inverteu o sentido (o .ino imprime e zera) */
    int   identificado;     /* 1 se o sentido foi medido fisicamente no tranco */
    int   ident_pendente;   /* tranco atual começou com a haste parada      */
    long  count_chute0;     /* encoder no início do tranco                  */
    float trab;             /* soma de acel*thd*cos(th)*dt na meia-oscilação */
    float ep_trab;          /* energia do pico anterior (1 - cos pico)      */
    int8_t ev_sinal;        /* sentido apontado pela última evidência       */
    int   ev_n;             /* evidências seguidas concordando             */
    int   confirmado;       /* sentido confirmado por ganho de energia      */
    float pico_melhor;      /* maior pico desde a última inversão           */
    int   n_estag;          /* meias-oscilações sem bater o maior pico      */
} SwingUp_t;

void  Swing_Init(SwingUp_t *s, const SwingParams_t *p);
/* Recomeça o swing (após queda ou novo comando). Mantém w0 medido e a margem aprendida. */
void  Swing_Reiniciar(SwingUp_t *s);
/* Chamar TODO ciclo de controle (mesmo equilibrando): atualiza ângulo, velocidade e energia. */
void  Swing_AtualizarSensores(SwingUp_t *s, long encoder_count);
/* Retorna a aceleração do braço (rad/s^2) no sentido "cru" do motor de passo. */
float Swing_Calcular(SwingUp_t *s, float braco_rad, float braco_vel, int8_t sinal_motor);

const char *Swing_NomeFase(const SwingUp_t *s);

#ifdef __cplusplus
}
#endif

#endif
