#include "swingup.h"
#include <math.h>

#define DOIS_PI_F      (2.0f * PI_F)
#define HIST_VEL       0.3f    /* rad/s - histerese para detectar inversão de sentido   */
#define T_MEDICAO_MAX  8.0f    /* s     - desiste de medir o período depois disso      */
#define AMP_MEDIR_MAX  1.2f    /* rad   - prefere cronometrar oscilações menores que ~70° */
#define AMP_MEDIR_CRESC 0.3f   /* rad/s - ...mas aceita amplitudes maiores com o tempo     */
#define N_CRUZ_MEDIR   3       /* passagens pelo ponto baixo => 2 períodos completos   */
#define E_PARADO       0.004f  /* energia abaixo disso (~5°) = haste praticamente parada */
#define T_PARADO_MAX   1.0f    /* s     - parada por mais que isso no bombeio => tranco  */
#define N_QUEDA_INV    2       /* picos encolhendo seguidos => bombeio no sentido errado */
#define PICO_MIN_RAD   0.052f  /* 3°   - abaixo disso o pico é ruído e não é avaliado   */
#define PICO_COLAPSO   0.035f  /* 2°   - pico que despenca para isso => sentido errado  */
#define PICO_AVAL_MAX  1.22f   /* 70°  - só avalia abaixo disso (lá o bombeio certo cresce) */
#define T_IDENT        0.04f   /* s    - janela do tranco usada para ver o sentido da haste */
#define CONT_IDENT_MIN 3       /* contagens (~0.45°) mínimas para confiar na identificação */
#define TRAB_MIN       0.05f   /* trabalho mínimo do braço para valer como evidência      */
#define GANHO_MIN      0.005f  /* ganho mínimo de energia de pico para valer como evidência */
#define N_EVID         2       /* evidências seguidas concordando => sentido confirmado    */
#define N_ESTAG_INV    6       /* meias-oscilações sem crescer => tenta o outro sentido    */
#define N_PARADAS_INV  2       /* paradas seguidas no bombeio => sentido invertido      */

static float limitar(float x, float lo, float hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

/* Servo de posição do braço (usado no tranco inicial e para segurar o braço) */
static float servo_braco(float ref, float braco, float vel, float wn, float amax)
{
    float a = wn * wn * (ref - braco) - 2.0f * wn * vel;   /* amortecimento crítico */
    return limitar(a, -amax, amax);
}

/* Guarda de frenagem: se, freando com acel_freio, o braço passaria de lim_braco,
 * freia agora. Garante que o swing nunca leve o braço até o limite do cabo. */
static float guarda_braco(const SwingParams_t *p, float a, float braco, float vel)
{
    float parada = braco + vel * fabsf(vel) / (2.0f * p->acel_freio);
    if (parada >  p->lim_braco_rad && vel > 0.0f) return -p->acel_freio;
    if (parada < -p->lim_braco_rad && vel < 0.0f) return  p->acel_freio;
    return a;
}

void Swing_Init(SwingUp_t *s, const SwingParams_t *p)
{
    s->p         = *p;
    s->alfa_vel  = 1.0f - expf(-DOIS_PI_F * p->vel_filt_hz * p->dt);
    s->sensor_ok = 0;
    s->count_ant = 0;
    s->th = s->th_w = s->th_w_ant = 0.0f;
    s->thd       = 0.0f;
    s->energia   = 0.0f;
    s->w0        = p->w0_padrao;
    s->w0_medido = 0;
    s->margem    = 0.0f;
    s->med_tentada = 0;
    s->braco_ref = 0.0f;
    s->sinal_bombeio   = 1;
    s->n_paradas       = 0;
    s->evento_inversao = 0;
    s->identificado    = 0;
    s->ident_pendente  = 0;
    s->count_chute0    = 0;
    s->ev_sinal        = 0;
    s->ev_n            = 0;
    s->confirmado      = 0;
    s->pico_melhor     = 0.0f;
    s->n_estag         = 0;
    Swing_Reiniciar(s);
}

void Swing_Reiniciar(SwingUp_t *s)
{
    s->fase        = s->med_tentada ? SW_BOMBEANDO : SW_CHUTE;
    s->t_fase      = 0.0f;
    s->t_parado    = 0.0f;
    s->sinal_thd   = 0;
    s->virou       = 0;
    s->pico_meia   = 0.0f;
    s->pico_ultimo = 0.0f;
    s->n_cruz      = 0;
    s->amp_soma    = 0.0f;
    s->amp_n       = 0;
    s->n_queda     = 0;
    s->ep_ant      = -1.0f;
    s->trab        = 0.0f;
    s->ep_trab     = -1.0f;
}

static void inverter_bombeio(SwingUp_t *s)
{
    s->sinal_bombeio   = (int8_t)(-s->sinal_bombeio);
    s->n_queda         = 0;
    s->n_paradas       = 0;
    s->ep_ant          = -1.0f;
    s->margem          = 0.0f;
    s->pico_melhor     = 0.0f;
    s->n_estag         = 0;
    s->evento_inversao = 1;
}

void Swing_AtualizarSensores(SwingUp_t *s, long count)
{
    const float cpr = s->p.cpr;
    if (!s->sensor_ok) {
        s->count_ant = count;
        s->sensor_ok = 1;
    }

    /* Velocidade da haste: diferença de contagens + passa-baixas
     * (a quantização de 0.15° em 1 ms sozinha daria degraus de 2.6 rad/s) */
    float bruto = (float)(count - s->count_ant) * (DOIS_PI_F / cpr) / s->p.dt;
    s->count_ant = count;
    s->thd += s->alfa_vel * (bruto - s->thd);

    /* Ângulo dentro de uma volta (evita perder precisão se a haste girar várias vezes) */
    long m = count % (long)cpr;
    if (m < 0) m += (long)cpr;
    s->th_w_ant = s->th_w;
    s->th   = (float)m * (DOIS_PI_F / cpr);
    s->th_w = (s->th > PI_F) ? (s->th - DOIS_PI_F) : s->th;

    float r = s->thd / s->w0;
    s->energia = 0.5f * r * r + (1.0f - cosf(s->th));

    /* Detecta a inversão de sentido da haste (fim de cada meia-oscilação) */
    s->virou = 0;
    float abs_thw = fabsf(s->th_w);
    if (abs_thw > s->pico_meia) s->pico_meia = abs_thw;
    if ((s->thd >  HIST_VEL && s->sinal_thd <= 0) ||
        (s->thd < -HIST_VEL && s->sinal_thd >= 0)) {
        if (s->sinal_thd != 0) {
            s->virou       = 1;
            s->pico_ultimo = s->pico_meia;
        }
        s->sinal_thd = (s->thd > 0.0f) ? 1 : -1;
        s->pico_meia = abs_thw;
    }
}

float Swing_Calcular(SwingUp_t *s, float braco, float vel, int8_t sinal_motor)
{
    const SwingParams_t *p = &s->p;
    float a = 0.0f;
    s->t_fase += p->dt;

    switch (s->fase) {
    case SW_CHUTE: {
        /* Tranco: braço vai rápido +chute (a partir de onde está) e volta. */
        if (s->t_fase <= p->dt * 1.5f) {
            s->braco_ref      = braco;
            s->count_chute0   = s->count_ant;
            s->ident_pendente = (s->energia < 0.003f && fabsf(s->thd) < 0.3f);  /* haste parada? */
        }
        /* Identificação: braço acelerando para +; a haste é empurrada para um lado
         * conforme o acoplamento físico. Bombeio certo => acel = -sinal_fisico * u */
        if (s->ident_pendente && s->t_fase >= T_IDENT) {
            long d = s->count_ant - s->count_chute0;
            s->ident_pendente = 0;
            if (d >= CONT_IDENT_MIN || d <= -CONT_IDENT_MIN) {
                int8_t sinal_fisico = (d < 0) ? 1 : -1;
                int8_t novo = (int8_t)(sinal_fisico * sinal_motor);
                if (novo != s->sinal_bombeio) s->evento_inversao = 1;
                s->sinal_bombeio = novo;
                s->identificado  = 1;
                s->n_queda = 0;
                s->n_paradas = 0;
            }
        }
        float ref = s->braco_ref + ((s->t_fase < p->chute_t) ? p->chute_rad : 0.0f);
        a = servo_braco(ref, braco, vel, 40.0f, p->acel_max);
        if (s->t_fase >= p->chute_t + 0.25f) {
            s->t_fase = 0.0f;
            if (!s->med_tentada) {
                s->med_tentada = 1;
                s->fase     = SW_MEDINDO;
                s->n_cruz   = 0;
                s->amp_soma = 0.0f;
                s->amp_n    = 0;
            } else {
                s->fase     = SW_BOMBEANDO;
                s->t_parado = 0.0f;
            }
        }
        break;
    }

    case SW_MEDINDO: {
        /* Braço parado onde o tranco começou; cronometra a haste oscilando livre */
        a = servo_braco(s->braco_ref, braco, vel, 12.0f, 60.0f);

        if (s->virou) {
            if (s->pico_ultimo > AMP_MEDIR_MAX + AMP_MEDIR_CRESC * s->t_fase) {
                s->n_cruz = 0;              /* ainda balançando muito: espera o atrito diminuir */
            } else if (s->n_cruz > 0) {
                s->amp_soma += s->pico_ultimo;
                s->amp_n++;
            }
        }
        /* passagem pelo ponto mais baixo no sentido positivo */
        if (s->th_w_ant < 0.0f && s->th_w >= 0.0f && fabsf(s->th_w) < 1.0f) {
            if (s->n_cruz == 0) {
                s->t_cruz0  = s->t_fase;
                s->amp_soma = 0.0f;
                s->amp_n    = 0;
            }
            s->t_cruz_ult = s->t_fase;
            s->n_cruz++;
        }

        if (s->n_cruz >= N_CRUZ_MEDIR) {
            float T   = (s->t_cruz_ult - s->t_cruz0) / (float)(s->n_cruz - 1);
            float amp = (s->amp_n > 0) ? (s->amp_soma / (float)s->amp_n) : 0.0f;
            if (amp > 2.6f) amp = 2.6f;
            /* Período de grande amplitude: T = T0 / AGM(1, cos(amp/2)) */
            float ag = 1.0f, bg = cosf(0.5f * amp);
            for (int i = 0; i < 6; i++) {
                float an = 0.5f * (ag + bg);
                bg = sqrtf(ag * bg);
                ag = an;
            }
            float w = DOIS_PI_F / (T * ag);
            if (w > 3.0f && w < 15.0f) {
                s->w0        = w;
                s->w0_medido = 1;
            }
            s->fase     = SW_BOMBEANDO;
            s->t_fase   = 0.0f;
            s->t_parado = 0.0f;
        } else if (s->t_fase > T_MEDICAO_MAX) {
            s->fase   = SW_BOMBEANDO;   /* segue com w0_padrao */
            s->t_fase = 0.0f;
        }
        break;
    }

    case SW_BOMBEANDO:
    default: {
        /* Margem de energia adaptativa: se a meia-oscilação terminou sem chegar
         * na zona de captura (atrito, w0 impreciso...), pede um pouco mais. */
        if (s->virou && s->pico_ultimo < (PI_F - p->zona_captura_rad)) {
            s->margem += p->margem_passo;
            if (s->margem > p->margem_max) s->margem = p->margem_max;
        }

        /* Verifica se o bombeio está mesmo subindo a haste: abaixo de ~70° o
         * bombeio certo SEMPRE aumenta o pico a cada meia-oscilação; o errado
         * faz a oscilação encolher rapidamente. */
        if (s->virou && !s->identificado && !s->confirmado) {
            float pico = s->pico_ultimo;                 /* não depende de w0 */
            if (pico > 0.5f) s->n_paradas = 0;           /* passou de ~30°: está funcionando */
            if (s->ep_ant >= PICO_MIN_RAD && s->ep_ant < PICO_AVAL_MAX) {
                if (pico < PICO_COLAPSO) {
                    s->n_queda = N_QUEDA_INV;            /* despencou: errado com certeza */
                } else if (pico < 0.75f * s->ep_ant) {
                    s->n_queda++;
                } else if (pico > s->ep_ant) {
                    s->n_queda = 0;
                }
            }
            s->ep_ant = pico;
            if (s->n_queda >= N_QUEDA_INV) inverter_bombeio(s);
        }
        float e_ref = 2.0f * (1.0f + s->margem);
        if (s->energia > e_ref + 0.6f) {          /* passou do topo com sobra: alivia */
            s->margem -= p->margem_passo * p->dt * 10.0f;
            if (s->margem < 0.0f) s->margem = 0.0f;
        }

        if (s->virou) {
            /* Evidência pelo ganho de energia: atrito só tira energia, então se a
             * meia-oscilação GANHOU altura, o braço fez trabalho positivo nela:
             * dE ~ -sinal_fisico * acel*thd*cos(th)  =>  sinal_fisico = -sinal(trab) */
            float ep = 1.0f - cosf(s->pico_ultimo);
            if (s->ep_trab >= 0.0f && !s->identificado && fabsf(s->trab) > TRAB_MIN &&
                (ep - s->ep_trab) > GANHO_MIN && ep < 1.5f) {
                int8_t sf = (s->trab > 0.0f) ? -1 : 1;
                if (sf == s->ev_sinal) s->ev_n++;
                else { s->ev_sinal = sf; s->ev_n = 1; }
                if (s->ev_n >= N_EVID) {
                    int8_t novo = (int8_t)(sf * sinal_motor);
                    if (novo != s->sinal_bombeio) {
                        s->evento_inversao = 1;
                        s->n_queda   = 0;
                        s->n_paradas = 0;
                    }
                    s->sinal_bombeio = novo;
                    s->confirmado    = 1;
                }
            }
            s->ep_trab = ep;
            s->trab    = 0.0f;

            /* Estagnação: picos sem crescer (preso balançando baixo) => outro sentido */
            if (!s->identificado && !s->confirmado && s->pico_ultimo < PICO_AVAL_MAX) {
                if (s->pico_ultimo > s->pico_melhor + 0.02f) {
                    s->pico_melhor = s->pico_ultimo;
                    s->n_estag     = 0;
                } else if (++s->n_estag >= N_ESTAG_INV) {
                    inverter_bombeio(s);
                }
            }
        }

        float gE = limitar(p->ganho_energia * (e_ref - s->energia), -1.0f, 1.0f);
        float u  = (float)s->sinal_bombeio * p->acel_bombeio * gE * tanhf(s->thd * cosf(s->th) / p->s0);

        a = -(float)sinal_motor * u - p->kp_braco * braco - p->kd_braco * vel;
        a = guarda_braco(p, a, braco, vel);

        /* Haste parada embaixo (sem velocidade não há o que bombear): novo tranco */
        s->t_parado = (s->energia < E_PARADO) ? (s->t_parado + p->dt) : 0.0f;
        if (s->t_parado > T_PARADO_MAX) {
            s->fase   = SW_CHUTE;
            s->t_fase = 0.0f;
            /* parou logo depois de uma oscilação de verdade: o bombeio a matou */
            if (!s->identificado && !s->confirmado &&
                (s->ep_ant >= PICO_MIN_RAD || ++s->n_paradas >= N_PARADAS_INV)) inverter_bombeio(s);
            s->ep_ant  = -1.0f;
            s->n_queda = 0;
            s->trab    = 0.0f;
            s->ep_trab = -1.0f;
        }
        break;
    }
    }

    a = limitar(a, -p->acel_max, p->acel_max);
    if (s->fase == SW_BOMBEANDO) {
        s->trab += a * s->thd * cosf(s->th) * p->dt;   /* "trabalho" do braço na meia-oscilação */
    }
    return a;
}

const char *Swing_NomeFase(const SwingUp_t *s)
{
    switch (s->fase) {
    case SW_CHUTE:     return "CHUTE";
    case SW_MEDINDO:   return "MEDINDO";
    case SW_BOMBEANDO: return "BOMBEANDO";
    default:           return "?";
    }
}
