#include <Arduino.h>
#include "pid.h"
#include "swingup.h"
#include <math.h>
#include "soc/gpio_struct.h"
#include "soc/gpio_reg.h"

/* ============================================================
 * PARÂMETROS DO SISTEMA (Calculados via LGR para m=8.5g, l=29cm)
 * ============================================================ */
#define K_LGR            0.01039f // Ganho K = Kd do LGR (positivo)
#define Z_LGR            4.5371f   // Zero duplo s = -z do LGR (rad/s)
#define DT               0.001f    // 1 ms → 1 kHz
#define TORQUE_MAX_NM    0.14f     // Torque máximo via A4988 (bipolar)
#define SETPOINT_GRAUS   180.0f    // Topo = 180° (haste para baixo no boot = 0°)
#define ZONA_ATIVA_GRAUS 30.0f     // Equilibrando: desiste (queda) se a haste sair de ±30° do topo
                                   // Modo manual: o PID assume quando você levanta a haste até ±30°

/* Parâmetros físicos para conversão Torque -> Aceleração -> Freq STEP */
#define M_MASSA          0.0085f   // 8.5 g -> kg
#define L_COMP           0.29f     // 29 cm -> m
#define J_INERCIA        (M_MASSA * L_COMP * L_COMP) // 0.00071485 kg.m^2
#define PASSOS_POR_VOLTA 3200.0f   // 200 passos * 16 microsteps (A4988)
#define ACEL_MAX_BRACO   (TORQUE_MAX_NM / J_INERCIA) // ~196 rad/s^2 (mesmo limite já usado no equilíbrio)

/* Realimentação suave do braço do motor (evita que o braço fuja e enrole o cabo) */
#define K_POS_BRACO      0.0025f   // N.m/rad de retorno suave para o centro do braço
#define K_VEL_BRACO      0.0012f   // N.m/(rad/s) de amortecimento da velocidade do braço
#define LIMITE_BRACO_DEG 130.0f    // Desarma por segurança se o braço girar mais de ±130° do CENTRO (protege o cabo)

/* ============================================================
 * SWING-UP AUTOMÁTICO (o braço balança a haste até ela subir sozinha)
 * ============================================================
 * Comando 'u' no Serial (ou segurar o botão BOOT por mais de 3 s) liga/desliga.
 * IMPORTANTE: ligue a placa (ou use 'c') com a haste PARADA embaixo e o braço
 * no CENTRO do cabo. Essa posição vira a referência 0° da haste e do braço.
 */
#define SWING_AUTO_NO_BOOT      0       // 1 = começa o swing-up sozinho logo após ligar
#define SWING_ATRASO_BOOT_MS    3000    // espera antes de começar (se SWING_AUTO_NO_BOOT = 1)

#define PERIODO_NATURAL_S       1.08f   // Período das pequenas oscilações da haste. Só é usado se a
                                        // medição automática falhar (ela roda no 1º swing após ligar).
#define SWING_ACEL_BOMBEIO      20.0f   // rad/s^2 - aceleração do braço para bombear energia
#define SWING_GANHO_ENERGIA     3.0f    // reduz o bombeio suavemente quando a energia chega no alvo
#define SWING_S0                0.5f    // rad/s - suavização de sinal(thd*cos th)
#define SWING_KP_BRACO          2.0f    // 1/s^2 - mola que puxa o braço para o centro durante o swing
#define SWING_KD_BRACO          2.0f    // 1/s   - amortecimento do braço durante o swing
#define SWING_LIMITE_BRACO_DEG  80.0f   // o braço freia para nunca passar disso durante o swing
#define SWING_ACEL_FREIO        120.0f  // rad/s^2 - frenagem usada pela guarda do braço
#define SWING_MARGEM_PASSO      0.0075f // +0.75% de energia alvo a cada meia-oscilação que não chega ao topo
#define SWING_MARGEM_MAX        0.30f
#define SWING_CHUTE_GRAUS       20.0f   // tranco inicial do braço quando a haste está parada
#define SWING_CHUTE_TEMPO_S     0.12f
#define VEL_HASTE_FILT_HZ       25.0f   // filtro da velocidade da haste (encoder)

#define ZONA_CAPTURA_GRAUS      8.0f    // no swing, o PID assume quando a haste chega a ±8° do topo...
#define VEL_CAPTURA_RAD_S       2.0f    // ...com velocidade abaixo disso (chegada suave)

/* Volta lenta do braço ao centro (comando 'u' depois de uma parada por limite do cabo) */
#define RETORNO_ACEL_MAX        10.0f   // rad/s^2
#define RETORNO_WN              3.0f    // rad/s

/* ============================================================
 * PINAGEM ESP32 (DevKit 30 pinos)
 * ============================================================ */
#define PIN_ENC_A     32   // fio VERDE do encoder  (pull-up p/ 3V3!)
#define PIN_ENC_B     33   // fio BRANCO do encoder (pull-up p/ 3V3!)
#define PIN_STEP      25   // driver STEP   (use GPIO < 32)
#define PIN_DIR       26   // driver DIR    (use GPIO < 32)
#define PIN_EN        27   // driver ENABLE (ativo em nível baixo). -1 se EN estiver fixo no GND
#define PIN_BTN_BOOT   0   // Botão físico BOOT da placa ESP32 (atalho p/ zerar encoder sem precisar do Serial)

#define PPR            600      // Pulsos por volta do encoder
#define CPR            (PPR * 4)// 2400 contagens por volta em quadratura x4
#define ENC_INVERT     0        // Mude para 1 se 90° e 270° estiverem trocados
#define DIR_INVERT     0        // Mude para 1 se o motor girar para o lado oposto da queda!

/* Gerador de passos DDS + amostragem do encoder a 20 kHz (50 us)
 * Leitura do encoder no próprio timer evita flood de interrupções de GPIO e libera a Serial!
 */
#define STEP_ISR_HZ    20000UL
#define FREQ_MAX_HZ    8000.0f  // Até 8 kHz de microstep sem perder passo

/* ============================================================
 * VARIÁVEIS GLOBAIS E HARDWARE TIMER (20 kHz Step + 1 kHz Controle)
 * ============================================================ */
volatile bool    flag_1khz     = false;
volatile long    encoder_count = 0;
volatile uint8_t enc_state     = 0;

/* Acumulador de fase DDS do gerador de passos (20 kHz) */
volatile int32_t g_rate_half   = 0;
volatile long    g_step_pos    = 0;
static uint32_t  s_phase       = 0;
static bool      s_pulse_high  = false;
static bool      s_pending     = false;
static int8_t    s_dir_cur     = 1;
static uint8_t   s_div_1khz    = 0;

static float     vel_cmd_rad_s = 0.0f; // Velocidade angular integrada p/ o A4988
static long      step_zero     = 0;    // Posição do braço quando o equilíbrio armou (referência do PID)
static long      step_centro   = 0;    // Centro do cabo (boot / 'c' / 'a'): referência do limite e do swing

/* ============================================================
 * MÁQUINA DE ESTADOS
 * ============================================================ */
typedef enum {
    MODO_PARADO = 0,   // motor parado (modo manual: espera você levantar a haste)
    MODO_SWING,        // balançando a haste para subir
    MODO_EQUILIBRIO,   // PID do LGR segurando a haste no topo
    MODO_RETORNO,      // levando o braço devagar de volta ao centro
    MODO_FALHA         // parou por segurança (limite do cabo)
} Modo_t;

static Modo_t     g_modo       = MODO_PARADO;
static bool       g_auto_swing = false;
static SwingUp_t  g_swing;
static bool       g_telemetria = false;   // 't' liga / 'q' desliga: linhas CSV "T,..." a 50 Hz p/ a interface

static const char *nome_modo(Modo_t m) {
    switch (m) {
        case MODO_PARADO:     return "PARADO";
        case MODO_SWING:      return "SWING";
        case MODO_EQUILIBRIO: return "EQUIL";
        case MODO_RETORNO:    return "RETORNO";
        case MODO_FALHA:      return "FALHA";
        default:              return "?";
    }
}

/* ============================================================
 * BUFFER DE LOG EM RAM (Grava 200 Hz enquanto equilibra e despeja com o motor parado)
 * ============================================================ */
#define LOG_MAX_SAMPLES 1000   // Até 5 segundos a 200 Hz (a cada 5 ms)
typedef struct {
    uint16_t t_ms;
    float    ang;
    float    erro;
    float    tau_pid;
    float    tau_total;
    float    step_hz;
    float    braco_deg;
} LogSample_t;

static LogSample_t g_log_buf[LOG_MAX_SAMPLES];
static int         g_log_count   = 0;
static int         g_log_div     = 0;
static uint32_t    g_log_t0_ms   = 0;
static bool        g_dump_pending = false;
static const char *g_dump_motivo  = "";

void imprimir_relatorio_queda(const char *motivo) {
    if (g_log_count <= 0) return;
    Serial.println("\n================ INICIO LOG.TXT (COPIE OU SALVE) ================");
    Serial.printf("# Motivo da parada: %s | Amostras gravadas: %d (dt = 5 ms)\n", motivo, g_log_count);
    Serial.printf("# K_LGR=%.6f | Z_LGR=%.4f | Kp=%.4f | Ki=%.4f | Kd=%.4f\n",
                  K_LGR, Z_LGR, 2.0f * K_LGR * Z_LGR, K_LGR * Z_LGR * Z_LGR, K_LGR);
    Serial.println("t_ms,ang_deg,erro_deg,tau_pid_nm,tau_total_nm,step_hz,braco_deg");
    for (int i = 0; i < g_log_count; i++) {
        Serial.printf("%u,%.2f,%.2f,%.4f,%.4f,%.1f,%.1f\n",
                      g_log_buf[i].t_ms,
                      g_log_buf[i].ang,
                      g_log_buf[i].erro,
                      g_log_buf[i].tau_pid,
                      g_log_buf[i].tau_total,
                      g_log_buf[i].step_hz,
                      g_log_buf[i].braco_deg);
    }
    Serial.println("================ FIM LOG.TXT ================\n");
}

hw_timer_t * timer = NULL;

#define STEP_MASK (1UL << PIN_STEP)
#define DIR_MASK  (1UL << PIN_DIR)

/* Tabela de transição de estados de quadratura (rejeita bounce/ruído em pino único) */
static const int8_t DRAM_ATTR QUAD_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

/* Leitura ultra-rápida dos GPIOs 32 e 33 direto do registrador in1 */
static inline uint8_t IRAM_ATTR ler_ab_fast(void) {
    uint32_t in1 = GPIO.in1.val;
    uint8_t a = (in1 >> (PIN_ENC_A - 32)) & 1U;
    uint8_t b = (in1 >> (PIN_ENC_B - 32)) & 1U;
    return (a << 1) | b;
}

/* ISR de 20 kHz (a cada 50 us):
 * 1) Amostra o encoder em quadratura com filtro natural de 50 us (nunca trava a Serial por ruído)
 * 2) Gera pulsos STEP precisos por acumulador de fase (DDS)
 * 3) Divide por 20 para disparar a malha de controle a exatos 1 kHz (1 ms)
 */
void IRAM_ATTR onTimer20kHz(void) {
    // 1) Decodifica o encoder diretamente a 20 kHz (sem flood de attachInterrupt)
    uint8_t ab = ler_ab_fast();
    enc_state = ((enc_state << 2) | ab) & 0x0F;
    int8_t delta = QUAD_TABLE[enc_state];
#if ENC_INVERT
    encoder_count -= delta;
#else
    encoder_count += delta;
#endif

    // 2) Divisor 20 kHz -> 1 kHz para o PID
    if (++s_div_1khz >= 20) {
        s_div_1khz = 0;
        flag_1khz = true;
    }

    // 3) Baixa o pino STEP após 50 us se estava em nível alto
    bool just_cleared = false;
    if (s_pulse_high) {
        REG_WRITE(GPIO_OUT_W1TC_REG, STEP_MASK);
        s_pulse_high = false;
        just_cleared = true;
    }

    int32_t r    = g_rate_half;
    int8_t  dir  = (r >= 0) ? 1 : -1;
    uint32_t inc = (uint32_t)(r >= 0 ? r : -r) << 1;

    // 4) Se mudou de sentido, atualiza DIR e aguarda 1 tick (50 us) de setup time do A4988
    if (dir != s_dir_cur) {
        bool level = (dir > 0) ^ (DIR_INVERT != 0);
        REG_WRITE(level ? GPIO_OUT_W1TS_REG : GPIO_OUT_W1TC_REG, DIR_MASK);
        s_dir_cur = dir;
        s_pending = false;
        return;
    }

    // 5) Acumulador de fase DDS: quando transborda 32 bits, agenda 1 pulso STEP
    uint32_t old = s_phase;
    s_phase += inc;
    if (s_phase < old) s_pending = true;

    if (s_pending && !just_cleared) {
        REG_WRITE(GPIO_OUT_W1TS_REG, STEP_MASK);
        s_pulse_high = true;
        s_pending    = false;
        g_step_pos  += s_dir_cur;
    }
}

/* ============================================================
 * FUNÇÕES DE HARDWARE (Encoder 0°..360° e Comando DDS de Passos)
 * ============================================================ */
float encoder_ler_graus(void) {
    long count = encoder_count;
    long mod = count % CPR;
    if (mod < 0) mod += CPR;
    return (float)mod * 360.0f / (float)CPR;
}

// Ângulo do braço do motor (em radianos) relativo a uma posição de referência em passos
float braco_ler_rad(long ref_passos) {
    long passos = g_step_pos - ref_passos;
    return (float)passos * (2.0f * PI_F / PASSOS_POR_VOLTA);
}

// Define a frequência de passos (com sinal) no gerador DDS de 20 kHz
void motor_set_freq_signed_hz(float freq_signed_hz) {
    if (freq_signed_hz >  FREQ_MAX_HZ) freq_signed_hz =  FREQ_MAX_HZ;
    if (freq_signed_hz < -FREQ_MAX_HZ) freq_signed_hz = -FREQ_MAX_HZ;
    g_rate_half = (int32_t)(freq_signed_hz * (2147483648.0f / (float)STEP_ISR_HZ));
}

void motor_enable(bool on) {
    if (PIN_EN >= 0) {
        digitalWrite(PIN_EN, on ? LOW : HIGH); // LOW = Habilita A4988
    }
}

static void motor_parar(void) {
    vel_cmd_rad_s = 0.0f;
    motor_set_freq_signed_hz(0.0f);
}

/* ============================================================
 * LÓGICA DE CONTROLE: Aceleração do braço -> Velocidade -> Freq STEP
 * ============================================================ */
static void aplicar_aceleracao_braco(float alpha_cmd) {
    if (alpha_cmd >  ACEL_MAX_BRACO) alpha_cmd =  ACEL_MAX_BRACO;
    if (alpha_cmd < -ACEL_MAX_BRACO) alpha_cmd = -ACEL_MAX_BRACO;

    // Integra a aceleração angular comandada para obter velocidade
    vel_cmd_rad_s += alpha_cmd * DT;

    // Converte velocidade angular (rad/s) em frequência de pulsos STEP (Hz)
    float freq_signed = vel_cmd_rad_s * (PASSOS_POR_VOLTA / (2.0f * PI_F));

    // Anti-windup da velocidade máxima de passos do driver A4988
    if (freq_signed >  FREQ_MAX_HZ) {
        freq_signed =  FREQ_MAX_HZ;
        vel_cmd_rad_s = freq_signed * (2.0f * PI_F / PASSOS_POR_VOLTA);
    } else if (freq_signed < -FREQ_MAX_HZ) {
        freq_signed = -FREQ_MAX_HZ;
        vel_cmd_rad_s = freq_signed * (2.0f * PI_F / PASSOS_POR_VOLTA);
    }

    // Atualiza o gerador DDS de 20 kHz sem engasgar o pulso atual
    motor_set_freq_signed_hz(freq_signed);
}

// Torque "virtual" do PID -> aceleração do braço (alpha = tau / J), como antes
static void aplicar_torque_motor(float tau_nm) {
    if (tau_nm >  TORQUE_MAX_NM) tau_nm =  TORQUE_MAX_NM;
    if (tau_nm < -TORQUE_MAX_NM) tau_nm = -TORQUE_MAX_NM;
    aplicar_aceleracao_braco(tau_nm / J_INERCIA);
}

static void swing_init_params(void) {
    SwingParams_t p;
    p.dt               = DT;
    p.cpr              = (float)CPR;
    p.w0_padrao        = 2.0f * PI_F / PERIODO_NATURAL_S;
    p.acel_bombeio     = SWING_ACEL_BOMBEIO;
    p.ganho_energia    = SWING_GANHO_ENERGIA;
    p.s0               = SWING_S0;
    p.kp_braco         = SWING_KP_BRACO;
    p.kd_braco         = SWING_KD_BRACO;
    p.lim_braco_rad    = SWING_LIMITE_BRACO_DEG * DEG2RAD;
    p.acel_freio       = SWING_ACEL_FREIO;
    p.acel_max         = ACEL_MAX_BRACO;
    p.zona_captura_rad = ZONA_CAPTURA_GRAUS * DEG2RAD;
    p.margem_passo     = SWING_MARGEM_PASSO;
    p.margem_max       = SWING_MARGEM_MAX;
    p.chute_rad        = SWING_CHUTE_GRAUS * DEG2RAD;
    p.chute_t          = SWING_CHUTE_TEMPO_S;
    p.vel_filt_hz      = VEL_HASTE_FILT_HZ;
    Swing_Init(&g_swing, &p);
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("==================================================");
    Serial.println("BOOT OK - Controle 2 ENG (DDS 20kHz + Deriv Filt) + SWING-UP");
    Serial.printf("PINOS -> STEP:%d DIR:%d EN:%d ENC_A:%d ENC_B:%d\n",
                  PIN_STEP, PIN_DIR, PIN_EN, PIN_ENC_A, PIN_ENC_B);
    Serial.println("Comandos no Monitor Serial (115200 baud):");
    Serial.println("  'u' -> LIGA/DESLIGA o swing-up automatico (sobe sozinho e equilibra)");
    Serial.println("  'a' -> AUTO-CALIBRAR sentido do motor + zerar em 0°");
    Serial.println("  'c' -> Zerar encoder (haste parada embaixo) e centro do braco");
    Serial.println("  'i' -> Inverter sentido do motor manualmente (+1 / -1)");
    Serial.println("  'l' -> Imprimir o LOG do ultimo equilibrio (com o motor parado)");
    Serial.println("  '+' / '-' -> Aumenta / diminui a forca do bombeio (SWING_ACEL_BOMBEIO) ao vivo");
    Serial.println("  'b' -> Inverte manualmente o sentido do bombeio do swing");
    Serial.println("  't' / 'q' -> Liga / desliga a telemetria rapida (interface_pendulo.html)");
    Serial.println("  'g' -> Envia os ganhos atuais (linha G,...)");
    Serial.println("Botao BOOT: clique = 'c' | segurar 1-3 s = 'a' | segurar > 3 s = 'u'");
    Serial.println("==================================================");

    // 1. Configuração dos pinos do Driver
    pinMode(PIN_STEP, OUTPUT);
    digitalWrite(PIN_STEP, LOW);
    pinMode(PIN_DIR, OUTPUT);
    digitalWrite(PIN_DIR, DIR_INVERT ? LOW : HIGH);
    if (PIN_EN >= 0) {
        pinMode(PIN_EN, OUTPUT);
    }
    motor_enable(true);

    // 2. Configuração do Encoder e botão físico BOOT (GPIO 0)
    pinMode(PIN_ENC_A, INPUT_PULLUP);
    pinMode(PIN_ENC_B, INPUT_PULLUP);
    pinMode(PIN_BTN_BOOT, INPUT_PULLUP);
    enc_state = ler_ab_fast();

    swing_init_params();

    // 3. Configuração do Timer de 20 kHz (50 us) p/ Encoder + DDS STEP + 1 kHz Controle
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
    timer = timerBegin(1000000);                         // 1 MHz (1 tick = 1 us)
    timerAttachInterrupt(timer, &onTimer20kHz);
    timerAlarm(timer, 1000000UL / STEP_ISR_HZ, true, 0); // 50 us = 20 kHz
#else
    timer = timerBegin(0, 80, true);                     // 80MHz/80 = 1MHz
    timerAttachInterrupt(timer, &onTimer20kHz, true);
    timerAlarmWrite(timer, 1000000UL / STEP_ISR_HZ, true);
    timerAlarmEnable(timer);
#endif
}

/* ============================================================
 * TRANSIÇÕES DE MODO
 * ============================================================ */
static void despejar_log_pendente(void) {
    // Só chame com o motor PARADO: imprimir 1000 linhas leva alguns segundos
    if (g_dump_pending) {
        g_dump_pending = false;
        imprimir_relatorio_queda(g_dump_motivo);
    }
}

static void entrar_parado(void) {
    motor_parar();
    g_modo = MODO_PARADO;
}

static void entrar_falha(const char *motivo) {
    motor_parar();
    g_auto_swing = false;
    g_modo = MODO_FALHA;
    Serial.printf("\n>>> [FALHA] %s. Motor parado. Envie 'u' para voltar o braco ao centro e tentar de novo.\n", motivo);
    if (g_log_count > 0) {
        g_dump_pending = true;
        g_dump_motivo  = motivo;
    }
    despejar_log_pendente();
}

static void entrar_swing(void) {
    Swing_Reiniciar(&g_swing);
    g_modo = MODO_SWING;
}

static void entrar_equilibrio(PID_t *pid, float ang, bool vindo_do_swing) {
    PID_Reset(pid);
    pid->erro_anterior  = (SETPOINT_GRAUS - ang) * DEG2RAD;
    pid->deriv_filtrada = -g_swing.thd;   // e = 180° - ang  =>  de/dt = -velocidade da haste
    if (!vindo_do_swing) {
        vel_cmd_rad_s = 0.0f;   // modo manual: braço estava parado
    }                           // vindo do swing: mantém a velocidade do braço (sem tranco)
    step_zero   = g_step_pos;
    g_log_count = 0;
    g_log_div   = 0;
    g_log_t0_ms = millis();
    g_modo      = MODO_EQUILIBRIO;
}

static void ligar_desligar_auto(void) {
    if (g_auto_swing) {
        g_auto_swing = false;
        entrar_parado();
        Serial.println("\n>>> [SWING] Swing-up automatico DESLIGADO. Motor parado.\n");
        despejar_log_pendente();
        return;
    }
    g_auto_swing = true;
    float braco_deg = braco_ler_rad(step_centro) * (180.0f / PI_F);
    if (fabsf(braco_deg) > 30.0f) {
        g_modo = MODO_RETORNO;   // braço longe do centro: volta devagar antes de balançar
        Serial.printf("\n>>> [SWING] LIGADO. Braco em %+.1f° -> voltando ao centro primeiro...\n\n", braco_deg);
    } else if (g_modo != MODO_EQUILIBRIO) {
        entrar_swing();
        Serial.println("\n>>> [SWING] Swing-up automatico LIGADO.\n");
    } else {
        Serial.println("\n>>> [SWING] LIGADO (ja equilibrando; se cair, ele sobe sozinho de novo).\n");
    }
}

void executar_autocalibracao(int8_t *sinal_motor) {
    g_auto_swing = false;
    entrar_parado();
    Serial.println("\n>>> [AUTO] Iniciando auto-calibracao (mantenha a haste solta embaixo)...");
    delay(300);
    long c0 = encoder_count;

    // Impulso curto positivo no motor
    motor_set_freq_signed_hz(800.0f);
    delay(80);
    motor_set_freq_signed_hz(0.0f);
    delay(40);
    long delta_enc = encoder_count - c0;

    // Volta o braço para a posição original
    motor_set_freq_signed_hz(-800.0f);
    delay(80);
    motor_set_freq_signed_hz(0.0f);

    if (delta_enc == 0) {
        Serial.println(">>> [AUTO] AVISO: Encoder nao contou durante o pulso! Verifique fios A/B.");
    } else {
        *sinal_motor = (delta_enc > 0) ? 1 : -1;
        Serial.printf(">>> [AUTO] Pulso gerou delta_enc = %+ld -> sinal_motor ajustado para %+d\n",
                      delta_enc, *sinal_motor);
    }

    Serial.println(">>> [AUTO] Aguardando 1.5s para a haste parar e zerar em 0.00°...");
    delay(1500);
    encoder_count = 0;
    step_zero     = g_step_pos;
    step_centro   = g_step_pos;
    swing_init_params();   // zerou o encoder: reinicia filtros (o período será medido de novo)
    Serial.printf(">>> [AUTO] Concluido! Encoder = 0.00°, sinal_motor = %+d. Levante ate 180° ou envie 'u'!\n\n",
                  *sinal_motor);
}

static void zerar_referencias(const char *origem) {
    g_auto_swing = false;
    entrar_parado();
    encoder_count = 0;
    step_centro   = g_step_pos;
    step_zero     = g_step_pos;
    swing_init_params();
    Serial.printf("\n>>> [%s] Encoder zerado (0.00 graus) e braco marcado como centro!\n\n", origem);
}

void loop() {
    static PID_t pid;
    static bool primeira_vez = true;
    static int8_t sinal_motor = -1; // Já inicia com o sentido invertido (-1) que funcionou nos micro-ajustes!
    static unsigned long btn_down_ms = 0;
    static bool btn_pressed = false;
    static float tau = 0.0f;

    if (primeira_vez) {
        PID_InitKz(&pid, K_LGR, Z_LGR, DT, TORQUE_MAX_NM);
        primeira_vez = false;
    }

#if SWING_AUTO_NO_BOOT
    static bool auto_boot_feito = false;
    if (!auto_boot_feito && millis() >= SWING_ATRASO_BOOT_MS) {
        auto_boot_feito = true;
        if (!g_auto_swing) ligar_desligar_auto();
    }
#endif

    // Atalho físico pelo botão BOOT (GPIO 0) na própria placa ESP32:
    // - Clique curto (< 1s): zera o encoder em 0.00° (igual ao comando 'c')
    // - Segurar 1 a 3 s: executa a auto-calibração completa (igual ao comando 'a')
    // - Segurar > 3 s: liga/desliga o swing-up automático (igual ao comando 'u')
    if (digitalRead(PIN_BTN_BOOT) == LOW) {
        if (!btn_pressed) {
            btn_pressed = true;
            btn_down_ms = millis();
        }
    } else if (btn_pressed) {
        unsigned long dur = millis() - btn_down_ms;
        btn_pressed = false;
        if (dur >= 3000) {
            ligar_desligar_auto();
        } else if (dur >= 1000) {
            executar_autocalibracao(&sinal_motor);
        } else if (dur >= 50) {
            zerar_referencias("BOTAO BOOT");
        }
    }

    // Comandos rápidos via Monitor Serial
    while (Serial.available() > 0) {
        char ch = (char)Serial.read();
        if (ch == 'c' || ch == 'C') {
            zerar_referencias("MANUAL");
        } else if (ch == 'i' || ch == 'I') {
            sinal_motor = -sinal_motor;
            Serial.printf("\n>>> [MANUAL] Sentido do motor invertido! sinal_motor = %+d\n\n", sinal_motor);
        } else if (ch == 'a' || ch == 'A') {
            executar_autocalibracao(&sinal_motor);
        } else if (ch == 'u' || ch == 'U') {
            ligar_desligar_auto();
        } else if (ch == '+' || ch == '=') {
            g_swing.p.acel_bombeio += 5.0f;
            if (g_swing.p.acel_bombeio > 80.0f) g_swing.p.acel_bombeio = 80.0f;
            Serial.printf("\n>>> [SWING] Forca do bombeio = %.0f rad/s^2\n\n", g_swing.p.acel_bombeio);
        } else if (ch == '-' || ch == '_') {
            g_swing.p.acel_bombeio -= 5.0f;
            if (g_swing.p.acel_bombeio < 5.0f) g_swing.p.acel_bombeio = 5.0f;
            Serial.printf("\n>>> [SWING] Forca do bombeio = %.0f rad/s^2\n\n", g_swing.p.acel_bombeio);
        } else if (ch == 't' || ch == 'T') {
            g_telemetria = true;
            Serial.println("TELEM,1");
        } else if (ch == 'q' || ch == 'Q') {
            g_telemetria = false;
            Serial.println("TELEM,0");
        } else if (ch == 'g' || ch == 'G') {
            // G,K_LGR,Z_LGR,Kp,Ki,Kd,K_POS_BRACO,K_VEL_BRACO,A_bombeio,kE,kp_sw,kd_sw,lim_sw_deg,
            //   zona_cap_deg,vel_cap,w0,w0_medido,margem,sinal_motor,sinal_bombeio,auto
            Serial.printf("G,%.6f,%.4f,%.5f,%.5f,%.5f,%.4f,%.4f,%.1f,%.2f,%.2f,%.2f,%.1f,%.1f,%.2f,%.3f,%d,%.3f,%d,%d,%d\n",
                          K_LGR, Z_LGR, pid.Kp, pid.Ki, pid.Kd, K_POS_BRACO, K_VEL_BRACO,
                          g_swing.p.acel_bombeio, g_swing.p.ganho_energia, g_swing.p.kp_braco, g_swing.p.kd_braco,
                          g_swing.p.lim_braco_rad * (180.0f / PI_F), ZONA_CAPTURA_GRAUS, VEL_CAPTURA_RAD_S,
                          g_swing.w0, g_swing.w0_medido, g_swing.margem,
                          sinal_motor, g_swing.sinal_bombeio, g_auto_swing ? 1 : 0);
        } else if (ch == 'b' || ch == 'B') {
            g_swing.sinal_bombeio = (int8_t)(-g_swing.sinal_bombeio);
            g_swing.identificado  = 0;   // deixa a correção automática reavaliar depois
            g_swing.confirmado    = 0;
            Serial.printf("\n>>> [SWING] Sentido do bombeio invertido manualmente: %+d\n\n", g_swing.sinal_bombeio);
        } else if (ch == 'l' || ch == 'L') {
            if (g_modo == MODO_PARADO || g_modo == MODO_FALHA) {
                imprimir_relatorio_queda("Solicitado manualmente via 'l'");
            } else {
                Serial.println("\n>>> [LOG] Motor em movimento: desligue com 'u' antes de pedir o log.\n");
            }
        }
    }

    if (flag_1khz) {
        flag_1khz = false;

        // 1) Leituras: haste em [0°, 360°) (0° = embaixo, 180° = topo), velocidade/energia e braço
        Swing_AtualizarSensores(&g_swing, encoder_count);
        float ang          = encoder_ler_graus();
        float erro_topo    = fabsf(SETPOINT_GRAUS - ang);
        float braco_centro = braco_ler_rad(step_centro);           // relativo ao centro do cabo
        float braco_deg    = braco_centro * (180.0f / PI_F);
        tau = 0.0f;

        // 2) Segurança do cabo: vale em qualquer modo com o motor ativo
        if ((g_modo == MODO_SWING || g_modo == MODO_EQUILIBRIO) &&
            fabsf(braco_deg) > LIMITE_BRACO_DEG) {
            entrar_falha("Braco atingiu limite do cabo (±130 deg do centro)");
        }

        switch (g_modo) {
        case MODO_PARADO:
        case MODO_FALHA:
            // Modo manual: o PID assume quando você leva a haste até ±30° do topo (como antes)
            if (g_modo == MODO_PARADO && erro_topo <= ZONA_ATIVA_GRAUS) {
                entrar_equilibrio(&pid, ang, false);
            }
            break;

        case MODO_RETORNO: {
            float a = RETORNO_WN * RETORNO_WN * (0.0f - braco_centro) - 2.0f * RETORNO_WN * vel_cmd_rad_s;
            if (a >  RETORNO_ACEL_MAX) a =  RETORNO_ACEL_MAX;
            if (a < -RETORNO_ACEL_MAX) a = -RETORNO_ACEL_MAX;
            aplicar_aceleracao_braco(a);
            if (fabsf(braco_deg) < 3.0f && fabsf(vel_cmd_rad_s) < 0.1f) {
                motor_parar();
                if (g_auto_swing) entrar_swing();
                else              g_modo = MODO_PARADO;
            }
            break;
        }

        case MODO_SWING:
            // 3a) Chegou perto do topo devagar? O PID do LGR assume.
            if (erro_topo <= ZONA_CAPTURA_GRAUS && fabsf(g_swing.thd) < VEL_CAPTURA_RAD_S) {
                entrar_equilibrio(&pid, ang, true);
                Serial.printf(">>> [SWING] Capturou no topo! (ang=%.1f°, braco=%+.1f°)\n", ang, braco_deg);
            } else {
                aplicar_aceleracao_braco(Swing_Calcular(&g_swing, braco_centro, vel_cmd_rad_s, sinal_motor));
                if (g_swing.evento_inversao) {
                    g_swing.evento_inversao = 0;
                    Serial.printf(">>> [SWING] Sentido do bombeio corrigido automaticamente -> %+d\n", g_swing.sinal_bombeio);
                }
            }
            break;

        case MODO_EQUILIBRIO:
            if (erro_topo > ZONA_ATIVA_GRAUS) {
                // Caiu: no automático volta a balançar; no manual para o motor e despeja o LOG
                g_dump_pending = (g_log_count > 0);
                g_dump_motivo  = "Haste saiu da zona do topo (±30 deg)";
                if (g_auto_swing) {
                    entrar_swing();
                    Serial.println(">>> [SWING] Caiu do topo -> balancando de novo (LOG guardado, 'u' para parar e ver)");
                } else {
                    entrar_parado();
                    despejar_log_pendente();
                }
                break;
            }
            {
                // 3b) PID do LGR (haste) + amortecimento suave do braço (evita fugir/enrolar cabo)
                float braco_rel = braco_ler_rad(step_zero);
                float tau_pid   = PID_Calcular(&pid, SETPOINT_GRAUS, ang);
                // Mesmo termo de antes (-K*braco - K*vel com sinal_motor = -1), agora coerente para os dois sinais
                float tau_braco = (float)sinal_motor * (K_POS_BRACO * braco_rel + K_VEL_BRACO * vel_cmd_rad_s);
                tau = (tau_pid + tau_braco) * (float)sinal_motor;

                // 4) Aplica no gerador de passos DDS de 20 kHz
                aplicar_torque_motor(tau);

                // 5) Grava amostra no buffer de debug a cada 5 ms (200 Hz)
                if (++g_log_div >= 5 && g_log_count < LOG_MAX_SAMPLES) {
                    g_log_div = 0;
                    float freq_step = vel_cmd_rad_s * (PASSOS_POR_VOLTA / (2.0f * PI_F));
                    g_log_buf[g_log_count].t_ms      = (uint16_t)(millis() - g_log_t0_ms);
                    g_log_buf[g_log_count].ang       = ang;
                    g_log_buf[g_log_count].erro      = SETPOINT_GRAUS - ang;
                    g_log_buf[g_log_count].tau_pid   = tau_pid;
                    g_log_buf[g_log_count].tau_total = tau;
                    g_log_buf[g_log_count].step_hz   = freq_step;
                    g_log_buf[g_log_count].braco_deg = braco_deg;
                    g_log_count++;
                }
            }
            break;
        }

        // Telemetria rápida p/ a interface (50 Hz):
        // T,t_ms,modo,fase,ang_deg,braco_deg,thd_rad_s,E,tau_nm,step_hz,pico_deg,sinal_bombeio
        static unsigned long last_tel = 0;
        if (g_telemetria && millis() - last_tel >= 20) {
            last_tel = millis();
            Serial.printf("T,%lu,%d,%d,%.2f,%.1f,%.2f,%.3f,%.4f,%.0f,%.1f,%d\n",
                          last_tel, (int)g_modo, (int)g_swing.fase, ang, braco_deg, g_swing.thd,
                          g_swing.energia, tau, vel_cmd_rad_s * (PASSOS_POR_VOLTA / (2.0f * PI_F)),
                          g_swing.pico_ultimo * (180.0f / PI_F), g_swing.sinal_bombeio);
        }

        // Log a cada 100ms no Serial Monitor (115200 baud) — desligado durante a telemetria
        static unsigned long last_log = 0;
        if (!g_telemetria && millis() - last_log >= 100) {
            last_log = millis();
            float freq_step = vel_cmd_rad_s * (PASSOS_POR_VOLTA / (2.0f * PI_F));
            Serial.printf("%-7s%s | sin=%+d | bomb=%+d%s A=%2.0f | ang=%6.2f° | pico=%5.1f° | braco=%+6.1f° | E=%4.2f | w0=%4.2f%s | tau=%+6.4f | step=%+7.1f Hz\n",
                          nome_modo(g_modo),
                          (g_modo == MODO_SWING) ? (g_swing.fase == SW_CHUTE ? "/CHU" :
                                                    g_swing.fase == SW_MEDINDO ? "/MED" : "/BOM") : "    ",
                          sinal_motor, g_swing.sinal_bombeio,
                          g_swing.identificado ? "i" : (g_swing.confirmado ? "c" : "?"),
                          g_swing.p.acel_bombeio, ang, g_swing.pico_ultimo * (180.0f / PI_F),
                          braco_deg, g_swing.energia, g_swing.w0,
                          g_swing.w0_medido ? "m" : "p", tau, freq_step);
        }
    }
}
