# Firmware do pêndulo invertido rotacional (ESP32)

Pêndulo invertido rotacional (tipo Furuta) com braço acionado por motor de passo (A4988) e haste medida por encoder de quadratura. O firmware:

- **sobe a haste sozinho** (swing-up por bombeamento de energia, método de Åström–Furuta);
- **equilibra no topo** com o PID projetado pelo lugar das raízes (LGR);
- se a haste cair, **volta a balançar e sobe de novo**;
- protege o cabo limitando o giro do braço;
- envia telemetria para uma **interface no navegador** com gráficos e ganhos.

Não há rede neural: tudo é controle clássico, com parâmetros de significado físico.

---

## Estrutura

| Arquivo | Função |
|---|---|
| `esp32_pid_eng/esp32_pid_eng.ino` | Programa principal: hardware, máquina de estados, comandos, segurança, telemetria |
| `esp32_pid_eng/pid.c`, `pid.h` | PID do LGR (Kp = 2Kz, Ki = Kz², Kd = K) com filtro na derivada e anti-windup |
| `esp32_pid_eng/swingup.c`, `swingup.h` | Swing-up: tranco inicial, medição do período, bombeamento de energia, identificação do sentido |
| `interface_pendulo.html` | Painel no navegador (Chrome/Edge) via Web Serial |

Para compilar, abra `esp32_pid_eng/esp32_pid_eng.ino` na Arduino IDE (os outros arquivos da pasta aparecem como abas) com o core ESP32 instalado.

---

## Como o swing-up funciona

Energia normalizada da haste (0 = parada embaixo, 2 = parada no topo):

```
E = 0.5 * (velocidade / w0)^2 + (1 - cos(ângulo))
```

Lei de bombeamento:

```
u = A * sat(kE * (Eref - E)) * tanh(velocidade * cos(ângulo) / s0)
aceleração do braço = -sinal * u - kp*braço - kd*vel_braço
```

O termo `kp/kd` mantém o braço perto do centro, e uma guarda de frenagem impede o braço de passar de ±80° durante o swing.

| Fase | O que acontece |
|---|---|
| **Tranco** | O braço vai 20° e volta para tirar a haste do repouso; nesse tranco o firmware **identifica o sentido certo do bombeio** observando para que lado a haste se move. |
| **Medição** | Só na 1ª vez: o braço fica parado e o firmware cronometra a haste para calcular `w0` (com correção de amplitude). |
| **Bombeio** | O braço balança em sincronia e a oscilação cresce até o topo. O alvo de energia aumenta sozinho se o atrito impedir de chegar. |
| **Captura** | A ±8° do topo e com velocidade < 2 rad/s, o PID do LGR assume. |

Se a haste já estava balançando (sem como identificar no tranco), o sentido do bombeio é confirmado pelo ganho de altura a cada meia-oscilação; se os picos encolhem ou estagnam, ele tenta o sentido oposto.

---

## Modos

| Modo | Descrição |
|---|---|
| `PARADO` | Motor parado. Com o automático desligado, o PID assume se você levantar a haste até ±30° do topo. |
| `SWING` | Balançando para subir. |
| `EQUIL` | PID equilibrando. Se sair de ±30°: no automático volta ao swing; no manual para. |
| `RETORNO` | Braço volta devagar ao centro. |
| `FALHA` | Parada de segurança: braço passou de ±130° do centro do cabo. |

---

## Comandos (Monitor Serial, 115200 baud)

| Comando | Ação |
|---|---|
| `u` | Liga/desliga o swing-up automático |
| `c` | Zera o encoder (haste parada embaixo) e marca o centro do braço |
| `a` | Autocalibração do sentido do motor |
| `i` | Inverte `sinal_motor` (sentido do equilíbrio) |
| `+` / `-` | Aumenta / diminui a força do bombeio (5 a 80 rad/s²) |
| `b` | Inverte manualmente o sentido do bombeio |
| `l` | Imprime o log do último equilíbrio (motor parado) |
| `t` / `q` | Liga / desliga a telemetria rápida (50 Hz) |
| `g` | Envia os ganhos atuais |

Botão BOOT: clique = `c` · segurar 1–3 s = `a` · segurar > 3 s = `u`.

---

## Interface (`interface_pendulo.html`)

1. Grave o firmware e **feche o Monitor Serial** da Arduino IDE.
2. Abra `interface_pendulo.html` no **Chrome ou Edge**.
3. Clique em **Conectar** e escolha a porta do ESP32.

Mostra em tempo real o ângulo da haste, o braço, a energia, o torque do PID e os passos, além dos ganhos do PID e do swing-up. Tem botões para os comandos, pausa, exportação em CSV e um modo **Demo** que funciona sem a placa.

---

## Procedimento de uso

1. Braço no **centro do cabo**, livre para girar ±90°; fonte do motor ao alcance (parada de emergência).
2. Haste **parada embaixo** → `c`.
3. (Recomendado) Teste o equilíbrio manual levantando a haste com a mão. Se o PID não segurar, ajuste com `i`.
4. Haste parada, braço no centro → `c` → `u`.
5. Sequência esperada: tranco → medição → bombeio (pico crescendo) → `EQUIL`.

## Solução de problemas

| Sintoma | O que fazer |
|---|---|
| Oscilação cresce mas não chega ao topo | `+` algumas vezes |
| Oscilação não cresce | `b` |
| Motor faz barulho / perde passos | `-`; aumentar a corrente do A4988 (Vref); eliminar resistência no giro (cabo) |
| Passa pelo topo sem capturar | Aumentar `ZONA_CAPTURA_GRAUS` e `VEL_CAPTURA_RAD_S` |
| Braço balança demais | Reduzir `SWING_LIMITE_BRACO_DEG` ou aumentar `SWING_KP_BRACO` |
| `w0` fica com `p` (padrão) | Cronometrar 10 oscilações pequenas e colocar o período em `PERIODO_NATURAL_S` |

> **Atenção:** o motor de passo funciona em malha aberta. Se encontrar resistência e perder passos, o firmware deixa de saber a posição real do braço — o equilíbrio falha e o limite do cabo deixa de ser confiável. Depois disso, recoloque o braço no centro e envie `c`.

---

## Parâmetros principais (topo do `.ino`)

| `#define` | Padrão | Significado |
|---|---|---|
| `K_LGR`, `Z_LGR` | 0.01039, 4.5371 | Ganho e zero duplo do LGR |
| `ZONA_ATIVA_GRAUS` | 30° | Faixa em que o PID segura a haste |
| `LIMITE_BRACO_DEG` | 130° | Parada de segurança do cabo |
| `SWING_AUTO_NO_BOOT` | 0 | 1 = começa o swing sozinho ao ligar |
| `PERIODO_NATURAL_S` | 1.08 s | Período usado se a medição falhar |
| `SWING_ACEL_BOMBEIO` | 20 rad/s² | Força do bombeio |
| `SWING_LIMITE_BRACO_DEG` | 80° | Limite do braço no swing |
| `ZONA_CAPTURA_GRAUS` | 8° | Distância do topo para o PID assumir |
| `VEL_CAPTURA_RAD_S` | 2 rad/s | Velocidade máxima para capturar |

---

## Estado da validação

A lógica foi validada em um simulador do pêndulo de Furuta que reproduzia o firmware linha a linha, variando comprimento do braço, atrito, tipo de haste e sinais do motor. O teste no hardware real ainda está em andamento.
