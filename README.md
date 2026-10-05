# Firmware Robot SCARA — STM32F411 Black Pill (baremetal)

Proyecto Final Integrador — Sistemas Embebidos — Ing. Mecatrónica UNER.
Integrantes: Aquino Diego · Leandro Meolans.

Firmware 100 % baremetal (CMSIS + registros, sin HAL ni RTOS) para un brazo SCARA de 2 articulaciones
rotacionales + eje Z + gripper. Se desarrolla por etapas (E0…E7); cada una se valida antes de seguir.

## Estado

| Etapa | Contenido | Estado |
|---|---|---|
| E0 | Reloj 96 MHz, SysTick, LED PC13 | validado en la placa |
| E1 | `config.h`, cinemática, planificador, parser + tests | tests en PC OK |
| E2 | UART + consola microrl: `HELP`, `STATUS`, `SPEED`, `ACCEL` | validado en la placa |
| E3 + E5 | PWM, servos y movimiento **en lazo abierto** (sin encoders): `HOME`, `JOINT`, `PULSE`, `MOVE`, `STOP`, `TRACE` | tests en PC OK; **falta probar en la placa** |
| E4 | AS5600 por I2C: `ANGLE`, `RAW [LIVE]`, `I2CSCAN` | solo diagnóstico; calibración postergada |
| E6, E7 | Repetibilidad, eje Z, gripper | pendiente |

Mecánica cargada en `app/config.h`: L1 = L2 = 200 mm. Con **los dos servos en 90°** el brazo queda extendido y la
pinza en **X = 0, Y = 400** (postura de HOME). Hombro θ1 ∈ [0°, 180°]; codo θ2 ∈ [−90°, +90°]; radio operativo
de 285 a 395 mm.

**Sin encoders el firmware no sabe dónde está el brazo.** Al encender supone que está en HOME y no manda pulsos;
el primer `HOME` habilita los servos directamente en esa postura.

## Pinout (STM32F411CEU6 Black Pill)

Fuente única del cableado: **actualizar esta tabla en el mismo commit en que cambie un pin.**
"En uso" = el firmware actual ya lo configura. "Reservado" = asignado, pero su código todavía no existe.

| Pin | Función | Periférico / AF | Estado | Notas |
|---|---|---|---|---|
| PA0 | Motor DC A, enable (ENA del L298N) | TIM2_CH1, AF1 | Reservado (E7) | PWM 10 kHz |
| PA1 | Motor DC B, enable (ENB del L298N) | TIM2_CH2, AF1 | Reservado (E7) | PWM 10 kHz |
| PA2 | L298N IN1 (motor A) | GPIO salida | Reservado (E7) | confirmado |
| PA3 | L298N IN2 (motor A) | GPIO salida | Reservado (E7) | confirmado |
| PA4 | L298N IN3 (motor B) | GPIO salida | Reservado (E7) | confirmado |
| PA5 | L298N IN4 (motor B) | GPIO salida | Reservado (E7) | confirmado |
| PA6 | Servo hombro (MG996R), señal | TIM3_CH1, AF2 | **En uso** | 50 Hz, 500–2500 µs; en bajo hasta el primer `HOME` |
| PA7 | Servo codo (MG996R), señal | TIM3_CH2, AF2 | **En uso** | 50 Hz, 500–2500 µs; en bajo hasta el primer `HOME` |
| PA8 | VL53L0X XSHUT | GPIO salida | Reservado | fuera de alcance por ahora |
| PA9 | Consola TX (→ RX del adaptador USB-serie) | USART1_TX, AF7 | **En uso** | 115200 8N1, 3,3 V |
| PA10 | Consola RX (← TX del adaptador USB-serie) | USART1_RX, AF7 | **En uso** | pull-up interno |
| PA11 / PA12 | USB D− / D+ | — | Libre | conector USB de la placa |
| PA13 / PA14 | SWDIO / SWCLK | SWD | **En uso** | ST-Link; no reasignar |
| PA15 | — | — | Libre | sale de reset como JTDI |
| PB0 / PB1 | — | (ADC1_IN8 / IN9) | Libres | se probó leer ahí los AS5600 por su salida analógica; no funcionó con estos módulos |
| PB2 | BOOT1 | — | No usar | pin de arranque |
| PB3 | AS5600 codo, SDA | I2C2, AF9 | **En uso** | sale de reset como JTDO |
| PB4 | Encoder óptico eje Z, canal A | EXTI4 | Reservado (E7) | sale de reset como NJTRST |
| PB5 | Encoder óptico eje Z, canal B | EXTI9_5 | Reservado (E7) | |
| PB6 | Servo gripper (MG90S), señal | TIM4_CH1, AF2 | **En uso** (sin pulsos) | el timer ya está configurado; el comando `GRIPPER` es de E7 |
| PB7 | — | — | Libre | |
| PB8 | AS5600 hombro, SCL | I2C1, AF4 | **En uso** | bus compartido con el VL53L0X (futuro) |
| PB9 | AS5600 hombro, SDA | I2C1, AF4 | **En uso** | |
| PB10 | AS5600 codo, SCL | I2C2, AF4 | **En uso** | |
| PB12–PB15 | — | — | Libres | |
| PC13 | LED de estado | GPIO salida | **En uso** | integrado, activo en bajo; 1 Hz = OK, 5 Hz = sin cristal |
| PC14 / PC15 | Cristal de 32,768 kHz (LSE) | — | No usar | |
| PH0 / PH1 | Cristal de 25 MHz (HSE) | — | **En uso** | reloj del sistema |

Alimentación:

- **Black Pill:** 5 V por USB o ST-Link; 3,3 V del regulador de la placa.
- **AS5600:** VDD a 3,3 V de la placa.
- **Servos:** fuente externa de 5–6 V, con **GND común** con la Black Pill y un capacitor de 470–1000 µF cerca de los servos.

## Carpetas

- `app/` — aplicación. `cinematica`, `planificador`, `parser` y `config.h` son el **núcleo portable** (sin hardware).
- `drivers/` — periféricos por registros (CMSIS, sin HAL).
- `lib/microrl/` — microrl 1.5.1 adaptado: `config.h` renombrado a `microrl_config.h`, sin `#define true/false`,
  prompt `SCARA> `, salida con CR+LF. `microrl.c` sin cambios.
- `cmsis/` — headers CMSIS y `system_stm32f4xx.c` del paquete ST (STM32Cube_FW_F4 V1.28.3), sin modificar.
- `Startup/`, `Src/` — startup, y stubs de newlib (`syscalls.c`, `sysmem.c`) de CubeIDE.
- `tests/` — tests en PC y `sim_move` (excluida del build de CubeIDE).

## Compilar el firmware

STM32CubeIDE: *File → Open Projects from File System* → esta carpeta → *Build*.
El `.cproject` ya tiene las carpetas `app/`, `drivers/`, `cmsis/`, `lib/`, el símbolo `STM32F411xE`, float ABI *hard*,
`-Wextra` y *Use float with printf from newlib-nano* (necesario para que `STATUS` muestre los números).

## Tests en PC

Necesitan `gcc` y `make` en el PATH (p. ej. el MinGW de Qt y el `make` de CubeIDE), desde Git Bash:

```sh
cd tests
make test                                   # cinemática, planificador, parser, consola y movimiento
make sim
./sim_move 100 300 -100 300 > recta.csv     # x0 y0 xf yf [speed] [accel] [t_stop_ms]
./sim_move J 90 0 0 300 > desde_home.csv    # partiendo de una postura articular (q1 q2)
```

`sim_move` imprime por `stderr` el resumen (duración, velocidad articular máxima, desviación de la recta) y por
`stdout` el CSV: `t_ms, s_mm, v_mm_s, x_cmd, y_cmd, q1_deg, q2_deg, w1_dps, w2_dps, desv_mm`.

## Prueba pendiente en la placa: mover el brazo en lazo abierto

Marco de coordenadas: origen en el eje del hombro. **+Y es hacia donde apunta el brazo extendido** (HOME);
+X queda a la **derecha** mirando desde atrás del brazo hacia la pinza. Ángulos positivos = antihorario visto
desde arriba. θ1 = 90° es el brazo apuntando a +Y; θ2 = 0° es el codo recto.

### Antes de empezar (seguridad)

- Tener a mano el corte de la **fuente de los servos**: es la parada de emergencia. `STOP` detiene por software.
- Despejar la zona: el brazo barre hasta 40 cm alrededor del hombro.
- El firmware mueve cada junta a 30°/s como máximo en `JOINT`/`PULSE` y nunca a más de 60°/s en `MOVE`.
  **La única excepción es el primer `HOME`**: ahí cada servo va de golpe a 90°, desde donde esté.
- No usar `RAW LIVE` con el brazo en movimiento (el firmware lo rechaza).

### 1. Habilitar los servos

1. Con la fuente de servos **apagada**, grabar y abrir la consola. `STATUS` debe mostrar `servos=OFF`.
2. Llevar el brazo **a mano** a la postura extendida (los dos servos cerca de 90°), para que el salto del paso 4
   sea mínimo.
3. Encender la fuente de servos. **No se tienen que mover ni hacer fuerza** (todavía no hay pulsos). Si se mueven,
   apagar y avisar.
4. `HOME` → responde `OK servos habilitados en HOME x=0.0 y=400.0 q1=90.0 q2=0.0`. Los servos quedan firmes en 90°.
5. Mirar el brazo: ¿quedó recto y apuntando al frente? Anotar a ojo cuánto se desvía cada eslabón.

### 2. Sentido de giro de cada servo (hacerlo antes de cualquier `MOVE`)

`config.h` **supone** que al subir el ángulo de servo la junta gira en sentido antihorario. Hay que comprobarlo:

| Comando | Qué tiene que pasar (visto desde arriba) | Si gira al revés, cambiar en `config.h` |
|---|---|---|
| `JOINT 100 0` | solo el **hombro** gira 10° hacia la **izquierda** (antihorario) | `SHOULDER_SIGN (-1.0f)` y `SHOULDER_ZERO_SERVO_DEG 180.0f` |
| `JOINT 90 0` | vuelve | |
| `JOINT 90 10` | solo el **codo** se pliega 10° hacia la **izquierda** (antihorario) | `ELBOW_SIGN (-1.0f)` (`ELBOW_ZERO_SERVO_DEG` queda en 90) |
| `HOME` | vuelve a extendido | |

Cada comando responde `OK` y, al terminar, `DONE x=... y=... q1=... q2=...`. Si hubo que cambiar un signo:
recompilar, grabar y repetir desde la sección 1. **Con un signo mal, `MOVE` no hace rectas.**

### 3. Recorrido de cada junta

De a pasos de 15° y mirando que nada choque (`STOP` si hace falta):

- Hombro: `JOINT 75 0`, `JOINT 60 0`, … hacia 0, y `JOINT 105 0`, `JOINT 120 0`, … hacia 180. Volver con `HOME`.
- Codo: `JOINT 90 15`, `JOINT 90 30`, … hacia 90, y `JOINT 90 -15`, … hacia −90. Volver con `HOME`.

Anotar hasta qué ángulo llega cada junta sin chocar ni forzar, para los dos lados. Si un servo zumba o empuja
contra algo, retroceder enseguida: ese es el tope.

`PULSE n us` (n = 1 hombro, 2 codo) manda un pulso crudo entre 500 y 2500 µs, también a 30°/s. No respeta los
límites articulares: usarlo solo para buscar el centro o los extremos reales de un servo.

### 4. `MOVE` en línea recta

Desde `HOME`, con una regla o papel milimetrado bajo la pinza:

| Comando | Esperado | Medido |
|---|---|---|
| `MOVE 0 300` | la pinza retrocede 100 mm en línea recta hacia la base; `DONE x=0.0 y=300.0` | recorrido = ____ mm |
| `MOVE 100 300` | 100 mm hacia la derecha, recto | ____ mm, desvío de la recta ____ mm |
| `MOVE -100 300` | 200 mm hacia la izquierda, recto | ____ mm, desvío ____ mm |
| `MOVE 0 350` | vuelve al centro, 50 mm más lejos | |
| `MOVE 0 100` | `ERR 2 fuera del espacio de trabajo`; no se mueve | |
| `MOVE 100 300` y, mientras se mueve, `STOP` | frena suave sobre la recta; `STOPPED x=... y=...` | |

- `SPEED v` (1 a 50 mm/s) y `ACCEL a` cambian el perfil; empezar con los valores por defecto (30 mm/s).
- `TRACE ON` imprime una línea por tick, `T,t_ms,x,y,q1,q2`, para graficar lo **comandado**. `TRACE OFF` la apaga.
- `STATUS` muestra `servos=ON q1=.. q2=.. x=.. y=..` (lo comandado, no lo medido).

El espacio alcanzable hoy es un anillo de 285 a 395 mm del hombro, del lado del frente.

## Qué reportar

1. **Habilitación:** si los servos se movieron al encender la fuente, y cómo quedó el brazo después de `HOME`.
2. **Sentidos:** hacia dónde giró cada junta en la sección 2 y si hubo que cambiar algún signo.
3. **Recorridos:** hasta qué ángulo llega cada junta para cada lado.
4. **MOVE:** la tabla de la sección 4 (recorrido medido y desvío de la recta), y si hubo tirones o temblores.
5. Cualquier paso que no dio lo esperado, con lo que apareció en la consola.
