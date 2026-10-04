# Firmware Robot SCARA — STM32F411 Black Pill (baremetal)

Proyecto Final Integrador — Sistemas Embebidos — Ing. Mecatrónica UNER.
Integrantes: Aquino Diego · Leandro Meolans.

Firmware 100 % baremetal (CMSIS + registros, sin HAL ni RTOS) para un brazo SCARA de 2 articulaciones
rotacionales + eje Z + gripper. Se desarrolla por etapas (E0…E7); cada una se valida antes de seguir.

## Estado

| Etapa | Contenido | Estado |
|---|---|---|
| E0 | Reloj 96 MHz, SysTick, LED PC13 | LED a 1 Hz validado en la placa |
| E1 | `config.h`, cinemática, planificador, parser + tests | tests en PC OK (mecánica nueva: codo recto en 180° de servo) |
| E2 | UART + consola microrl: `HELP`, `STATUS`, `SPEED`, `ACCEL` | consola y comandos validados en la placa |
| E4a | AS5600 por salida analógica (ADC): `ANGLE`, `RAW` (adelantado, para que E3 arranque desde la posición medida) | test en PC OK; **falta validar en la placa y calibrar** |
| E3 | PWM + servos: `PULSE`, `JOINT` | pendiente (necesita la calibración de E4a) |
| E4b…E7 | `CALIB`, `MOVE`, repetibilidad, Z | pendiente |

Mecánica cargada en `app/config.h`: L1 = L2 = 200 mm; hombro θ1 ∈ [−90°, +90°]; codo θ2 ∈ [0°, 152°]
(se pliega en sentido antihorario; el servo se monta con su 0° en el tope de plegado y el ángulo de servo con el
brazo recto se mide en E3); radio operativo de 100 a 395 mm. Los valores del codo son provisorios hasta E3.

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
| PA6 | Servo hombro (MG996R), señal | TIM3_CH1, AF2 | Reservado (E3) | 50 Hz |
| PA7 | Servo codo (MG996R), señal | TIM3_CH2, AF2 | Reservado (E3) | 50 Hz |
| PA8 | VL53L0X XSHUT | GPIO salida | Reservado | fuera de alcance por ahora |
| PA9 | Consola TX (→ RX del adaptador USB-serie) | USART1_TX, AF7 | **En uso** | 115200 8N1, 3,3 V |
| PA10 | Consola RX (← TX del adaptador USB-serie) | USART1_RX, AF7 | **En uso** | pull-up interno |
| PA11 / PA12 | USB D− / D+ | — | Libre | conector USB de la placa |
| PA13 / PA14 | SWDIO / SWCLK | SWD | **En uso** | ST-Link; no reasignar |
| PA15 | — | — | Libre | sale de reset como JTDI |
| PB0 | Encoder hombro: OUT analógico del AS5600 | ADC1_IN8 | **En uso** | 0–3,3 V; sensor a 3,3 V |
| PB1 | Encoder codo: OUT analógico del AS5600 | ADC1_IN9 | **En uso** | 0–3,3 V; sensor a 3,3 V |
| PB2 | BOOT1 | — | No usar | pin de arranque |
| PB3 | I2C2 SDA | I2C2, AF9 | Libre (solo `I2CSCAN`) | era del AS5600 del codo; sale de reset como JTDO |
| PB4 | Encoder óptico eje Z, canal A | EXTI4 | Reservado (E7) | sale de reset como NJTRST |
| PB5 | Encoder óptico eje Z, canal B | EXTI9_5 | Reservado (E7) | |
| PB6 | Servo gripper (MG90S), señal | TIM4_CH1, AF2 | Reservado (E7) | 50 Hz |
| PB7 | — | — | Libre | |
| PB8 | I2C1 SCL | I2C1, AF4 | Reservado (VL53L0X) | era del AS5600 del hombro; hoy solo `I2CSCAN` |
| PB9 | I2C1 SDA | I2C1, AF4 | Reservado (VL53L0X) | ídem |
| PB10 | I2C2 SCL | I2C2, AF4 | Libre (solo `I2CSCAN`) | era del AS5600 del codo |
| PB12–PB15 | — | — | Libres | |
| PC13 | LED de estado | GPIO salida | **En uso** | integrado, activo en bajo; 1 Hz = OK, 5 Hz = sin cristal |
| PC14 / PC15 | Cristal de 32,768 kHz (LSE) | — | No usar | |
| PH0 / PH1 | Cristal de 25 MHz (HSE) | — | **En uso** | reloj del sistema |

Alimentación:

- **Black Pill:** 5 V por USB o ST-Link; 3,3 V del regulador de la placa.
- **AS5600:** VDD a **3,3 V** de la placa (nunca 5 V: la salida OUT dañaría PB0/PB1). Pin DIR fijo a GND o VDD.
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
make test                                   # cinemática, planificador, parser y consola
make sim
./sim_move 300 -100 300 100 > recta.csv     # x0 y0 xf yf [speed] [accel] [t_stop_ms]
./sim_move J 0 0 300 100 > desde_home.csv   # partiendo de una postura articular (q1 q2)
```

`sim_move` imprime por `stderr` el resumen (duración, velocidad articular máxima, desviación de la recta) y por
`stdout` el CSV: `t_ms, s_mm, v_mm_s, x_cmd, y_cmd, q1_deg, q2_deg, w1_dps, w2_dps, desv_mm`.

## Prueba pendiente en la placa: encoders AS5600 (E4a)

Las pruebas de E0 (reloj y LED) y E2 (consola) ya pasaron; esta es la única pendiente.

**Con los servos sin alimentar** (fuente de servos apagada): en esta etapa las juntas se mueven a mano.

- Consola: adaptador USB-serie de 3,3 V, TX del adaptador → PA10, RX → PA9, GND común; 115200 8N1.
- Encoders: se leen por su salida analógica **OUT** con el ADC. OUT del AS5600 del hombro → **PB0**;
  OUT del AS5600 del codo → **PB1**. VDD del sensor a **3,3 V** (nunca 5 V: OUT llegaría a 5 V y daña el pin) y GND.
- En cada módulo, el pin **DIR** tiene que estar fijo a GND o a VDD, no al aire: si flota, el sentido de la
  lectura puede invertirse solo. SDA y SCL quedan sin conectar.

Por ADC el firmware no puede saber si falta el imán o si hay un cable suelto (un pin al aire da lecturas al
azar). Lo que avisa de un problema es el **ruido** que muestra `RAW`.

### 1. Lectura

1. `RAW` → `OK raw1=<0..4095> ruido1=<n> raw2=<0..4095> ruido2=<n>`.
   `raw` son las cuentas del ADC (0 = 0°, 4095 ≈ 360°); `ruido` es cuánto varió la lectura dentro del promedio.
2. Ruido: con el brazo quieto, repetir `RAW` 10 veces (flecha arriba + Enter).
   - `ruido` debe quedar en 8 cuentas o menos, y `raw` no debe moverse más de ±3 cuentas (0,3°) entre repeticiones.
   - Si `ruido` da cientos o `raw` salta sin tocar el brazo: cable de OUT suelto, sensor sin alimentar o sin imán.
   - Anotar el mínimo y el máximo de `raw1` y de `raw2`.
3. Girar cada junta a mano de tope a tope repitiendo `RAW`: `raw` debe cambiar de forma continua.
   **Anotar `raw` en cada tope.** Si en el recorrido la lectura pasa de ~4095 a ~0 (o al revés), el cruce 0/360 del
   sensor cae dentro del rango de trabajo: funciona, pero es la zona más ruidosa. Si se puede, girar el imán
   respecto del sensor para que el cruce quede fuera del recorrido, y repetir este paso.
4. Desconectar un momento el cable de OUT de un sensor y hacer `RAW`: el `ruido` de ese sensor tiene que subir
   mucho. Sirve para comprobar que cada lectura viene del sensor que corresponde. Volver a conectar.
5. `STATUS` → `tick_max_ms` en 20 o 21.

### 2. Calibración a mano

No hacen falta finales de carrera: el AS5600 es absoluto también por su salida analógica, así que el cero se
fija una sola vez y queda guardado en `config.h` (no hay que buscar referencia en cada encendido).

Usar `ANGLE`, que responde `OK enc1=<grados> q1=<grados> enc2=<grados> q2=<grados>`.
`enc` es la lectura del sensor (0 a 360); `q` es el ángulo de junta, que recién vale después de calibrar.

La precisión del cero depende de qué tan bien se alinee el brazo en el paso (a): cada grado de error ahí
corre la pinza unos 7 mm con el brazo extendido. Conviene marcar en la mesa una línea recta desde el eje del
hombro hacia el frente y alinear contra ella los dos eslabones (o apoyar una regla a lo largo del brazo).

Esta calibración es la de los **encoders**. La de los **servos** (qué pulso corresponde a cada ángulo) se hace
en E3, moviéndolos despacio y usando los encoders ya calibrados como referencia. Ahí se mide también en qué
ángulo de servo queda el codo completamente extendido (el servo se monta con su 0° en el tope de plegado).

Hacer esta calibración con el montaje mecánico **ya definitivo**: si después se desarma una junta y el imán del
AS5600 gira o se corre respecto del sensor, hay que repetirla. Reposicionar solo el brazo (horn) del servo, sin
tocar el imán, no la afecta.

| Paso | Postura (mirando el brazo desde arriba) | Anotar |
|---|---|---|
| a | Brazo recto y apuntando al frente (+X) | `enc1` = ______ `enc2` = ______ |
| b | Desde (a), girar solo el hombro unos 45° hacia la **izquierda** (antihorario) | `enc1` = ______ |
| c | Volver a (a) y plegar solo el codo unos 45° (se pliega en sentido antihorario) | `enc2` = ______ |

En (b) y (c) el giro es **a ojo**: solo sirve para ver si la lectura sube o baja, no importa si son 30° o 60°.
El único paso que pide precisión es el (a).

Con eso, editar `app/config.h`:

- `ENC1_ZERO_DEG` = `enc1` del paso (a). `ENC2_ZERO_DEG` = `enc2` del paso (a).
- `ENC1_SIGN` = `1.0f` si `enc1` **aumentó** de (a) a (b); `(-1.0f)` si bajó.
- `ENC2_SIGN` = `1.0f` si `enc2` **aumentó** de (a) a (c); `(-1.0f)` si bajó.
- Si la lectura cruzó el 0/360 entre dos pasos (p. ej. de 350 a 35), cuenta como aumento.

### 3. Verificación (después de recompilar con los valores nuevos)

| Postura | `ANGLE` debe dar | Medido |
|---|---|---|
| Brazo recto al frente | `q1` ≈ 0, `q2` ≈ 0 (±1°) | `q1` = ____ `q2` = ____ |
| Hombro 90° a la izquierda (con escuadra) | `q1` ≈ +90 | `q1` = ____ |
| Codo plegado 90° (con escuadra) | `q2` ≈ +90 | `q2` = ____ |
| Hombro contra el tope izquierdo | — | `q1` = ____ |
| Hombro contra el tope derecho | negativo | `q1` = ____ |
| Codo plegado hasta que choca la pinza | positivo, cerca de 150 | `q2` = ____ |

**Corrección de ganancia.** La salida analógica no llega exactamente a 0 V ni a 3,3 V, así que los 90° de la
escuadra pueden leerse como 88° o 92°. Si la diferencia pasa de 1°, cargar en `config.h`
`ENC1_ESCALA` = 90 / (`q1` medido a 90°) y `ENC2_ESCALA` = 90 / (`q2` medido a 90°), recompilar y repetir la tabla.

Las tres últimas filas son los topes mecánicos reales: con ellas se ajustan los límites de `config.h` antes de mover servos.

## Qué reportar

1. **Lectura:** lo que respondió `RAW` (mínimo y máximo de `raw` y de `ruido` con el brazo quieto), `raw` en cada
   tope, y si el cruce 0/360 cae dentro del recorrido de alguna junta.
2. **Calibración:** las tablas de calibración y de verificación completas, y los valores cargados en `config.h`.
3. Cualquier paso que no dio lo esperado, con lo que apareció en la consola.
