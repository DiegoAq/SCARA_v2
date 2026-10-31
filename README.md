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
| E3 + E5 | PWM, servos y movimiento: `HOME`, `JOINT`, `MOVE` | validado en la placa |
| E3 + E5 | `PULSE`, `STOP`, `TRACE` | tests en PC OK; **falta probar en la placa** |
| E4 | AS5600 por I2C: `ANGLE`, `RAW [LIVE]`, `I2CSCAN`, `CALIB` y supervisión en cada tick | tests en PC OK; **falta probar en la placa**; anda un solo encoder |
| E6 | Repetibilidad y error: `REPEAT`, corrección final (`TRIM_ENABLE`) | tests en PC OK; **falta probar en la placa** |
| E7 | Eje Z y orientación, gripper, dos VL53L0X: `Z`, `ORIENT`, `SETZ0`, `ZTEST`, `GRIPPER`, `DIST` | tests en PC OK; **hardware sin montar** |
| E8 | Rutinas: `MOVE X Y Z`, `PICK`, `PLACE`, `PICKPLACE`, `PEN`, `DRAW` | tests en PC OK; **hardware sin montar** |
| E9 | Búsqueda de la caja: `SCAN`, `MAP`, `PICK` sin coordenadas | tests en PC OK; **hardware sin montar**, y hace falta **remontar el codo** (sección 8) |

Todo lo que dice "tests en PC" corrió contra hardware simulado (`tests/hw_sim.c`): verifica la lógica, no la
mecánica ni los registros. Los valores físicos que todavía no se midieron están en `app/config.h` marcados
`[CONFIRMAR]`.

Mecánica cargada en `app/config.h`: L1 = L2 = 200 mm. Con **los dos servos en 90°** el brazo queda extendido y la
pinza en **X = 0, Y = 400** (postura de HOME). Hombro θ1 ∈ [0°, 180°]; codo θ2 ∈ [−90°, +90°]; radio operativo
de 285 a 380 mm.

**Una junta sin encoder habilitado no sabe dónde está.** Al encender no se mandan pulsos; el primer `HOME` la
habilita directamente en la postura de HOME, y si estaba en otro lado da un salto. Una junta con encoder
habilitado arranca desde lo que mide y va a HOME con rampa.

## Lo que piden las pautas (Primera Instancia)

| Requisito | Dónde queda | Estado |
|---|---|---|
| Consola UART por interrupción, coordenadas por comando | `MOVE X Y [Z]`, en **mm**. El `MOVE X20 Y15 Z-5` de las pautas es `MOVE X200 Y150 Z-50`, que hoy da `ERR 2`: son 250 mm de radio, y el mínimo actual es 285 | anda |
| Cinemática directa e inversa en el micro, una sola rama de codo por movimiento | `cinematica`, `planificador` | anda |
| Dos juntas con servo, trayectoria coordinada | `MOVE`, `JOINT`, `HOME` | anda |
| Eje Z de ±10 cm | `Z`, `MOVE X Y Z` | código listo; **distinto de las pautas:** motores DC en vez de servo |
| Garra de 15 a 60 mm | `GRIPPER`, `PICK` | código listo |
| Orientación de la garra | `ORIENT`, `PICK ... R` | código listo |
| Sensor de distancia antes de sujetar | `DIST`, `PICK` | código listo, con dos sensores |
| Marcador: rutinas de trazado | `PEN`, `DRAW` | código listo; la fuerza de contacto la fija el resorte del soporte |
| Secuencia del Anexo B | `PICKPLACE` | código listo |
| Realimentación supervisada | AS5600, `TRACE`, `FAULT` | código listo; **distinto de las pautas:** I2C en vez de ADC, porque la salida analógica de estos módulos no funcionó |
| Precisión, repetibilidad, `e = √(Δx²+Δy²+Δz²)` | `REPEAT`, `CALIB`, y el error teórico en `e_resolucion` | código listo; falta medir |
| Espacio de trabajo 5 cm < R < 38 cm | `R_MIN_OP_MM`, `R_MAX_OP_MM` | el máximo cumple; **el mínimo no:** queda en 28,5 cm hasta remontar el codo y medir hasta dónde pliega |

Las pautas de esta instancia reciben las coordenadas por consola: buscar la caja con `SCAN` es un agregado.

No es código, y sigue a cargo del equipo: la memoria descriptiva, el análisis FEA y las mediciones.

## Pinout (STM32F411CEU6 Black Pill)

Fuente única del cableado: **actualizar esta tabla en el mismo commit en que cambie un pin.**
"En uso" = el firmware actual ya lo configura y el hardware está conectado. "Sin montar" = el firmware ya lo
configura, pero falta el hardware.

| Pin | Función | Periférico / AF | Estado | Notas |
|---|---|---|---|---|
| PA0 | Motor DC A, enable (ENA del L298N) | TIM2_CH1, AF1 | Sin montar | PWM 10 kHz; también es el botón KEY: no pulsarlo con los motores conectados |
| PA1 | Motor DC B, enable (ENB del L298N) | TIM2_CH2, AF1 | Sin montar | PWM 10 kHz |
| PA2 | L298N IN1 (motor A) | GPIO salida | Sin montar | |
| PA3 | L298N IN2 (motor A) | GPIO salida | Sin montar | |
| PA4 | L298N IN3 (motor B) | GPIO salida | Sin montar | |
| PA5 | L298N IN4 (motor B) | GPIO salida | Sin montar | |
| PA6 | Servo hombro (MG996R), señal | TIM3_CH1, AF2 | **En uso** | 50 Hz, 500–2500 µs; en bajo hasta el primer `HOME` |
| PA7 | Servo codo (MG996R), señal | TIM3_CH2, AF2 | **En uso** | 50 Hz, 500–2500 µs; en bajo hasta el primer `HOME` |
| PA8 | VL53L0X A, XSHUT | GPIO salida | Sin montar | dirección 0x2A, que le carga el firmware al arrancar |
| PA9 | Consola TX (→ RX del adaptador USB-serie) | USART1_TX, AF7 | **En uso** | 115200 8N1, 3,3 V |
| PA10 | Consola RX (← TX del adaptador USB-serie) | USART1_RX, AF7 | **En uso** | pull-up interno |
| PA11 / PA12 | USB D− / D+ | — | Libre | conector USB de la placa |
| PA13 / PA14 | SWDIO / SWCLK | SWD | **En uso** | ST-Link; no reasignar |
| PA15 | — | — | Libre | sale de reset como JTDI |
| PB0 / PB1 | — | (ADC1_IN8 / IN9) | Libres | se probó leer ahí los AS5600 por su salida analógica; no funcionó con estos módulos |
| PB2 | BOOT1 | — | No usar | pin de arranque |
| PB3 | AS5600 codo, SDA | I2C2, AF9 | **En uso** | sale de reset como JTDO |
| PB4 | Encoder óptico del motor A | EXTI4 | Sin montar | un canal; sale de reset como NJTRST |
| PB5 | Encoder óptico del motor B | EXTI9_5 | Sin montar | un canal |
| PB6 | Servo gripper (MG90S), señal | TIM4_CH1, AF2 | Sin montar | en bajo hasta el primer `GRIPPER` |
| PB7 | VL53L0X B, XSHUT | GPIO salida | Sin montar | dirección 0x29, la de fábrica |
| PB8 | AS5600 hombro, SCL | I2C1, AF4 | **En uso** | bus compartido con los dos VL53L0X |
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

- `app/` — aplicación: `motion` (brazo), `eje_z`, `gripper`, `secuencia` (rutinas de varios pasos), `laseres` (lecturas
  por turnos de los dos sensores), `busqueda` (de las lecturas a la caja), `console`.
  `cinematica`, `planificador`, `parser`, `metricas`, `mapa` y `config.h` son el **núcleo portable** (sin hardware).
- `drivers/` — periféricos por registros (CMSIS, sin HAL): `clock`, `systick`, `gpio`, `uart`, `pwm`, `servo`, `i2c`,
  `as5600`, `vl53l0x`, `motor`, `encoder`.
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

Necesitan `gcc` y `make` en el PATH (p. ej. el MinGW de Qt, con `mingw32-make`), desde Git Bash:

```sh
cd tests
make test                                   # todo: cinemática, planificador, parser, mapa, consola, robot, supervisión y búsqueda
make sim
./sim_move 100 300 -100 300 > recta.csv     # x0 y0 xf yf [speed] [accel] [t_stop_ms]
./sim_move J 90 0 0 300 > desde_home.csv    # partiendo de una postura articular (q1 q2)
```

`sim_move` imprime por `stderr` el resumen (duración, velocidad articular máxima, desviación de la recta) y por
`stdout` el CSV: `t_ms, s_mm, v_mm_s, x_cmd, y_cmd, q1_deg, q2_deg, w1_dps, w2_dps, desv_mm`.

## Pruebas pendientes en la placa

Marco de coordenadas: origen en el eje del hombro. **+Y es hacia donde apunta el brazo extendido** (HOME);
+X queda a la **derecha** mirando desde atrás del brazo hacia la pinza. Ángulos positivos = antihorario visto
desde arriba. θ1 = 90° es el brazo apuntando a +Y; θ2 = 0° es el codo recto. **+Z es hacia arriba.**

### Seguridad

- Tener a mano el corte de la **fuente de los servos** y de la **fuente de los motores**: son la parada de
  emergencia. `STOP` frena todo por software, con rampa.
- Despejar la zona: el brazo barre hasta 40 cm alrededor del hombro.
- Antes de cada prueba nueva, leer los valores de `app/config.h` que usa y bajar velocidades si hace falta:
  `JOINT_MOVE_DPS`, `JOINT_VMAX_DPS`, `JOINT_ACCEL_DPS2`, `Z_VEL_MM_S`, `MOTOR_PWM_MAX`.
- Probar de a un eje. El orden de abajo está pensado para que cada paso se apoye en el anterior.

### 1. `PULSE`, `STOP` y `TRACE`

Con los servos habilitados (`HOME`):

| Comando | Qué tiene que pasar |
|---|---|
| `PULSE 2 1700` | el codo se mueve despacio, unos 18°; al llegar, `DONE` |
| `HOME` | vuelve |
| `TRACE ON`, `MOVE 0 300` | sale una línea `T,t_ms,x,y,q1,q2,,,0.0` por tick, y `DONE`. Las columnas vacías son lo medido, que aparece al habilitar encoders |
| `TRACE OFF`, `MOVE 100 300` y, mientras se mueve, `STOP` | frena suave sobre la recta y avisa `STOPPED x=... y=...` |
| `JOINT 90 0` y, mientras se mueve, `STOP` | frena con rampa y avisa `STOPPED` |

Copiar la salida de `TRACE` a un archivo sirve para graficar la recta comandada.

### 2. Encoders (hoy, el que anda)

Primero, con los encoders en `0` en `config.h` (así vienen), comprobar la lectura:

1. `I2CSCAN` tiene que mostrar `0x36` en el bus del encoder que anda (`i2c1` hombro, `i2c2` codo).
2. `RAW` tiene que dar `iman=OK` en ese encoder. Girar la junta a mano, con la fuente de servos apagada: las
   cuentas tienen que cambiar parejo. Si el imán no da `OK`, ajustar la distancia al sensor antes de seguir.
3. `RAW LIVE` muestra las cuentas en vivo; cualquier tecla sale.

Calibrar. `CALIB n` (n = 1 hombro, 2 codo) recorre esa junta ±30° alrededor de HOME, espera en cada punto y mide.
**Revisar antes que ese recorrido no choque.** Después de `HOME`:

```
SCARA> CALIB 1
OK
C,1,60.00,123.40               <- junta, angulo comandado, lectura del encoder
...
CALIB junta=1 ENC_SIGN=1 ENC_ZERO_DEG=63.40 ganancia=0.9800 RANGE_DEG=176.4 ZERO_SERVO_DEG=0.0 residuo=0.40
DONE x=0.0 y=400.0 ...
```

| Resultado | Qué hacer en `config.h` |
|---|---|
| `ENC_SIGN`, `ENC_ZERO_DEG` | cargarlos en `ENC1_SIGN` y `ENC1_ZERO_DEG` (o `ENC2_...`) |
| `ganancia` | si se aleja de 1, el servo no recorre lo que se le pide: cargar `RANGE_DEG` en `SHOULDER_RANGE_DEG` (o `ELBOW_...`) y `ZERO_SERVO_DEG` en `SHOULDER_ZERO_SERVO_DEG` (o `ELBOW_...`) |
| `residuo` | peor desvío respecto de una recta, en grados. Más de 1° indica juego mecánico o un servo poco lineal |

`CALIB` toma al servo como referencia: si el brazo no está geométricamente recto en HOME, ese error queda en el cero.
Para afinarlo, llevar la junta con `JOINT` hasta alinearla contra una regla y cargar en `ENCx_ZERO_DEG` lo que lea
`ANGLE` en ese momento.

Habilitar: poner `ENC1_HABILITADO` (o `ENC2_...`) en `1` y recompilar. **Recién ahí** esa junta arranca desde lo
medido y se supervisa. Comprobar:

| Prueba | Qué tiene que pasar |
|---|---|
| Con la fuente de servos apagada, mover la junta unos 20° a mano. Encender y `HOME` | esa junta **no salta**: vuelve despacio a HOME |
| `STATUS` | agrega `q1med=.. err1=..` |
| `MOVE 0 300` | `DONE` agrega `q1med=.. err1=.. e=..`. `e` es la distancia en mm entre el destino y lo medido |
| Frenar el eslabón con la mano | a los 200 ms con más de 10° de error: `FAULT`. `MOVE` y `JOINT` se rechazan con `ERR 5`. `HOME` rearma |

Con un solo encoder, `e` mide solo el error de esa junta. Al reemplazar el otro módulo, repetir esta sección.

### 3. Precisión y repetibilidad

Necesita al menos un encoder habilitado. Va y vuelve entre dos puntos y mide cada llegada:

```
SCARA> REPEAT 0 300 100 300 10
R,0,1,0.00,300.00,0.12,300.08,0.144      <- ciclo, punto, x y comandados, x y medidos, e
...
REPEAT punto=1 n=10 e_media=.. e_desvio=.. e_max=.. exactitud=.. repetibilidad=.. e_resolucion=..
```

- `e_media`, `e_desvio`, `e_max`: el error `e` de las pautas (mm).
- `exactitud`: distancia del promedio de las llegadas al punto pedido. Es lo que corrige calibrar mejor.
- `repetibilidad`: 3σ de las llegadas alrededor de su promedio. Es lo que no se corrige calibrando.
- `e_resolucion`: cálculo analítico, el menor error que el encoder distingue en esa postura. Medir menos que esto
  no significa nada.

Estos números salen de los encoders: miden el ángulo de las juntas, no la flexión de los eslabones. Para la memoria,
contrastarlos con una regla o papel milimetrado en la punta. Si la exactitud no alcanza, probar `TRIM_ENABLE` en `1`:
al llegar, corrige hasta 3° por junta con el error medido.

### 4. Eje Z y orientación (al montar motores, L298N y encoders ópticos)

Con la varilla **lejos de los extremos**: no hay finales de carrera.

1. `ZTEST A 40 300`: mueve solo el motor A, 40 % de duty durante 300 ms, y responde `DONE ztest motor=A pulsos=..`.
   Tiene que subir el eje (o la parte que corresponda) y dar más de 0 pulsos. Si baja, `MOTOR_A_SIGN (-1.0f)`.
   Si da 0 pulsos, revisar el encoder antes de seguir. Repetir con `ZTEST B 40 300`.
2. Buscar el mínimo: bajar el duty hasta que el motor apenas se mueva. Ese valor, como fracción, es `MOTOR_PWM_MIN`.
3. Medir pulsos por mm: alternar `ZTEST A` y `ZTEST B` en el mismo sentido, sumar los pulsos de cada motor y medir
   con una regla cuánto subió la varilla. `PULSOS_POR_MM` = pulsos de un motor / mm. Igual para `PULSOS_POR_GRADO`,
   con sentidos opuestos y midiendo cuánto giró. Si con duty positivo en A y negativo en B gira en horario,
   `ROT_SIGN (-1.0f)`.
4. `SETZ0` declara la posición actual como Z = 0. Hay que hacerlo en cada encendido, y después de un atasco o un `ZTEST`.

| Comando | Qué tiene que pasar |
|---|---|
| `Z 20` | sube 20 mm; `DONE z=20.0 rot=0.0`. Medir con una regla |
| `Z -20` | baja; invierte frenando antes, sin golpe |
| `ORIENT 45` | la varilla gira 45° sin cambiar la altura |
| `Z 80` y, mientras sube, `STOP` | frena y avisa `STOPPED` |
| Frenar un motor con la mano durante `Z 20` | a los 500 ms sin pulsos corta y avisa `FAULT eje Z atascado`; `SETZ0` rearma |

El encoder es de un canal: el firmware cuenta según el sentido que comanda. Si el eje se mueve solo, o a mano, no se
entera. Cargar en `config.h` los recorridos reales: `Z_MIN_MM`, `Z_MAX_MM`, `ROT_MIN_DEG`, `ROT_MAX_DEG`.

### 5. Garra (al montar el MG90S en PB6)

`PULSE 3 us` mueve el servo con rampa, sin límites. Buscar el pulso con la garra en 15 mm y en 60 mm, sin que el servo
quede forzando, y cargarlos en `GRIPPER_PULSO_MIN_MM_US` y `GRIPPER_PULSO_MAX_MM_US`.

Al primer comando el servo va de golpe a la apertura supuesta (60 mm), porque no tiene encoder: arrancar con la garra
cerca de abierta. Después, `GRIPPER OPEN`, `GRIPPER CLOSE`, `GRIPPER 30` responden `DONE gripper=.. pulso=..`.

### 6. Sensores de distancia (al montar los dos VL53L0X en I2C1, XSHUT en PA8 y PB7)

Van fijos a la garra, mirando hacia abajo, uno a cada lado: la línea que los une es perpendicular al cierre de los
dedos. El **A** es el del XSHUT en PA8 y el **B** el de PB7. Salen de fábrica con la misma dirección: el firmware los
separa al arrancar usando los XSHUT, así que los dos tienen que estar cableados.

1. `I2CSCAN` tiene que mostrar `0x2A` y `0x29` en `i2c1`, además del `0x36`.
2. `STATUS` tiene que decir `sensor=OK sensor2=OK` (el A y el B). Si uno dice `NO`, revisar el cableado y reiniciar: se
   inicializan solo al encender. `DIST` y `PICK` andan con uno solo; `SCAN` necesita los dos.
3. `DIST` responde `DONE dist=.. dist2=..` en mm (el A y el B). Comparar con una regla a varias distancias. Sin nada
   delante: `SIN_OBJETO`. Tapar solo uno con la mano: si cambia el otro, los XSHUT están cruzados.
4. Con los dos a la misma altura sobre una superficie plana, cargar lo que cada uno mide de más en
   `SENSOR_A_OFFSET_MM` y `SENSOR_B_OFFSET_MM`.

### 7. Rutinas (con todo lo anterior andando)

Cargar en `config.h` las alturas reales: `Z_SEGURA_MM`, `Z_DEJAR_MM`, `Z_LAPIZ_ARRIBA_MM`, `Z_LAPIZ_ABAJO_MM`, y
`PICK_DIST_TOMA_MM` (distancia de los sensores al objeto con el objeto dentro de la garra). Cada paso que empieza se
avisa con una línea `SEQ paso=..`. `STOP` frena todo y corta la rutina. Antes de mover nada se validan todos los
tramos: si uno no se puede hacer, responde `ERR` con el brazo quieto.

**El `R` de `PICK` es un ángulo en la mesa**, desde +X: la dirección del lado largo de la caja. Para eso el firmware suma
las juntas a la orientación, y necesita saber cómo quedó montada la garra. Antes de `SETZ0`, girarla a mano hasta que la
línea del sensor B al A quede paralela al antebrazo, con el A hacia la punta: así `GARRA_YAW_CERO_DEG` queda en 0. Si no
se puede, cargar ahí el ángulo que quedó. `ORIENT` sigue siendo relativo al antebrazo.

| Comando | Qué hace |
|---|---|
| `MOVE 100 300 30` | si Z sube, primero sube y después se traslada; si baja, al revés |
| `PICK 0 300 R45 A25` | sube, va al punto, deja la línea de sensores a 45° en la mesa, abre, se acerca midiendo en tres veces hasta tener el objeto a `PICK_DIST_APROX_MM`, baja hasta dejarlo a `PICK_DIST_TOMA_MM`, cierra a 25 mm y sube. Sin objeto a la vista corta con `FAULT rutina cortada: sin objeto...`; si los sensores ven cosas distintas, con `...los dos sensores no ven lo mismo`, sin seguir bajando |
| `PLACE 100 300` | sube, va al punto, baja a la altura a la que tomó más `PLACE_HOLGURA_MM` (sin un `PICK` antes, a `Z_DEJAR_MM`), abre y sube |
| `PICKPLACE 0 300 100 300 45 25` | el diagrama del Anexo B: `PICK` + `PLACE` + volver a HOME |
| `PEN DOWN`, `PEN UP` | apoya y levanta el marcador; después, `MOVE X Y` traza rectas |
| `DRAW RECT 0 300 40 30` | rectángulo desde esa esquina, de 40 × 30 mm |
| `DRAW CIRCLE 0 330 20` | círculo de 20 mm de radio, en tramos rectos de 5 mm que frenan en cada unión |

Para probar `PICK` antes de tener los sensores, poner `PICK_USA_SENSOR` en `0`: baja siempre a `Z_TOMA_MM`.

### 8. Buscar la caja: `SCAN`, `MAP` y `PICK`

**Antes hay que remontar el codo.** `SCAN` cruza la caja con pasadas lentas de 55 mm a cada lado de su centro, y hoy el
anillo de trabajo tiene 95 mm de ancho (de 285 a 380). Barre y ubica la caja, pero corta con `FAULT rutina cortada: las
pasadas sobre la caja salen del espacio de trabajo`. Hasta entonces, `SEQ mapa` da una ubicación aproximada para probar
con `PICK x y R`. Hay que cambiar `ELBOW_ZERO_SERVO_DEG` y `R_MIN_OP_MM` según hasta dónde pliegue sin chocar. Los tests
de la PC usan 20 y 150 solo como ejemplo.

Cargar en `config.h`:

| Constante | Qué es |
|---|---|
| `CAJA_LARGO_MM`, `CAJA_ANCHO_MM`, `CAJA_ALTOS_MM` | medidas de las cajas. **Hoy están de memoria: confirmarlas** |
| `SENSOR_SEPARACION_MM` | distancia entre los dos haces, medida con regla. Tiene que ser menor que el largo de la caja |
| `SENSOR_DESVIO_MM` | cuánto queda corrida la línea de sensores respecto del centro de los dedos, si no pasa por ahí |
| `Z_BARRIDO_MM` | altura durante el barrido: la más baja en la que la punta de los dedos pasa sobre la caja más alta |
| `BARRIDO_DIST_MIN_MM` | distancia de los sensores a la punta de los dedos, más un margen: una lectura menor corta el barrido |

**Primero, medir cómo ve un borde el sensor real.** El simulador supone que la lectura promedia lo que entra en el cono, y
eso es lo que falta confirmar. Con una caja de cada alto, a `Z_BARRIDO_MM`, cruzar un lado largo con `MOVE x y` de a 5 mm
y anotar `dist` y `dist2` en cada punto con `DIST`. Tiene que verse un escalón suave entre la mesa y la tapa.

**Después, buscar.** Poner una caja en un lugar medido con regla, y dar una esquina del área respecto de la base y sus
lados, con signo. El área queda alineada con los ejes X e Y del robot:

```
SCARA> SCAN -90 180 180 150
OK
SEQ paso=0 Z a=50.0 b=0.0
...
SEQ mapa cajas=1 x=-0.9 y=250.3 rot=-1.5 largo=105 ancho=46 mesa=150     <- terminó el barrido: ubicación aproximada
...
SEQ eje x=-1.0 y=249.8 rot=179.2 ancho=30.9                              <- terminó la pasada a lo ancho
...
CAJA x=-0.2 y=249.8 rot=179.2 largo=96.3 ancho=30.9 alto=79.5 tipo=2     <- la caja medida
DONE x=-0.2 y=249.8 ...
```

| Qué | Qué tiene que pasar |
|---|---|
| Recorrido | zigzag sobre toda el área, después pasadas lentas sobre la caja. Baja con la **garra abierta** y los dedos a los costados hasta `PICK_DIST_APROX_MM` de la tapa, mide y vuelve a subir. **No la toca.** Un área de 180 × 150 tarda unos 2,5 minutos |
| `x`, `y` de `CAJA` | centro de la caja, a 3 mm de lo medido con regla |
| `rot` | dirección del lado largo, a 3° de lo medido. `rot` y `rot ± 180` son la misma caja |
| `alto`, `tipo` | altura sobre la mesa, y el orden en `CAJA_ALTOS_MM` (1 = la más baja); `tipo=0` si no se parece a ninguna |
| `SPEED` | al terminar, o después de un `STOP`, vuelve a lo que estaba |
| Varias cajas | `cajas=` dice cuántas vio y sigue con la de mayor superficie |

`MAP` repite lo que vio el barrido, para pegarlo en una planilla: `OK celda=15.0 filas=11 columnas=13 mesa=150`, una línea
`M,fila,altura,altura,...` por cada Y, con las alturas en mm sobre la mesa y el campo vacío si esa celda no tuvo lecturas,
y `DONE map`. La caja tiene que verse como una mancha. Las cajas bajas aparecen más bajas de lo que son: desde esa altura
el cono toma parte de la mesa. Por eso `alto` se mide de cerca.

`PICK`, sin argumentos, toma esa caja. Después `PLACE x y` la suelta a la misma altura, sea del tipo que sea. Un segundo
`PICK` responde `ERR 9` hasta hacer otro `SCAN`: la caja ya no está donde estaba.

| Si aparece | Qué revisar |
|---|---|
| `ERR 8` | `STATUS`: `SCAN` necesita los dos sensores |
| `ERR 2` | el área sale del espacio de trabajo. Nada se movió |
| `ERR 6` | un lado vale 0, o el área no entra en la grilla del mapa |
| `FAULT ... no se encontro ninguna caja` | mirar `MAP`. Si la caja no se distingue, bajar `Z_BARRIDO_MM` o `CAJA_UMBRAL_MM` |
| `FAULT ... un sensor quedo muy cerca` | `Z_BARRIDO_MM` demasiado baja para esa caja |
| `FAULT ... las pasadas sobre la caja salen del espacio de trabajo` | la caja está demasiado cerca del borde del anillo. El brazo no llegó a ir hacia ella |
| `FAULT ... no se encontraron los bordes de la caja` | `SENSOR_SEPARACION_MM`, o subir `CENTRADO_MARGEN_MM` |
| `FAULT ... los dos sensores no ven lo mismo` | los `SENSOR_x_OFFSET_MM`, `GARRA_YAW_CERO_DEG` y `ROT_SIGN` |
| `FAULT ... lo medido no coincide con las medidas de la caja` | las `CAJA_...`; el margen es `CAJA_TOL_MEDIDA_MM` |

Si una rutina corta con la garra abajo, subir con `Z` antes de mover en XY: `MOVE X Y` se traslada a la altura actual.

## Qué reportar

1. **Sección 1:** si `PULSE`, `STOP` y `TRACE` dieron lo esperado.
2. **Encoders:** cuál anda, la salida de `CALIB`, y si el arranque sin salto y el `FAULT` funcionaron.
3. **`REPEAT`:** las líneas de resultado, y lo medido con regla en la punta.
4. **Eje Z, garra y sensores:** los valores que quedaron en `config.h`, y si hubo tirones, atascos falsos o pulsos perdidos.
5. **Rutinas:** si `PICKPLACE` completa el ciclo, y en qué paso falló si no.
6. **Montaje del codo:** hasta qué ángulo de servo llega sin chocar con la base. Con eso se recalcula `R_MIN_OP_MM`.
7. **`SCAN`:** las medidas reales de las cajas, la separación entre sensores, la tabla del borde con cada alto y la línea
   `CAJA` contra lo medido con regla.
8. Cualquier paso que no dio lo esperado, con lo que apareció en la consola. `STATUS` muestra `tick_max_ms`: tiene
   que quedar cerca de 20.
