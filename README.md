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
| E4a | I2C + AS5600: `ANGLE`, `RAW`, `I2CSCAN` (adelantado, para que E3 arranque desde la posición medida) | test en PC OK; **falta validar en la placa y calibrar** |
| E3 | PWM + servos: `PULSE`, `JOINT` | pendiente (necesita la calibración de E4a) |
| E4b…E7 | `CALIB`, `MOVE`, repetibilidad, Z | pendiente |

Mecánica cargada en `app/config.h`: L1 = L2 = 200 mm; hombro θ1 ∈ [−90°, +90°]; codo θ2 ∈ [0°, 152°]
(se pliega en sentido antihorario; el servo se monta con su 0° en el tope de plegado y el ángulo de servo con el
brazo recto se mide en E3); radio operativo de 100 a 395 mm. Los valores del codo son provisorios hasta E3.

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
- Encoders: AS5600 del hombro en I2C1 (PB8 SCL, PB9 SDA); AS5600 del codo en I2C2 (PB10 SCL, PB3 SDA); 3,3 V y GND.

### 1. Comunicación

1. `I2CSCAN` → `OK i2c1=0x36 i2c2=0x36`.
   `ninguno` = el sensor no contesta (alimentación, SCL/SDA cruzados). `BUS_ERROR` = línea en corto o sin pull-ups.
2. `RAW` → `OK raw1=<0..4095> iman1=OK raw2=<0..4095> iman2=OK`.
   `DEBIL` = imán lejos; `FUERTE` = imán muy cerca; `AUSENTE` = sin imán. Ajustar la distancia (0,5 a 3 mm) hasta `OK`.
3. Ruido: con el brazo quieto, repetir `RAW` 10 veces (flecha arriba + Enter), o usar `RAW LIVE`
   (repite una línea cada 150 ms hasta que se aprieta cualquier tecla; bloquea el loop principal,
   no usar con los servos alimentados). Cada lectura no debe moverse más de ±3 cuentas (0,3°).
   Anotar mínimo y máximo de cada una.
4. Girar cada junta a mano de tope a tope con `RAW LIVE`: la lectura debe cambiar de forma continua
   (el paso de 4095 a 0 es normal).
5. `STATUS` → `enc1=OK enc2=OK` y `tick_max_ms` en 20 o 21.

### 2. Calibración a mano

No hacen falta finales de carrera: el AS5600 es absoluto, así que el cero se fija una sola vez y queda
guardado en `config.h` (no hay que buscar referencia en cada encendido).

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
| Hombro 90° a la izquierda | `q1` ≈ +90 | `q1` = ____ |
| Hombro contra el tope izquierdo | — | `q1` = ____ |
| Hombro contra el tope derecho | negativo | `q1` = ____ |
| Codo plegado hasta que choca la pinza | positivo, cerca de 150 | `q2` = ____ |

Las tres últimas filas son los topes mecánicos reales: con ellas se ajustan los límites de `config.h` antes de mover servos.

## Qué reportar

1. **Comunicación:** lo que respondieron `I2CSCAN`, `RAW` y `STATUS`, y el mínimo y máximo de cuentas del paso de ruido.
2. **Calibración:** las tablas de calibración y de verificación completas.
3. Cualquier paso que no dio lo esperado, con lo que apareció en la consola.
