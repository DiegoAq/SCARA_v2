# Firmware Robot SCARA — STM32F411 Black Pill (baremetal)

Proyecto Final Integrador — Sistemas Embebidos — Ing. Mecatrónica UNER.
Integrantes: Aquino Diego · Leandro Meolans.

Firmware 100 % baremetal (CMSIS + registros, sin HAL ni RTOS) para un brazo SCARA de 2 articulaciones
rotacionales + eje Z + gripper. Se desarrolla por etapas (E0…E7); cada una se valida antes de seguir.

## Estado

| Etapa | Contenido | Estado |
|---|---|---|
| E0 | Reloj 96 MHz, SysTick, LED PC13 | compila; **falta validar en la placa** |
| E1 | `config.h`, cinemática, planificador, parser + tests | tests en PC OK |
| E2 | UART + consola microrl: `HELP`, `STATUS`, `SPEED`, `ACCEL` | test de consola en PC OK; **falta validar en la placa** |
| E3…E7 | PWM, AS5600, `MOVE`, Z | pendiente |

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

## Prueba de E0 en la placa

1. Compilar y grabar por SWD (ST-Link).
2. El LED PC13 debe parpadear a **1 Hz**: contar 60 destellos en 60 s con cronómetro (o medir 1,000 Hz en PC13).
   Si parpadea a 5 Hz, el cristal HSE no arrancó (el micro está con el HSI).
3. En el depurador, pausar y verificar: `SystemCoreClock == 96000000`; `RCC->CR` con `HSERDY` (bit 17) y
   `PLLRDY` (bit 25) en 1; `RCC->CFGR` con `SWS` (bits 3:2) = `0b10` (PLL); `FLASH->ACR & 0xF == 3`.

## Prueba de E2 en la placa

Conexión: adaptador USB-serie **de 3,3 V** con TX del adaptador → PA10, RX del adaptador → PA9 y GND común.
Terminal (PuTTY o Tera Term): 115200 baudios, 8N1, sin control de flujo.

1. Grabar y resetear. Debe aparecer `SCARA v2 - consola lista. HELP para ver los comandos.` y el prompt `SCARA>`.
   Si aparece basura: revisar baudios del terminal y que E0 haya pasado (reloj a 96 MHz).
2. `HELP` → lista de comandos y `OK`.
3. `STATUS` → `OK estado=IDLE speed=30.0 accel=80.0 tick_max_ms=20 rx_perdidos=0 tx_perdidos=0 reloj=HSE`.
   Si en vez de `30.0` sale vacío, falta *Use float with printf from newlib-nano*.
4. `SPEED 50` → `OK speed=50.0`; `SPEED 999` → `ERR 6 speed fuera de [1.0, 150.0] mm/s`;
   `ACCEL 120` → `OK accel=120.0`; `HOLA` → `ERR 1 comando desconocido ...`.
5. Edición: escribir `STTUS`, flecha izquierda tres veces, escribir `A`, Enter → responde como `STATUS`.
   Flecha arriba recupera el comando anterior; Backspace borra.
6. Pegar de una vez (clic derecho en PuTTY) estas 10 líneas, unos 100 caracteres en total:
   ```
   speed 11
   speed 12
   speed 13
   speed 14
   speed 15
   speed 16
   speed 17
   speed 18
   speed 19
   speed 20
   ```
   Deben salir 10 `OK speed=...` en orden, del 11 al 20.
7. `STATUS` otra vez → `speed=20.0`, `rx_perdidos=0`, `tx_perdidos=0` y `tick_max_ms` en 20 o 21
   (el tick de 20 ms no se atrasó mientras se procesaba lo pegado).

Anotar: lo que responde `STATUS` en los pasos 3 y 7, y si algún paso no dio lo esperado.

## Datos a medir en el brazo

No hacen falta para E0 ni E2, pero sí para E3 (servos). Aprovechar mientras se tiene el brazo enfrente,
con los servos **sin alimentar** (moviendo las articulaciones a mano). Hoy `app/config.h` tiene valores por defecto.

| Dato | Cómo medirlo | Default en `config.h` | Medido |
|---|---|---|---|
| L1: eje del hombro → eje del codo | centro a centro de los ejes, con calibre | 200 mm | |
| L2: eje del codo → centro de la pinza | centro a centro | 200 mm | |
| Tope del hombro hacia la derecha (−) | ángulo respecto de +X donde choca | −90° | |
| Tope del hombro hacia la izquierda (+) | ídem | +90° | |
| Tope del codo hacia cada lado | ángulo respecto del brazo recto | ±90° | |
| Codo recto: ¿en qué ángulo de servo? | ¿90° (rango ±90) o 180° (rango 0…−180)? | 90° | |
| Hombro: ¿el servo a 90° apunta a +X? | el TP2 usaba 0° = +X; el SPEC dice 90° = +X | 90° = +X | |
| Sentido de giro de cada servo | al subir el pulso, ¿gira antihorario visto desde arriba? | antihorario (+1) | |
| Distancia mínima pinza–base pedida | ¿el profesor exige llegar a 5 cm de la base? | — | |

Si no se puede medir algo, dejarlo en blanco: sigue el valor por defecto.

## Qué reportar

Para dar por aprobadas E0 y E2, pasar:

1. **E0:** destellos contados en 60 s, y los registros si se usó el depurador (`SystemCoreClock`, `RCC->CR`,
   `RCC->CFGR`, `FLASH->ACR`).
2. **E2:** la respuesta de `STATUS` en los pasos 3 y 7, y qué pasos no dieron lo esperado (con lo que apareció).
3. **Brazo:** la tabla de "Datos a medir" completa (lo que se pudo).
4. Si algo se reinicia solo o los servos tiemblan: anotarlo. Primero descartar la alimentación (fuente de los servos,
   GND común con la Black Pill, capacitor cerca de los servos), que suele parecer un bug de software.
