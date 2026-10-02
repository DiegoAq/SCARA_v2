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
| E2…E7 | consola, PWM, AS5600, `MOVE`, Z | pendiente |

## Carpetas

- `app/` — aplicación. `cinematica`, `planificador`, `parser` y `config.h` son el **núcleo portable** (sin hardware).
- `drivers/` — periféricos por registros (CMSIS, sin HAL).
- `cmsis/` — headers CMSIS y `system_stm32f4xx.c` del paquete ST (STM32Cube_FW_F4 V1.28.3), sin modificar.
- `Startup/`, `Src/` — startup, y stubs de newlib (`syscalls.c`, `sysmem.c`) de CubeIDE.
- `tests/` — tests en PC y `sim_move` (excluida del build de CubeIDE).

## Compilar el firmware

STM32CubeIDE: *File → Open Projects from File System* → esta carpeta → *Build*.
El `.cproject` ya tiene las carpetas `app/`, `drivers/`, `cmsis/`, el símbolo `STM32F411xE` y float ABI *hard*.

## Tests en PC

Necesitan `gcc` y `make` en el PATH (p. ej. el MinGW de Qt y el `make` de CubeIDE), desde Git Bash:

```sh
cd tests
make test                                   # cinemática, planificador y parser
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
