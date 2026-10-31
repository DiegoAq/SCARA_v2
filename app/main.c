/**
 * @file main.c
 * @brief Firmware del robot SCARA: super-loop sin RTOS.
 *
 * Reloj, SysTick, LED de estado, consola por USART1 y un tick de control de 20 ms que avanza
 * el brazo, el eje Z, la garra y la rutina en curso, en ese orden.
 */
#include <stdbool.h>
#include <stdint.h>

#include "clock.h"
#include "config.h"
#include "console.h"
#include "eje_z.h"
#include "gpio.h"
#include "gripper.h"
#include "motion.h"
#include "secuencia.h"
#include "systick.h"
#include "uart.h"
#include "vl53l0x.h"

int main(void)
{
	bool reloj_ok = Clock_Init();

	(void)SysTick_Init();
	GPIO_LedInit();
	UART_Init();
	Motion_Init();
	Gripper_Init();
	EjeZ_Init();
	/* Despues de Motion_Init, que inicializa I2C1. Si un sensor no esta, se informa en STATUS. */
	(void)VL53L0X_Init();
	Secuencia_Init();
	Consola_Init(reloj_ok);

	/* Si el cristal no arranco el LED parpadea rapido, ademas del WARN por consola. */
	const uint32_t semiperiodo_ms = (reloj_ok ? LED_PERIODO_MS : LED_PERIODO_FALLA_MS) / 2U;
	uint32_t t_led = Millis();
	uint32_t t_tick = Millis();

	for (;;) {
		Consola_Procesar();

		/* Por diferencia (inmune al desborde de Millis) y sumando el periodo en vez de
		 * re-leer Millis: asi el periodo no acumula el atraso del loop. */
		if ((uint32_t)(Millis() - t_tick) >= TICK_MS) {
			t_tick += TICK_MS;
			/* Si algo freno el loop mas de un periodo (p. ej. RAW LIVE), los ticks perdidos
			 * se descartan. Ejecutarlos todos juntos para "recuperar" moveria el brazo de
			 * golpe todo lo atrasado, salteando el limite de velocidad de los servos. */
			if ((uint32_t)(Millis() - t_tick) >= TICK_MS) {
				t_tick = Millis();
			}
			Motion_Tick();
			EjeZ_Tick();
			Gripper_Tick();
			Secuencia_Tick();
			Consola_DespuesDelTick();
		}

		if ((uint32_t)(Millis() - t_led) >= semiperiodo_ms) {
			t_led += semiperiodo_ms;
			GPIO_LedConmutar();
		}
	}
}
