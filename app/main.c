/**
 * @file main.c
 * @brief Firmware del robot SCARA: super-loop sin RTOS.
 *
 * Etapa E2: reloj, SysTick, LED de estado, consola por USART1 y tick de control de 20 ms.
 */
#include <stdbool.h>
#include <stdint.h>

#include "clock.h"
#include "config.h"
#include "console.h"
#include "gpio.h"
#include "motion.h"
#include "systick.h"
#include "uart.h"

int main(void)
{
	bool reloj_ok = Clock_Init();

	(void)SysTick_Init();
	GPIO_LedInit();
	UART_Init();
	Motion_Init();
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
			Motion_Tick();
		}

		if ((uint32_t)(Millis() - t_led) >= semiperiodo_ms) {
			t_led += semiperiodo_ms;
			GPIO_LedConmutar();
		}
	}
}
