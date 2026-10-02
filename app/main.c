/**
 * @file main.c
 * @brief Firmware del robot SCARA: super-loop sin RTOS.
 *
 * Etapa E0: reloj a 96 MHz, SysTick de 1 ms y LED de estado.
 */
#include <stdbool.h>
#include <stdint.h>

#include "clock.h"
#include "config.h"
#include "gpio.h"
#include "systick.h"

int main(void)
{
	bool reloj_ok = Clock_Init();

	(void)SysTick_Init();
	GPIO_LedInit();

	/* Si el cristal no arranco el LED parpadea rapido: es la unica senal visible
	 * hasta que haya consola. */
	const uint32_t semiperiodo_ms = (reloj_ok ? LED_PERIODO_MS : LED_PERIODO_FALLA_MS) / 2U;
	uint32_t t_led = Millis();

	for (;;) {
		/* Por diferencia (inmune al desborde de Millis) y sumando el semiperiodo en
		 * vez de re-leer Millis: asi el periodo no acumula el atraso del loop. */
		if ((uint32_t)(Millis() - t_led) >= semiperiodo_ms) {
			t_led += semiperiodo_ms;
			GPIO_LedConmutar();
		}
	}
}
