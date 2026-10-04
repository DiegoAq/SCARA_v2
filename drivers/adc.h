/**
 * @file adc.h
 * @brief ADC1 por polling, conversiones simples de 12 bits con timeout.
 *
 * Canales habilitados: IN8 (PB0) e IN9 (PB1), las salidas analogicas de los AS5600.
 * Una conversion tarda ~7 us. Llamar solo desde el loop principal.
 */
#ifndef ADC_H
#define ADC_H

#include <stdbool.h>
#include <stdint.h>

#define ADC_CANAL_PB0		8U		/**< ADC1_IN8 */
#define ADC_CANAL_PB1		9U		/**< ADC1_IN9 */
#define ADC_CUENTAS_MAX		4095U	/**< Fondo de escala: 12 bits, referencia VDDA (3,3 V) */

/**
 * @brief Configura PB0 y PB1 como entradas analogicas y enciende el ADC1.
 *
 * Espera la estabilizacion del ADC con Millis(): llamar despues de SysTick_Init().
 */
void ADC_Init(void);

/**
 * @brief Hace una conversion de un canal.
 * @param canal ADC_CANAL_PB0 o ADC_CANAL_PB1.
 * @param cuentas Salida: 0..4095 (0 V .. VDDA).
 * @return false si el canal no es valido o la conversion no termino en ADC_TIMEOUT_MS.
 */
bool ADC_LeerCanal(uint8_t canal, uint16_t *cuentas);

#endif /* ADC_H */
