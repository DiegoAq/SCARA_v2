/**
 * @file as5600.h
 * @brief Encoder magnetico absoluto AS5600 leido por su salida analogica OUT.
 *
 * La salida OUT entrega 0 V .. VDD para 0 .. 360 grados. Se lee con el ADC1:
 * hombro en PB0 (IN8) y codo en PB1 (IN9). El sensor debe alimentarse con 3,3 V,
 * la misma tension que la referencia del ADC.
 *
 * Limitaciones frente a la lectura por I2C:
 *  - No se puede consultar el estado del iman (ausente / debil / fuerte) ni detectar un
 *    cable suelto: un pin al aire da lecturas al azar. Se controla mirando el ruido (RAW).
 *  - En el cruce 0/360 la salida salta de VDD a 0 V. El promedio lo tiene en cuenta, pero
 *    conviene montar el iman para que el cruce quede fuera del recorrido de la junta.
 *
 * Este driver entrega la lectura cruda; la conversion a angulo de junta se hace en motion.
 */
#ifndef AS5600_H
#define AS5600_H

#include <stdbool.h>
#include <stdint.h>

#define AS5600_CUENTAS_VUELTA	4096U	/**< Cuentas del ADC por vuelta (0,088 grados por cuenta) */

/** Sensores disponibles. */
typedef enum {
	AS5600_HOMBRO = 0,	/**< OUT en PB0 */
	AS5600_CODO,		/**< OUT en PB1 */
	AS5600_CANT
} as5600_id_t;

/**
 * @brief Inicializa el ADC. Llamar despues de SysTick_Init().
 */
void AS5600_Init(void);

/**
 * @brief Lee la posicion en cuentas, promediando ENC_MUESTRAS conversiones.
 * @param sensor Sensor a leer.
 * @param cuentas Salida: 0..4095.
 * @param dispersion Salida opcional (puede ser NULL): maximo menos minimo de las muestras
 *                   promediadas, en cuentas. Mide el ruido de la lectura.
 * @return false si el ADC no contesto.
 */
bool AS5600_LeerCuentas(as5600_id_t sensor, uint16_t *cuentas, uint16_t *dispersion);

/**
 * @brief Lee la posicion en grados.
 * @param grados Salida: 0 <= grados < 360.
 * @return false si el ADC no contesto.
 */
bool AS5600_LeerAngulo(as5600_id_t sensor, float *grados);

#endif /* AS5600_H */
