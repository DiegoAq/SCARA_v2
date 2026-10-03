/**
 * @file as5600.h
 * @brief Encoder magnetico absoluto AS5600 (12 bits, I2C, direccion fija 0x36).
 *
 * Los dos AS5600 tienen la misma direccion, por eso va uno en cada bus:
 * hombro en I2C1 y codo en I2C2. Este driver entrega la lectura cruda del sensor;
 * la conversion a angulo de junta (cero y signo) se hace en motion.
 */
#ifndef AS5600_H
#define AS5600_H

#include <stdbool.h>
#include <stdint.h>

#include "i2c.h"

#define AS5600_DIRECCION		0x36U
#define AS5600_CUENTAS_VUELTA	4096U	/**< 12 bits: 0,088 grados por cuenta */

/** Estado del iman, segun el registro STATUS. */
typedef enum {
	AS5600_IMAN_OK = 0,		/**< Detectado y con campo dentro de rango (MD=1, ML=0, MH=0) */
	AS5600_IMAN_AUSENTE,	/**< MD=0: no hay iman */
	AS5600_IMAN_DEBIL,		/**< ML=1: iman muy lejos o muy debil */
	AS5600_IMAN_FUERTE,		/**< MH=1: iman muy cerca o muy fuerte */
	AS5600_SIN_RESPUESTA	/**< El sensor no contesta por I2C */
} as5600_iman_t;

/**
 * @brief Inicializa el bus y verifica que el sensor conteste.
 * @return false si el bus no se pudo inicializar o no hay respuesta en 0x36.
 */
bool AS5600_Init(i2c_bus_t bus);

/**
 * @brief Lee el angulo en cuentas (registro ANGLE, 0x0E/0x0F).
 * @param cuentas Salida: 0..4095.
 * @return false si fallo la comunicacion. No verifica el iman.
 */
bool AS5600_LeerCuentas(i2c_bus_t bus, uint16_t *cuentas);

/**
 * @brief Lee el angulo en grados.
 * @param grados Salida: 0 <= grados < 360, resolucion 0,088.
 * @return false si fallo la comunicacion. No verifica el iman.
 */
bool AS5600_LeerAngulo(i2c_bus_t bus, float *grados);

/**
 * @brief Estado del iman (registro STATUS, 0x0B).
 */
as5600_iman_t AS5600_EstadoIman(i2c_bus_t bus);

/**
 * @brief true si el iman esta detectado y dentro de rango. Verificar antes de confiar en la lectura.
 */
bool AS5600_ImanOK(i2c_bus_t bus);

#endif /* AS5600_H */
