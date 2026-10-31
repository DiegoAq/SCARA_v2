/**
 * @file i2c.h
 * @brief I2C maestro por polling (I2C1 e I2C2), con timeout en toda espera y recuperacion de bus.
 *
 * I2C1: PB8 SCL / PB9 SDA.  I2C2: PB10 SCL / PB3 SDA.
 * Ninguna funcion espera mas que I2C_TIMEOUT_MS por bandera: si el esclavo no contesta
 * o el bus esta trabado, vuelve con error. Llamar solo desde el loop principal.
 */
#ifndef I2C_H
#define I2C_H

#include <stddef.h>
#include <stdint.h>

/** Buses disponibles. */
typedef enum {
	I2C_BUS_1 = 0,		/**< I2C1: PB8 SCL / PB9 SDA */
	I2C_BUS_2,			/**< I2C2: PB10 SCL / PB3 SDA */
	I2C_CANT_BUSES
} i2c_bus_t;

/** Resultado de una operacion. */
typedef enum {
	I2C_OK = 0,
	I2C_ERR_NACK,		/**< Nadie contesto en esa direccion (o el esclavo rechazo el dato) */
	I2C_ERR_TIMEOUT,	/**< Una bandera no llego a tiempo: bus trabado, sin pull-ups o sin esclavo */
	I2C_ERR_BUS,		/**< Error de bus o perdida de arbitraje; o el bus quedo ocupado */
	I2C_ERR_PARAM		/**< Bus inexistente, puntero NULL o longitud 0 */
} i2c_res_t;

/**
 * @brief Configura los pines y el periferico a I2C_FRECUENCIA_HZ.
 *
 * Si el bus arranca ocupado (tipico del F4 despues de un reset a mitad de una
 * transferencia) lo destraba con 9 pulsos de SCL. Requiere el SysTick andando.
 * @return I2C_OK, o I2C_ERR_BUS si el bus sigue ocupado despues de la recuperacion.
 */
i2c_res_t I2C_Init(i2c_bus_t bus);

/**
 * @brief Lee @p n bytes consecutivos a partir de un registro del esclavo.
 * @param bus Bus a usar.
 * @param dir7 Direccion de 7 bits del esclavo.
 * @param reg Registro inicial.
 * @param datos Salida, @p n bytes.
 * @param n Cantidad de bytes (>= 1).
 * @return I2C_OK o el motivo de la falla. Tras un timeout se reinicia el periferico.
 */
i2c_res_t I2C_LeerRegistros(i2c_bus_t bus, uint8_t dir7, uint8_t reg, uint8_t *datos, size_t n);

/**
 * @brief Escribe @p n bytes consecutivos a partir de un registro del esclavo.
 * @param bus Bus a usar.
 * @param dir7 Direccion de 7 bits del esclavo.
 * @param reg Registro inicial.
 * @param datos Bytes a escribir.
 * @param n Cantidad de bytes (>= 1).
 * @return I2C_OK o el motivo de la falla. Tras un timeout se reinicia el periferico.
 */
i2c_res_t I2C_EscribirRegistros(i2c_bus_t bus, uint8_t dir7, uint8_t reg, const uint8_t *datos, size_t n);

/**
 * @brief Verifica si un esclavo contesta en una direccion (para I2CSCAN).
 * @return I2C_OK si hubo ACK, I2C_ERR_NACK si no hay nadie, u otro error de bus.
 */
i2c_res_t I2C_Sondear(i2c_bus_t bus, uint8_t dir7);

#endif /* I2C_H */
