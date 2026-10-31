/**
 * @file vl53l0x.h
 * @brief Dos sensores de distancia VL53L0X (time of flight) en I2C1, con XSHUT en PA8 y PB7.
 *
 * Salen de fabrica con la misma direccion: al arrancar se despierta el A solo, se le cambia la
 * direccion y recien entonces se despierta el B. Cada XSHUT devuelve su sensor a la direccion de
 * fabrica, asi que la secuencia sirve tambien despues de un reset del micro con los sensores
 * encendidos. Sin el A, el B queda en la de fabrica.
 *
 * La inicializacion (bloqueante, solo al arrancar) es la secuencia de la API de ST reducida a
 * lo necesario. Las mediciones no bloquean: se inicia una y se consulta tick a tick.
 */
#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdbool.h>
#include <stdint.h>

/** Sensores de la garra. */
typedef enum {
	VL53_A = 0,		/**< XSHUT en PA8; direccion 0x2A */
	VL53_B,			/**< XSHUT en PB7; direccion 0x29 */
	VL53_CANT
} vl53_id_t;

/** Resultado de consultar una medicion. */
typedef enum {
	VL53_EN_CURSO = 0,	/**< Todavia midiendo */
	VL53_LISTO,			/**< Medicion valida en la salida */
	VL53_SIN_OBJETO,	/**< El sensor no recibio retorno: no hay nada al alcance */
	VL53_ERROR			/**< Fallo el I2C, se agoto el tiempo o no hay medicion en curso */
} vl53_res_t;

/**
 * @brief Reinicia los dos sensores por XSHUT y deja listos los que contestan.
 *
 * Bloquea unas decenas de ms por sensor presente; sin sensores vuelve de inmediato.
 * Requiere el SysTick y el bus I2C1 ya inicializados.
 * @return Cantidad de sensores listos (0, 1 o 2).
 */
int VL53L0X_Init(void);

/** @brief true si ese sensor paso la ultima VL53L0X_Init(). */
bool VL53L0X_Listo(vl53_id_t id);

/**
 * @brief Arranca una medicion de un sensor. Tarda ~30 ms en completarse; no espera.
 * @return false si el sensor no esta listo o fallo el I2C.
 */
bool VL53L0X_IniciarMedicion(vl53_id_t id);

/**
 * @brief Consulta la medicion en curso de un sensor. Llamar periodicamente despues de iniciarla.
 * @param id Sensor.
 * @param mm Salida: distancia en mm, valida solo con VL53_LISTO.
 * @return Estado de la medicion. Salvo VL53_EN_CURSO, la medicion queda terminada.
 */
vl53_res_t VL53L0X_Consultar(vl53_id_t id, uint16_t *mm);

#endif /* VL53L0X_H */
