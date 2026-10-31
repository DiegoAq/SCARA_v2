/**
 * @file laseres.h
 * @brief Lecturas de los dos sensores de la garra, de a uno por vez.
 *
 * Los dos miran hacia abajo a pocos centimetros: si midieran juntos, cada uno podria recibir el
 * pulso del otro. Por eso miden por turnos y la lectura de cada uno ya sale con su correccion de
 * SENSOR_x_OFFSET_MM. Sirve tambien con uno solo de los dos montado.
 */
#ifndef LASERES_H
#define LASERES_H

#include <stdbool.h>

#include "vl53l0x.h"

/** Resultado de una vuelta de Laseres_Tick(). */
typedef enum {
	LASER_NADA = 0,		/**< No hay una lectura nueva */
	LASER_LECTURA,		/**< Termino una lectura */
	LASER_FALLA			/**< Un sensor no contesta o se agoto el tiempo */
} laser_res_t;

/** Una lectura terminada. */
typedef struct {
	vl53_id_t sensor;	/**< Quien midio */
	bool con_retorno;	/**< false: no hubo retorno y mm no vale */
	float mm;			/**< Distancia corregida (mm) */
} laser_lectura_t;

/** @brief Estado inicial, sin mediciones pedidas. */
void Laseres_Init(void);

/** @brief Cantidad de sensores que pasaron la inicializacion. */
int Laseres_Listos(void);

/**
 * @brief Empieza a medir por turnos, hasta Laseres_Detener().
 *
 * Si quedo una medicion en curso de un pedido anterior, se descarta: la tomo otra postura.
 * @return false si no hay ningun sensor listo o el sensor no contesto.
 */
bool Laseres_Arrancar(void);

/** @brief Deja de pedir mediciones. */
void Laseres_Detener(void);

/**
 * @brief Consulta la medicion en curso y arranca la siguiente. Llamar en cada tick.
 * @param l Salida, valida con LASER_LECTURA.
 */
laser_res_t Laseres_Tick(laser_lectura_t *l);

#endif /* LASERES_H */
