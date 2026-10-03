/**
 * @file motion.h
 * @brief Estado del movimiento y tick de control de 20 ms.
 *
 * Hasta ahora: parametros de movimiento (SPEED, ACCEL), medicion de la regularidad del
 * tick y lectura de las juntas con los AS5600. La ejecucion de MOVE/JOINT/HOME se agrega
 * en E3 y E5.
 */
#ifndef MOTION_H
#define MOTION_H

#include <stdbool.h>
#include <stdint.h>

#include "as5600.h"

/** Estados del brazo (SPEC 7.2). */
typedef enum {
	MOTION_IDLE = 0,
	MOTION_MOVING,
	MOTION_STOPPING,
	MOTION_FAULT
} motion_estado_t;

/** Juntas del brazo. */
typedef enum {
	JUNTA_HOMBRO = 0,	/**< theta1, encoder en I2C1 */
	JUNTA_CODO,			/**< theta2, encoder en I2C2 */
	JUNTA_CANT
} junta_id_t;

/**
 * @brief Deja los parametros en sus valores por defecto, el estado en IDLE e inicializa
 *        los encoders. Llamar despues de SysTick_Init() (el I2C usa timeouts por Millis).
 */
void Motion_Init(void);

/**
 * @brief Lee la posicion real de una junta con su AS5600.
 *
 * Aplica ENCx_ZERO_DEG y ENCx_SIGN de config.h. No verifica el iman.
 * @param junta Junta a leer.
 * @param enc_deg Salida opcional (puede ser NULL): lectura cruda del encoder, 0..360 grados.
 * @param theta_deg Salida opcional (puede ser NULL): angulo de junta en (-180, 180].
 * @return false si el encoder no contesta.
 */
bool Motion_LeerJuntaMedida(junta_id_t junta, float *enc_deg, float *theta_deg);

/**
 * @brief Lectura cruda del encoder de una junta, para diagnostico (comando RAW).
 * @param cuentas Salida: 0..4095.
 * @return false si el encoder no contesta.
 */
bool Motion_LeerEncoderCrudo(junta_id_t junta, uint16_t *cuentas);

/**
 * @brief Estado del iman del encoder de una junta.
 */
as5600_iman_t Motion_EstadoIman(junta_id_t junta);

/**
 * @brief Tick de control. Llamar cada TICK_MS desde el loop principal (nunca desde una ISR).
 */
void Motion_Tick(void);

/**
 * @brief Estado actual del brazo.
 */
motion_estado_t Motion_Estado(void);

/**
 * @brief Nombre del estado para la consola ("IDLE", "MOVING", ...).
 */
const char *Motion_NombreEstado(motion_estado_t estado);

/**
 * @brief Fija la velocidad de crucero de MOVE.
 * @param v_mm_s Velocidad en mm/s, dentro de [SPEED_MIN_MM_S, SPEED_MAX_MM_S].
 * @return false (y no cambia nada) si esta fuera de rango o no es un numero.
 */
bool Motion_ConfigurarVelocidad(float v_mm_s);

/** @brief Velocidad de crucero vigente (mm/s). */
float Motion_Velocidad(void);

/**
 * @brief Fija la aceleracion (y frenado) de MOVE.
 * @param a_mm_s2 Aceleracion en mm/s2, dentro de [ACCEL_MIN_MM_S2, ACCEL_MAX_MM_S2].
 * @return false (y no cambia nada) si esta fuera de rango o no es un numero.
 */
bool Motion_ConfigurarAceleracion(float a_mm_s2);

/** @brief Aceleracion vigente (mm/s2). */
float Motion_Aceleracion(void);

/**
 * @brief Mayor intervalo medido entre dos ticks (ms). Idealmente TICK_MS.
 *
 * Si algo bloquea el loop, el tick se atrasa y esto lo muestra.
 * @param reiniciar true para empezar una medicion nueva despues de leer.
 */
uint32_t Motion_PeriodoTickMaxMs(bool reiniciar);

#endif /* MOTION_H */
