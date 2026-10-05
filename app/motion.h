/**
 * @file motion.h
 * @brief Movimiento del brazo: estado, tick de control de 20 ms y comandos HOME/JOINT/PULSE/MOVE/STOP.
 *
 * LAZO ABIERTO (sin encoders): el firmware no mide donde esta el brazo. Al arrancar supone
 * la postura de HOME y los servos estan sin pulsos; el primer Motion_Home() los habilita en
 * esa postura. A partir de ahi todo movimiento sale de la ultima postura comandada y ninguna
 * junta se mueve a mas de JOINT_VMAX_DPS.
 *
 * Los AS5600 quedan solo como diagnostico (ANGLE, RAW): no intervienen en el movimiento.
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

/** Resultado de un comando de movimiento. */
typedef enum {
	MOTION_OK = 0,				/**< Aceptado; el fin se avisa con un evento */
	MOTION_YA_EN_DESTINO,		/**< No hay nada que mover */
	MOTION_SERVOS_HABILITADOS,	/**< Primer HOME: se habilito el PWM en la postura de HOME */
	MOTION_ERR_RADIO,			/**< Destino o tramo fuera del espacio de trabajo */
	MOTION_ERR_LIMITE,			/**< Limite articular */
	MOTION_ERR_OCUPADO,			/**< Hay un movimiento en curso */
	MOTION_ERR_APAGADO,			/**< Servos sin habilitar: falta el primer HOME */
	MOTION_ERR_PARAM			/**< Argumento invalido */
} motion_res_t;

/** Avisos asincronos, para que la consola los informe. */
typedef enum {
	MOTION_EVENTO_NINGUNO = 0,
	MOTION_EVENTO_FIN,			/**< El movimiento llego a destino */
	MOTION_EVENTO_DETENIDO,		/**< Termino de frenar tras un STOP */
	MOTION_EVENTO_FALLA			/**< El planificador aborto; el brazo quedo donde estaba */
} motion_evento_t;

/**
 * @brief Parametros por defecto, estado IDLE, servos sin pulsos y encoders inicializados.
 *        Llamar despues de SysTick_Init() (el I2C usa timeouts por Millis).
 */
void Motion_Init(void);

/**
 * @brief Tick de control: avanza un paso el movimiento en curso.
 *
 * Llamar cada TICK_MS desde el loop principal (nunca desde una ISR). Si el loop se atrasa
 * NO hay que llamarlo varias veces seguidas para "recuperar": cada llamada avanza el brazo
 * un paso completo.
 */
void Motion_Tick(void);

/**
 * @brief HOME. La primera vez habilita el PWM directamente en la postura de HOME (el brazo
 *        salta hasta ahi si estaba en otro lado); despues vuelve a HOME a JOINT_MOVE_DPS.
 * @return MOTION_SERVOS_HABILITADOS, MOTION_OK, MOTION_YA_EN_DESTINO o MOTION_ERR_OCUPADO.
 */
motion_res_t Motion_Home(void);

/**
 * @brief Mueve las dos juntas a los angulos pedidos, sincronizadas, a JOINT_MOVE_DPS.
 * @param th1_deg,th2_deg Angulos de junta (grados), dentro de los limites articulares.
 */
motion_res_t Motion_MoverJuntas(float th1_deg, float th2_deg);

/**
 * @brief Mueve la pinza en linea recta hasta (x, y) con el perfil SPEED/ACCEL vigente.
 *
 * Valida todo el segmento antes de mover (pre-vuelo). Si falla, el brazo no se mueve.
 * @param x_mm,y_mm Destino (mm).
 */
motion_res_t Motion_MoverLineal(float x_mm, float y_mm);

/**
 * @brief Lleva un servo a un pulso crudo, a JOINT_MOVE_DPS (diagnostico y calibracion).
 *
 * No mira los limites articulares: sirve justamente para encontrarlos. Usar con cuidado.
 * @param junta Servo a mover.
 * @param pulso_us Pulso destino; se satura a [PULSE_MIN_US, PULSE_MAX_US].
 */
motion_res_t Motion_MoverPulso(junta_id_t junta, uint32_t pulso_us);

/**
 * @brief STOP. Un MOVE frena con rampa sobre la recta; un movimiento articular se detiene ya.
 */
void Motion_Detener(void);

/**
 * @brief Devuelve (una sola vez) el aviso pendiente, o MOTION_EVENTO_NINGUNO.
 */
motion_evento_t Motion_TomarEvento(void);

/** @brief Estado actual del brazo. */
motion_estado_t Motion_Estado(void);

/** @brief Nombre del estado para la consola ("IDLE", "MOVING", ...). */
const char *Motion_NombreEstado(motion_estado_t estado);

/** @brief true si ya se hizo el primer HOME y los servos reciben pulsos. */
bool Motion_ServosActivos(void);

/**
 * @brief Ultima postura comandada (grados). Antes del primer HOME es la postura supuesta.
 */
void Motion_PosturaComandada(float *th1_deg, float *th2_deg);

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

/**
 * @brief Lee la posicion real de una junta con su AS5600 (solo diagnostico).
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

#endif /* MOTION_H */
