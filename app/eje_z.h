/**
 * @file eje_z.h
 * @brief Eje Z y orientacion de la garra: dos motores DC diferenciales sobre la misma varilla.
 *
 * Mismo sentido en los dos motores: la varilla sube o baja. Sentidos opuestos: gira sobre su eje.
 * Los dos motores siguen una misma trayectoria con rampa y un lazo proporcional sobre los
 * pulsos de cada uno, asi la varilla no se tuerce mientras sube.
 *
 * Sin finales de carrera: SETZ0 (EjeZ_FijarCero) declara el cero. Hasta entonces, y despues de
 * un atasco o de ZTEST, no se acepta ningun movimiento.
 */
#ifndef EJE_Z_H
#define EJE_Z_H

#include <stdbool.h>
#include <stdint.h>

#include "motor.h"

/** Resultado de un comando del eje. */
typedef enum {
	EJEZ_OK = 0,			/**< Aceptado; el fin se avisa con un evento */
	EJEZ_YA_EN_DESTINO,		/**< No hay nada que mover */
	EJEZ_ERR_OCUPADO,		/**< Hay un movimiento en curso */
	EJEZ_ERR_RANGO,			/**< Fuera de los limites de config.h */
	EJEZ_ERR_SIN_CERO		/**< Falta SETZ0 */
} ejez_res_t;

/** Avisos asincronos, para que los informe la consola o la secuencia. */
typedef enum {
	EJEZ_EVENTO_NINGUNO = 0,
	EJEZ_EVENTO_FIN,		/**< Llego a destino */
	EJEZ_EVENTO_DETENIDO,	/**< Termino de frenar tras un STOP */
	EJEZ_EVENTO_ATASCO,		/**< Empujaba sin recibir pulsos: se corto y se perdio la referencia */
	EJEZ_EVENTO_PRUEBA_FIN	/**< Termino ZTEST */
} ejez_evento_t;

/**
 * @brief Configura motores y encoders. El eje queda quieto y sin referencia.
 */
void EjeZ_Init(void);

/**
 * @brief Tick de control. Llamar cada TICK_MS desde el loop principal.
 */
void EjeZ_Tick(void);

/**
 * @brief Declara la posicion actual como Z = 0 y orientacion = 0.
 * @return EJEZ_OK, o EJEZ_ERR_OCUPADO si el eje se esta moviendo.
 */
ejez_res_t EjeZ_FijarCero(void);

/**
 * @brief Busqueda de cero con finales de carrera. Reservada: todavia no estan instalados.
 * @return false siempre; usar SETZ0.
 */
bool EjeZ_Homing(void);

/**
 * @brief Lleva el eje a una altura, sin cambiar la orientacion.
 * @param z_mm Altura en mm, dentro de [Z_MIN_MM, Z_MAX_MM]. +Z es hacia arriba.
 */
ejez_res_t EjeZ_MoverAltura(float z_mm);

/**
 * @brief Gira la garra a una orientacion, sin cambiar la altura.
 * @param rot_deg Orientacion en grados, dentro de [ROT_MIN_DEG, ROT_MAX_DEG].
 */
ejez_res_t EjeZ_MoverOrientacion(float rot_deg);

/**
 * @brief Prueba a lazo abierto para calibrar: un motor, duty fijo, tiempo fijo. Pierde la referencia.
 * @param motor Motor a mover.
 * @param duty_pct Duty en %, con signo, hasta +-ZTEST_PWM_MAX_PCT y distinto de 0.
 * @param ms Duracion, de 1 a ZTEST_MS_MAX.
 */
ejez_res_t EjeZ_Probar(motor_id_t motor, float duty_pct, int32_t ms);

/**
 * @brief STOP: frena con rampa sobre la trayectoria. ZTEST se corta en el acto.
 */
void EjeZ_Detener(void);

/** @brief true mientras hay un movimiento en curso. */
bool EjeZ_Ocupado(void);

/** @brief true si hay cero declarado y no se perdio. */
bool EjeZ_Referenciado(void);

/** @brief Devuelve (una sola vez) el aviso pendiente, o EJEZ_EVENTO_NINGUNO. */
ejez_evento_t EjeZ_TomarEvento(void);

/** @brief Altura medida con los pulsos de los dos motores (mm). */
float EjeZ_Altura(void);

/** @brief Orientacion medida con los pulsos de los dos motores (grados). */
float EjeZ_Orientacion(void);

/** @brief Ultima altura comandada (mm). */
float EjeZ_AlturaComandada(void);

/** @brief Ultima orientacion comandada (grados). */
float EjeZ_OrientacionComandada(void);

/** @brief Pulsos que conto el motor de la ultima prueba desde que empezo (modulo). */
uint32_t EjeZ_PulsosPrueba(void);

/** @brief Motor de la ultima prueba. */
motor_id_t EjeZ_MotorPrueba(void);

#endif /* EJE_Z_H */
