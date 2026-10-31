/**
 * @file gripper.h
 * @brief Garra: servo MG90S en PB6, apertura en mm (15 a 60 segun las pautas).
 *
 * Sin realimentacion. El PWM sale recien con el primer comando; ese primer pulso es la apertura
 * supuesta (GRIPPER_INICIO_MM), asi que si la garra estaba en otro lado da un salto. Desde ahi
 * se mueve siempre con rampa, sin superar GRIPPER_DPS.
 */
#ifndef GRIPPER_H
#define GRIPPER_H

#include <stdbool.h>
#include <stdint.h>

/** Resultado de un comando de la garra. */
typedef enum {
	GRIPPER_OK = 0,				/**< Aceptado; el fin se avisa con Gripper_TomarFin() */
	GRIPPER_YA_EN_DESTINO,		/**< No hay nada que mover */
	GRIPPER_ERR_OCUPADO,		/**< Hay un movimiento en curso */
	GRIPPER_ERR_RANGO			/**< Fuera de [GRIPPER_MIN_MM, GRIPPER_MAX_MM] */
} gripper_res_t;

/** @brief Estado inicial, sin pulsos. Llamar despues de Servo_Init() (configura el PWM). */
void Gripper_Init(void);

/** @brief Tick de control. Llamar cada TICK_MS desde el loop principal. */
void Gripper_Tick(void);

/**
 * @brief Lleva la garra a una apertura.
 * @param apertura_mm Apertura en mm, dentro de [GRIPPER_MIN_MM, GRIPPER_MAX_MM].
 */
gripper_res_t Gripper_MoverApertura(float apertura_mm);

/**
 * @brief Lleva el servo a un pulso crudo, con la misma rampa (para calibrar con PULSE 3).
 * @param pulso_us Pulso destino; se satura a [PULSE_MIN_US, PULSE_MAX_US].
 */
gripper_res_t Gripper_MoverPulso(uint32_t pulso_us);

/** @brief STOP: frena con rampa. */
void Gripper_Detener(void);

/** @brief true mientras hay un movimiento en curso. */
bool Gripper_Ocupado(void);

/** @brief true una sola vez despues de terminar un movimiento. */
bool Gripper_TomarFin(void);

/** @brief Apertura que corresponde al pulso actual (mm). */
float Gripper_Apertura(void);

/** @brief Pulso actual (us). Mientras no hubo un primer comando, el servo no recibe pulsos. */
uint32_t Gripper_Pulso(void);

/** @brief true si el servo ya recibe pulsos. */
bool Gripper_Activo(void);

#endif /* GRIPPER_H */
