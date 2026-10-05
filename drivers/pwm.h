/**
 * @file pwm.h
 * @brief Senal de servo: 50 Hz (periodo de 20 ms) con resolucion de 1 us.
 *
 * TIM3_CH1 = PA6 (hombro), TIM3_CH2 = PA7 (codo), TIM4_CH1 = PB6 (gripper).
 * Con preload de ARR y CCR: un cambio de pulso se aplica entero al empezar el periodo
 * siguiente, nunca deforma el pulso en curso.
 *
 * Una salida deshabilitada queda en nivel bajo permanente (sin pulsos): el servo no
 * recibe senal y no hace fuerza.
 */
#ifndef PWM_H
#define PWM_H

#include <stdbool.h>
#include <stdint.h>

/** Salidas de servo. */
typedef enum {
	PWM_SERVO_HOMBRO = 0,	/**< PA6, TIM3_CH1 */
	PWM_SERVO_CODO,			/**< PA7, TIM3_CH2 */
	PWM_SERVO_GRIPPER,		/**< PB6, TIM4_CH1 */
	PWM_CANT
} pwm_id_t;

/**
 * @brief Configura TIM3 y TIM4 y deja las tres salidas deshabilitadas (nivel bajo).
 */
void PWM_Init(void);

/**
 * @brief Fija el ancho de pulso de una salida.
 *
 * Si la salida esta habilitada, el pulso nuevo sale en el periodo siguiente. Si no,
 * queda guardado para cuando se habilite.
 * @param id Salida.
 * @param pulso_us Ancho en microsegundos; se satura a [PULSE_MIN_US, PULSE_MAX_US].
 */
void PWM_ConfigurarPulsoUs(pwm_id_t id, uint32_t pulso_us);

/**
 * @brief Habilita o deshabilita los pulsos de una salida.
 * @param habilitar true: empieza a salir el ultimo pulso configurado; false: nivel bajo.
 */
void PWM_Habilitar(pwm_id_t id, bool habilitar);

#endif /* PWM_H */
