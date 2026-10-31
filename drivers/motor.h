/**
 * @file motor.h
 * @brief Motores DC del eje Z por L298N: PWM de 10 kHz en los enables y sentido por GPIO.
 *
 * Motor A: ENA = PA0 (TIM2_CH1), IN1 = PA2, IN2 = PA3.
 * Motor B: ENB = PA1 (TIM2_CH2), IN3 = PA4, IN4 = PA5.
 * PA0 es ademas el boton KEY de la Black Pill: no pulsarlo con los motores conectados.
 */
#ifndef MOTOR_H
#define MOTOR_H

/** Motores del eje diferencial. */
typedef enum {
	MOTOR_A = 0,
	MOTOR_B,
	MOTOR_CANT
} motor_id_t;

/**
 * @brief Configura TIM2 y los pines. Los motores quedan sueltos (enable en bajo).
 */
void Motor_Init(void);

/**
 * @brief Fija duty y sentido de un motor.
 * @param id Motor.
 * @param duty Fraccion de -1 a 1; el signo elige el sentido. Se satura a ese rango.
 */
void Motor_ConfigurarDuty(motor_id_t id, float duty);

/**
 * @brief Frena un motor poniendo sus dos bornes al mismo potencial (enable en alto).
 *
 * Corta la inercia mucho antes que dejarlo suelto, y con encoder de un canal eso reduce
 * los pulsos que llegan despues de la orden de parar.
 */
void Motor_Frenar(motor_id_t id);

#endif /* MOTOR_H */
