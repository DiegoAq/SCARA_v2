/**
 * @file servo.c
 * @brief Conversion junta <-> pulso y limites de los servos de junta.
 */
#include "servo.h"

#include "config.h"
#include "pwm.h"

/** Calibracion y salida de PWM de cada servo. */
typedef struct {
	pwm_id_t pwm;
	float zero_deg;		/**< Angulo de servo con theta = 0 */
	float sign;			/**< +1 o -1 */
	float range_deg;	/**< Recorrido real entre PULSE_MIN_US y PULSE_MAX_US */
	float theta_min;	/**< Limite articular inferior (grados) */
	float theta_max;	/**< Limite articular superior (grados) */
} servo_cfg_t;

static const servo_cfg_t servos[SERVO_CANT] = {
	[SERVO_HOMBRO] = { PWM_SERVO_HOMBRO, SHOULDER_ZERO_SERVO_DEG, SHOULDER_SIGN, SHOULDER_RANGE_DEG,
					   JOINT1_MIN_DEG, JOINT1_MAX_DEG },
	[SERVO_CODO]   = { PWM_SERVO_CODO, ELBOW_ZERO_SERVO_DEG, ELBOW_SIGN, ELBOW_RANGE_DEG,
					   JOINT2_MIN_DEG, JOINT2_MAX_DEG },
};

static float theta_cmd[SERVO_CANT];
static uint32_t pulso_cmd[SERVO_CANT];

void Servo_Init(void)
{
	PWM_Init();
	for (int i = 0; i < SERVO_CANT; i++) {
		pulso_cmd[i] = (PULSE_MIN_US + PULSE_MAX_US) / 2U;
		theta_cmd[i] = Servo_JuntaDePulso((servo_id_t)i, pulso_cmd[i]);
	}
}

void Servo_Habilitar(bool habilitar)
{
	for (int i = 0; i < SERVO_CANT; i++) {
		PWM_Habilitar(servos[i].pwm, habilitar);
	}
}

uint32_t Servo_PulsoDeJunta(servo_id_t id, float theta_deg)
{
	if (id >= SERVO_CANT) {
		return PULSE_MIN_US;
	}

	float servo_deg = servos[id].zero_deg + servos[id].sign * theta_deg;
	float us = (float)PULSE_MIN_US + servo_deg * Servo_UsPorGrado(id);

	/* Saturar antes de convertir a entero: un float negativo a uint32_t no esta definido. */
	if (us < (float)PULSE_MIN_US) {
		return PULSE_MIN_US;
	}
	if (us > (float)PULSE_MAX_US) {
		return PULSE_MAX_US;
	}
	return (uint32_t)(us + 0.5f);
}

float Servo_JuntaDePulso(servo_id_t id, uint32_t us)
{
	if (id >= SERVO_CANT) {
		return 0.0f;
	}

	/* SIGN vale +1 o -1: dividir por el es lo mismo que multiplicar. */
	return servos[id].sign * (Servo_AnguloDePulso(id, us) - servos[id].zero_deg);
}

float Servo_AnguloDePulso(servo_id_t id, uint32_t us)
{
	if (id >= SERVO_CANT) {
		return 0.0f;
	}
	return ((float)us - (float)PULSE_MIN_US) / Servo_UsPorGrado(id);
}

float Servo_UsPorGrado(servo_id_t id)
{
	const float rango = (id < SERVO_CANT) ? servos[id].range_deg : SERVO_RANGE_DEG;

	return (float)(PULSE_MAX_US - PULSE_MIN_US) / rango;
}

bool Servo_ConfigurarJunta(servo_id_t id, float theta_deg)
{
	/* Comparacion negada a proposito: tambien rechaza NaN. */
	if (id >= SERVO_CANT
			|| !(theta_deg >= servos[id].theta_min - JOINT_LIM_TOL_DEG
				 && theta_deg <= servos[id].theta_max + JOINT_LIM_TOL_DEG)) {
		return false;
	}

	theta_cmd[id] = theta_deg;
	pulso_cmd[id] = Servo_PulsoDeJunta(id, theta_deg);
	PWM_ConfigurarPulsoUs(servos[id].pwm, pulso_cmd[id]);
	return true;
}

void Servo_ConfigurarPulso(servo_id_t id, uint32_t us)
{
	if (id >= SERVO_CANT) {
		return;
	}
	if (us < PULSE_MIN_US) {
		us = PULSE_MIN_US;
	} else if (us > PULSE_MAX_US) {
		us = PULSE_MAX_US;
	}

	pulso_cmd[id] = us;
	theta_cmd[id] = Servo_JuntaDePulso(id, us);
	PWM_ConfigurarPulsoUs(servos[id].pwm, us);
}

float Servo_JuntaComandada(servo_id_t id)
{
	return (id < SERVO_CANT) ? theta_cmd[id] : 0.0f;
}

uint32_t Servo_PulsoComandado(servo_id_t id)
{
	return (id < SERVO_CANT) ? pulso_cmd[id] : 0U;
}
