/**
 * @file gripper.c
 * @brief Apertura de la garra con rampa sobre el pulso del servo.
 */
#include "gripper.h"

#include <math.h>

#include "config.h"
#include "planificador.h"
#include "pwm.h"

_Static_assert(GRIPPER_DPS <= JOINT_VMAX_DPS, "GRIPPER_DPS no puede superar JOINT_VMAX_DPS");

#define DT_S			((float)TICK_MS / 1000.0f)
#define US_POR_GRADO	((float)(PULSE_MAX_US - PULSE_MIN_US) / SERVO_RANGE_DEG)
#define DIST_MIN_US		0.5f

/** us de pulso por mm de apertura. */
#define US_POR_MM		((float)(GRIPPER_PULSO_MAX_MM_US - GRIPPER_PULSO_MIN_MM_US) \
						 / (GRIPPER_MAX_MM - GRIPPER_MIN_MM))

static bool activo;
static bool moviendo;
static bool fin_pendiente;
static perfil_t perfil;
static float ini_us;
static float fin_us;
static uint32_t pulso_us;

static uint32_t pulso_de_apertura(float mm)
{
	return (uint32_t)((float)GRIPPER_PULSO_MIN_MM_US + (mm - GRIPPER_MIN_MM) * US_POR_MM + 0.5f);
}

void Gripper_Init(void)
{
	activo = false;
	moviendo = false;
	fin_pendiente = false;
	pulso_us = pulso_de_apertura(GRIPPER_INICIO_MM);
	PWM_ConfigurarPulsoUs(PWM_SERVO_GRIPPER, pulso_us);
}

/** Habilita el PWM si hacia falta y arranca el perfil hacia un pulso. */
static gripper_res_t iniciar(uint32_t destino_us)
{
	if (moviendo) {
		return GRIPPER_ERR_OCUPADO;
	}
	if (!activo) {
		PWM_Habilitar(PWM_SERVO_GRIPPER, true);
		activo = true;
	}

	float ini = (float)pulso_us;
	float fin = (float)destino_us;

	if (fabsf(fin - ini) < DIST_MIN_US) {
		return GRIPPER_YA_EN_DESTINO;
	}
	Perfil_Iniciar(&perfil, fabsf(fin - ini), GRIPPER_DPS * US_POR_GRADO, GRIPPER_ACCEL_DPS2 * US_POR_GRADO);
	ini_us = ini;
	fin_us = fin;
	moviendo = true;
	return GRIPPER_OK;
}

gripper_res_t Gripper_MoverApertura(float apertura_mm)
{
	/* Comparacion negada a proposito: tambien rechaza NaN. */
	if (!(apertura_mm >= GRIPPER_MIN_MM && apertura_mm <= GRIPPER_MAX_MM)) {
		return GRIPPER_ERR_RANGO;
	}
	return iniciar(pulso_de_apertura(apertura_mm));
}

gripper_res_t Gripper_MoverPulso(uint32_t us)
{
	if (us < PULSE_MIN_US) {
		us = PULSE_MIN_US;
	} else if (us > PULSE_MAX_US) {
		us = PULSE_MAX_US;
	}
	return iniciar(us);
}

void Gripper_Tick(void)
{
	if (!moviendo) {
		return;
	}

	bool llego = Perfil_Paso(&perfil, DT_S);
	float us = (llego && !perfil.detenido)
			 ? fin_us
			 : ini_us + (fin_us - ini_us) * (perfil.s / perfil.d_total);

	pulso_us = (uint32_t)(us + 0.5f);
	PWM_ConfigurarPulsoUs(PWM_SERVO_GRIPPER, pulso_us);
	if (llego) {
		moviendo = false;
		fin_pendiente = true;
	}
}

void Gripper_Detener(void)
{
	if (moviendo) {
		Perfil_Detener(&perfil);
	}
}

bool Gripper_Ocupado(void)
{
	return moviendo;
}

bool Gripper_TomarFin(void)
{
	bool f = fin_pendiente;

	fin_pendiente = false;
	return f;
}

float Gripper_Apertura(void)
{
	return GRIPPER_MIN_MM + ((float)pulso_us - (float)GRIPPER_PULSO_MIN_MM_US) / US_POR_MM;
}

uint32_t Gripper_Pulso(void)
{
	return pulso_us;
}

bool Gripper_Activo(void)
{
	return activo;
}
