/**
 * @file motion.c
 * @brief Estado del movimiento y tick de control (etapa E2: parametros y medicion del tick).
 */
#include "motion.h"

#include "config.h"
#include "systick.h"

static motion_estado_t estado;
static float velocidad_mm_s;
static float aceleracion_mm_s2;

static uint32_t t_ultimo_tick_ms;
static uint32_t periodo_max_ms;
static bool hubo_tick;

void Motion_Init(void)
{
	estado = MOTION_IDLE;
	velocidad_mm_s = SPEED_DEFAULT_MM_S;
	aceleracion_mm_s2 = ACCEL_DEFAULT_MM_S2;
	periodo_max_ms = 0U;
	hubo_tick = false;
}

void Motion_Tick(void)
{
	uint32_t ahora = Millis();

	if (hubo_tick) {
		uint32_t periodo = ahora - t_ultimo_tick_ms;
		if (periodo > periodo_max_ms) {
			periodo_max_ms = periodo;
		}
	}
	t_ultimo_tick_ms = ahora;
	hubo_tick = true;
}

motion_estado_t Motion_Estado(void)
{
	return estado;
}

const char *Motion_NombreEstado(motion_estado_t e)
{
	switch (e) {
	case MOTION_IDLE:		return "IDLE";
	case MOTION_MOVING:		return "MOVING";
	case MOTION_STOPPING:	return "STOPPING";
	case MOTION_FAULT:		return "FAULT";
	default:				return "?";
	}
}

bool Motion_ConfigurarVelocidad(float v_mm_s)
{
	/* Comparacion escrita para que un NaN tambien quede afuera. */
	if (!(v_mm_s >= SPEED_MIN_MM_S && v_mm_s <= SPEED_MAX_MM_S)) {
		return false;
	}
	velocidad_mm_s = v_mm_s;
	return true;
}

float Motion_Velocidad(void)
{
	return velocidad_mm_s;
}

bool Motion_ConfigurarAceleracion(float a_mm_s2)
{
	if (!(a_mm_s2 >= ACCEL_MIN_MM_S2 && a_mm_s2 <= ACCEL_MAX_MM_S2)) {
		return false;
	}
	aceleracion_mm_s2 = a_mm_s2;
	return true;
}

float Motion_Aceleracion(void)
{
	return aceleracion_mm_s2;
}

uint32_t Motion_PeriodoTickMaxMs(bool reiniciar)
{
	uint32_t max = periodo_max_ms;

	if (reiniciar) {
		periodo_max_ms = 0U;
	}
	return max;
}
