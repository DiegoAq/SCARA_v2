/**
 * @file motion.c
 * @brief Estado del movimiento, tick de control y lectura de las juntas con los AS5600.
 */
#include "motion.h"

#include <stddef.h>

#include "as5600.h"
#include "cinematica.h"
#include "config.h"
#include "systick.h"

/** Encoder de cada junta: bus I2C (cableado fijo) y calibracion de config.h. */
typedef struct {
	i2c_bus_t bus;
	float cero_deg;
	float signo;
} encoder_cfg_t;

static const encoder_cfg_t encoders[JUNTA_CANT] = {
	[JUNTA_HOMBRO] = { I2C_BUS_1, ENC1_ZERO_DEG, ENC1_SIGN },
	[JUNTA_CODO]   = { I2C_BUS_2, ENC2_ZERO_DEG, ENC2_SIGN },
};

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

	/* Si un encoder no contesta no se frena el arranque: la consola lo informa
	 * (STATUS, RAW) y cada lectura posterior vuelve a intentar. */
	for (int j = 0; j < JUNTA_CANT; j++) {
		(void)AS5600_Init(encoders[j].bus);
	}
}

bool Motion_LeerJuntaMedida(junta_id_t junta, float *enc_deg, float *theta_deg)
{
	float lectura;

	if (junta >= JUNTA_CANT || !AS5600_LeerAngulo(encoders[junta].bus, &lectura)) {
		return false;
	}
	if (enc_deg != NULL) {
		*enc_deg = lectura;
	}
	if (theta_deg != NULL) {
		*theta_deg = Cinematica_JuntaDesdeEncoder(lectura, encoders[junta].cero_deg,
												  encoders[junta].signo);
	}
	return true;
}

bool Motion_LeerEncoderCrudo(junta_id_t junta, uint16_t *cuentas)
{
	return junta < JUNTA_CANT && AS5600_LeerCuentas(encoders[junta].bus, cuentas);
}

as5600_iman_t Motion_EstadoIman(junta_id_t junta)
{
	return (junta < JUNTA_CANT) ? AS5600_EstadoIman(encoders[junta].bus) : AS5600_SIN_RESPUESTA;
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
