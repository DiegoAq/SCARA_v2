/**
 * @file motion.c
 * @brief Movimiento del brazo en lazo abierto: HOME, JOINT, PULSE, MOVE y STOP.
 *
 * Seguridad de servos: los tres modos de movimiento generan, tick a tick, pasos que nunca
 * superan JOINT_VMAX_DPS. El unico salto posible es el del primer HOME, que habilita el
 * PWM en una postura supuesta (sin encoders no se sabe donde esta el brazo).
 */
#include "motion.h"

#include <math.h>
#include <stddef.h>

#include "cinematica.h"
#include "config.h"
#include "planificador.h"
#include "servo.h"
#include "systick.h"

_Static_assert(JOINT_MOVE_DPS <= JOINT_VMAX_DPS, "JOINT_MOVE_DPS no puede superar JOINT_VMAX_DPS");
_Static_assert((int)JUNTA_HOMBRO == (int)SERVO_HOMBRO && (int)JUNTA_CODO == (int)SERVO_CODO,
			   "junta_id_t y servo_id_t deben coincidir");

#define DT_S	((float)TICK_MS / 1000.0f)

/** Avance maximo por tick de un movimiento articular, en grados y en us de pulso. */
#define PASO_MAX_DEG	(JOINT_MOVE_DPS * DT_S)
#define PASO_MAX_US		(PASO_MAX_DEG * (float)(PULSE_MAX_US - PULSE_MIN_US) / SERVO_RANGE_DEG)

/** Holgura al contar pasos: sin ella, 30 grados / 0,6 da 50,000004 en float y se redondea a
 *  51. A lo sumo deja el paso un 0,1 % por encima de JOINT_MOVE_DPS, lejos de JOINT_VMAX_DPS. */
#define REDONDEO_PASOS	1.0e-3f

/** Que se esta ejecutando. */
typedef enum {
	MODO_REPOSO = 0,
	MODO_JUNTAS,	/**< JOINT / HOME: interpolacion lineal de las dos juntas */
	MODO_PULSO,		/**< PULSE: interpolacion del pulso de un servo */
	MODO_LINEAL		/**< MOVE: recta cartesiana con el planificador */
} modo_t;

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
static modo_t modo;
static motion_evento_t evento;
static bool servos_activos;
static float velocidad_mm_s;
static float aceleracion_mm_s2;

/** Ultima postura comandada (o la supuesta, antes del primer HOME). */
static float th_cmd[JUNTA_CANT];

/* Movimiento articular (MODO_JUNTAS). */
static float jnt_ini[JUNTA_CANT];
static float jnt_fin[JUNTA_CANT];

/* Movimiento de pulso (MODO_PULSO). */
static junta_id_t pul_junta;
static float pul_ini_us;
static float pul_fin_us;

/* Pasos del movimiento articular o de pulso en curso. */
static int32_t paso_actual;
static int32_t pasos_totales;

static plan_t plan;

static uint32_t t_ultimo_tick_ms;
static uint32_t periodo_max_ms;
static bool hubo_tick;

/** Manda una postura a los dos servos. @return false si alguna junta sale de sus limites. */
static bool comandar(float th1, float th2)
{
	if (!Servo_ConfigurarJunta(SERVO_HOMBRO, th1) || !Servo_ConfigurarJunta(SERVO_CODO, th2)) {
		return false;
	}
	th_cmd[JUNTA_HOMBRO] = th1;
	th_cmd[JUNTA_CODO] = th2;
	return true;
}

/** Termina el movimiento en curso y deja el aviso para la consola. */
static void terminar(motion_estado_t nuevo_estado, motion_evento_t aviso)
{
	modo = MODO_REPOSO;
	estado = nuevo_estado;
	evento = aviso;
}

/** Condiciones comunes a todo comando de movimiento. */
static motion_res_t puede_moverse(void)
{
	if (!servos_activos) {
		return MOTION_ERR_APAGADO;
	}
	if (modo != MODO_REPOSO) {
		return MOTION_ERR_OCUPADO;
	}
	return MOTION_OK;
}

void Motion_Init(void)
{
	estado = MOTION_IDLE;
	modo = MODO_REPOSO;
	evento = MOTION_EVENTO_NINGUNO;
	servos_activos = false;
	velocidad_mm_s = SPEED_DEFAULT_MM_S;
	aceleracion_mm_s2 = ACCEL_DEFAULT_MM_S2;
	th_cmd[JUNTA_HOMBRO] = HOME_TH1_DEG;
	th_cmd[JUNTA_CODO] = HOME_TH2_DEG;
	periodo_max_ms = 0U;
	hubo_tick = false;

	Servo_Init();

	/* Si un encoder no contesta no se frena el arranque: la consola lo informa
	 * (STATUS, RAW) y cada lectura posterior vuelve a intentar. */
	for (int j = 0; j < JUNTA_CANT; j++) {
		(void)AS5600_Init(encoders[j].bus);
	}
}

motion_res_t Motion_Home(void)
{
	if (!servos_activos) {
		/* Primer HOME: la postura se carga ANTES de habilitar, para que el primer pulso
		 * que vea cada servo ya sea el de HOME. */
		if (!comandar(HOME_TH1_DEG, HOME_TH2_DEG)) {
			return MOTION_ERR_LIMITE;
		}
		Servo_Habilitar(true);
		servos_activos = true;
		estado = MOTION_IDLE;
		return MOTION_SERVOS_HABILITADOS;
	}
	return Motion_MoverJuntas(HOME_TH1_DEG, HOME_TH2_DEG);
}

motion_res_t Motion_MoverJuntas(float th1_deg, float th2_deg)
{
	motion_res_t r = puede_moverse();
	if (r != MOTION_OK) {
		return r;
	}
	if (!Cinematica_JuntasEnLimites(th1_deg, th2_deg)) {
		return MOTION_ERR_LIMITE;
	}

	float d1 = fabsf(th1_deg - th_cmd[JUNTA_HOMBRO]);
	float d2 = fabsf(th2_deg - th_cmd[JUNTA_CODO]);
	float d_max = (d1 > d2) ? d1 : d2;

	/* Las dos juntas llegan juntas: la que mas recorre va a JOINT_MOVE_DPS y la otra, mas lento. */
	int32_t n = (int32_t)ceilf(d_max / PASO_MAX_DEG - REDONDEO_PASOS);
	if (n <= 0) {
		return MOTION_YA_EN_DESTINO;
	}

	jnt_ini[JUNTA_HOMBRO] = th_cmd[JUNTA_HOMBRO];
	jnt_ini[JUNTA_CODO] = th_cmd[JUNTA_CODO];
	jnt_fin[JUNTA_HOMBRO] = th1_deg;
	jnt_fin[JUNTA_CODO] = th2_deg;
	paso_actual = 0;
	pasos_totales = n;
	modo = MODO_JUNTAS;
	estado = MOTION_MOVING;
	return MOTION_OK;
}

motion_res_t Motion_MoverPulso(junta_id_t junta, uint32_t pulso_us)
{
	motion_res_t r = puede_moverse();
	if (r != MOTION_OK) {
		return r;
	}
	if (junta >= JUNTA_CANT) {
		return MOTION_ERR_PARAM;
	}
	if (pulso_us < PULSE_MIN_US) {
		pulso_us = PULSE_MIN_US;
	} else if (pulso_us > PULSE_MAX_US) {
		pulso_us = PULSE_MAX_US;
	}

	float ini = (float)Servo_PulsoComandado((servo_id_t)junta);
	float fin = (float)pulso_us;
	int32_t n = (int32_t)ceilf(fabsf(fin - ini) / PASO_MAX_US - REDONDEO_PASOS);
	if (n <= 0) {
		return MOTION_YA_EN_DESTINO;
	}

	pul_junta = junta;
	pul_ini_us = ini;
	pul_fin_us = fin;
	paso_actual = 0;
	pasos_totales = n;
	modo = MODO_PULSO;
	estado = MOTION_MOVING;
	return MOTION_OK;
}

motion_res_t Motion_MoverLineal(float x_mm, float y_mm)
{
	motion_res_t r = puede_moverse();
	if (r != MOTION_OK) {
		return r;
	}

	switch (Planificador_Preparar(&plan, th_cmd[JUNTA_HOMBRO], th_cmd[JUNTA_CODO],
								  x_mm, y_mm, velocidad_mm_s, aceleracion_mm_s2)) {
	case PLAN_OK:
		modo = MODO_LINEAL;
		estado = MOTION_MOVING;
		return MOTION_OK;
	case PLAN_YA_EN_DESTINO:
		return MOTION_YA_EN_DESTINO;
	case PLAN_ERR_RADIO:
		return MOTION_ERR_RADIO;
	case PLAN_ERR_LIMITE:
		return MOTION_ERR_LIMITE;
	default:
		return MOTION_ERR_PARAM;
	}
}

void Motion_Detener(void)
{
	switch (modo) {
	case MODO_LINEAL:
		Planificador_Detener(&plan);
		estado = MOTION_STOPPING;
		break;
	case MODO_JUNTAS:
	case MODO_PULSO:
		/* Van a velocidad baja y constante: se quedan donde estan, sin rampa. */
		terminar(MOTION_IDLE, MOTION_EVENTO_DETENIDO);
		break;
	default:
		break;
	}
}

/** Un paso de JOINT/HOME: interpolacion lineal; el ultimo paso cae exacto en el destino. */
static void tick_juntas(void)
{
	paso_actual++;

	bool ultimo = (paso_actual >= pasos_totales);
	float f = ultimo ? 1.0f : (float)paso_actual / (float)pasos_totales;
	float th1 = ultimo ? jnt_fin[JUNTA_HOMBRO]
					   : jnt_ini[JUNTA_HOMBRO] + (jnt_fin[JUNTA_HOMBRO] - jnt_ini[JUNTA_HOMBRO]) * f;
	float th2 = ultimo ? jnt_fin[JUNTA_CODO]
					   : jnt_ini[JUNTA_CODO] + (jnt_fin[JUNTA_CODO] - jnt_ini[JUNTA_CODO]) * f;

	if (!comandar(th1, th2)) {
		terminar(MOTION_FAULT, MOTION_EVENTO_FALLA);
	} else if (ultimo) {
		terminar(MOTION_IDLE, MOTION_EVENTO_FIN);
	}
}

/** Un paso de PULSE: interpolacion del pulso; la postura comandada sigue al pulso. */
static void tick_pulso(void)
{
	paso_actual++;

	bool ultimo = (paso_actual >= pasos_totales);
	float us = ultimo ? pul_fin_us
					  : pul_ini_us + (pul_fin_us - pul_ini_us) * (float)paso_actual / (float)pasos_totales;

	Servo_ConfigurarPulso((servo_id_t)pul_junta, (uint32_t)(us + 0.5f));
	th_cmd[pul_junta] = Servo_JuntaComandada((servo_id_t)pul_junta);

	if (ultimo) {
		terminar(MOTION_IDLE, MOTION_EVENTO_FIN);
	}
}

/** Un paso de MOVE: el planificador entrega la postura del tick. */
static void tick_lineal(void)
{
	float th1 = th_cmd[JUNTA_HOMBRO];
	float th2 = th_cmd[JUNTA_CODO];
	plan_paso_t r = Planificador_Paso(&plan, DT_S, &th1, &th2);

	if (r == PLAN_PASO_ERROR || !comandar(th1, th2)) {
		terminar(MOTION_FAULT, MOTION_EVENTO_FALLA);
	} else if (r == PLAN_PASO_FIN) {
		terminar(MOTION_IDLE, plan.detenido ? MOTION_EVENTO_DETENIDO : MOTION_EVENTO_FIN);
	}
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

	switch (modo) {
	case MODO_JUNTAS:
		tick_juntas();
		break;
	case MODO_PULSO:
		tick_pulso();
		break;
	case MODO_LINEAL:
		tick_lineal();
		break;
	default:
		break;
	}
}

motion_evento_t Motion_TomarEvento(void)
{
	motion_evento_t e = evento;

	evento = MOTION_EVENTO_NINGUNO;
	return e;
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

bool Motion_ServosActivos(void)
{
	return servos_activos;
}

void Motion_PosturaComandada(float *th1_deg, float *th2_deg)
{
	if (th1_deg != NULL) {
		*th1_deg = th_cmd[JUNTA_HOMBRO];
	}
	if (th2_deg != NULL) {
		*th2_deg = th_cmd[JUNTA_CODO];
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
