/**
 * @file motion.c
 * @brief Movimiento del brazo en lazo abierto supervisado: HOME, JOINT, PULSE, MOVE y STOP.
 *
 * Seguridad de servos: los tres modos de movimiento generan, tick a tick, pasos que nunca
 * superan JOINT_VMAX_DPS, y aceleran y frenan con rampa (JOINT_ACCEL_DPS2). El unico salto
 * posible es el del primer HOME en una junta sin encoder habilitado, que no sabe donde esta.
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

/** Por debajo de esto un movimiento articular no vale la pena (menos que la resolucion del servo). */
#define DIST_MIN_DEG	0.01f
#define DIST_MIN_US		0.5f

/** Que se esta ejecutando. */
typedef enum {
	MODO_REPOSO = 0,
	MODO_JUNTAS,	/**< JOINT / HOME: las dos juntas sincronizadas, con rampa */
	MODO_PULSO,		/**< PULSE: el pulso de un servo, con rampa */
	MODO_LINEAL,	/**< MOVE: recta cartesiana con el planificador */
	MODO_ASENTANDO	/**< Ya llego: espera SETTLE_MS para que la medicion final sea valida */
} modo_t;

/** Encoder de cada junta: bus I2C (cableado fijo) y calibracion de config.h. */
typedef struct {
	i2c_bus_t bus;
	float cero_deg;
	float signo;
	float centro_deg;	/**< Centro del rango de la junta: fija en que vuelta se expresa lo medido */
	bool habilitado;
} encoder_cfg_t;

static const encoder_cfg_t encoders[JUNTA_CANT] = {
	[JUNTA_HOMBRO] = { I2C_BUS_1, ENC1_ZERO_DEG, ENC1_SIGN,
					   0.5f * (JOINT1_MIN_DEG + JOINT1_MAX_DEG), ENC1_HABILITADO != 0 },
	[JUNTA_CODO]   = { I2C_BUS_2, ENC2_ZERO_DEG, ENC2_SIGN,
					   0.5f * (JOINT2_MIN_DEG + JOINT2_MAX_DEG), ENC2_HABILITADO != 0 },
};

/** Supervision de una junta con encoder habilitado. */
typedef struct {
	bool valida;		/**< El ultimo tick dejo una lectura confiable */
	bool iman_ok;		/**< Resultado de la ultima verificacion del iman */
	float th_deg;		/**< Angulo medido */
	float err_deg;		/**< Objetivo - medido */
	uint32_t ms_sin_lectura;
	uint32_t ms_con_error;
} supervision_t;

static motion_estado_t estado;
static modo_t modo;
static motion_evento_t evento;
static bool servos_activos;
static float velocidad_mm_s;
static float aceleracion_mm_s2;

/** Ultima postura comandada (o la supuesta, antes del primer HOME). */
static float th_cmd[JUNTA_CANT];

/** Postura que se quiso alcanzar. Solo difiere de th_cmd mientras corrige el trim. */
static float th_obj[JUNTA_CANT];

static supervision_t sup[JUNTA_CANT];
static uint32_t ms_verificar_iman;

/* Espera final y correccion (MODO_ASENTANDO). */
static uint32_t ms_asentar;
static int trim_iter;

/* Movimiento articular (MODO_JUNTAS). */
static float jnt_ini[JUNTA_CANT];
static float jnt_fin[JUNTA_CANT];

/* Movimiento de pulso (MODO_PULSO). */
static junta_id_t pul_junta;
static float pul_ini_us;
static float pul_fin_us;

/* Perfil (acelera, crucero, frena) del movimiento articular o de pulso en curso. */
static perfil_t perfil;

static plan_t plan;

static uint32_t t_ultimo_tick_ms;
static uint32_t periodo_max_ms;
static bool hubo_tick;

/**
 * Manda una postura a los dos servos.
 * @return false (y no manda nada) si alguna junta sale de sus limites.
 */
static bool comandar(float th1, float th2)
{
	if (!Cinematica_JuntasEnLimites(th1, th2)
			|| !Servo_ConfigurarJunta(SERVO_HOMBRO, th1) || !Servo_ConfigurarJunta(SERVO_CODO, th2)) {
		return false;
	}
	th_cmd[JUNTA_HOMBRO] = th1;
	th_cmd[JUNTA_CODO] = th2;
	if (trim_iter == 0) {
		th_obj[JUNTA_HOMBRO] = th1;
		th_obj[JUNTA_CODO] = th2;
	}
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
	if (estado == MOTION_FAULT) {
		return MOTION_ERR_FALLA;
	}
	return MOTION_OK;
}

/**
 * Prepara un movimiento articular desde la postura comandada.
 *
 * El perfil se arma sobre la junta que mas recorre; la otra la acompana en proporcion, asi
 * llegan juntas y ninguna supera la velocidad ni la aceleracion del perfil.
 * @return false si la distancia es despreciable y no hay nada que mover.
 */
static bool iniciar_juntas(float th1_deg, float th2_deg)
{
	float d1 = fabsf(th1_deg - th_cmd[JUNTA_HOMBRO]);
	float d2 = fabsf(th2_deg - th_cmd[JUNTA_CODO]);
	float d_max = (d1 > d2) ? d1 : d2;

	if (d_max < DIST_MIN_DEG) {
		return false;
	}

	Perfil_Iniciar(&perfil, d_max, JOINT_MOVE_DPS, JOINT_ACCEL_DPS2);
	jnt_ini[JUNTA_HOMBRO] = th_cmd[JUNTA_HOMBRO];
	jnt_ini[JUNTA_CODO] = th_cmd[JUNTA_CODO];
	jnt_fin[JUNTA_HOMBRO] = th1_deg;
	jnt_fin[JUNTA_CODO] = th2_deg;
	modo = MODO_JUNTAS;
	estado = MOTION_MOVING;
	return true;
}

/**
 * Habilita los servos desde la mejor posicion conocida. Es lo unico que evita el tiron del
 * primer pulso: un servo va a maxima velocidad hacia el primer pulso que recibe.
 *
 * @param primera true la primera vez: una junta sin encoder se supone en HOME. false al salir
 *                de un FAULT: una junta sin encoder sigue en lo ultimo comandado.
 * @return MOTION_OK, o MOTION_ERR_ENCODER (sin tocar los pulsos) si un encoder habilitado no
 *         da una lectura confiable o mide fuera de limites: arrancar ahi seria ir al tope.
 */
static motion_res_t habilitar_servos(bool primera)
{
	float th[JUNTA_CANT] = {
		[JUNTA_HOMBRO] = primera ? HOME_TH1_DEG : th_cmd[JUNTA_HOMBRO],
		[JUNTA_CODO] = primera ? HOME_TH2_DEG : th_cmd[JUNTA_CODO],
	};

	for (int j = 0; j < JUNTA_CANT; j++) {
		if (encoders[j].habilitado
				&& (!AS5600_ImanOK(encoders[j].bus)
					|| !Motion_LeerJuntaMedida((junta_id_t)j, NULL, &th[j]))) {
			return MOTION_ERR_ENCODER;
		}
	}

	trim_iter = 0;
	if (!comandar(th[JUNTA_HOMBRO], th[JUNTA_CODO])) {
		return MOTION_ERR_ENCODER;
	}
	Servo_Habilitar(true);
	servos_activos = true;
	estado = MOTION_IDLE;

	for (int j = 0; j < JUNTA_CANT; j++) {
		sup[j].ms_sin_lectura = 0U;
		sup[j].ms_con_error = 0U;
		sup[j].iman_ok = true;
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
	th_obj[JUNTA_HOMBRO] = HOME_TH1_DEG;
	th_obj[JUNTA_CODO] = HOME_TH2_DEG;
	trim_iter = 0;
	ms_verificar_iman = 0U;
	periodo_max_ms = 0U;
	hubo_tick = false;

	Servo_Init();

	/* Si un encoder no contesta no se frena el arranque: la consola lo informa
	 * (STATUS, RAW) y cada lectura posterior vuelve a intentar. */
	for (int j = 0; j < JUNTA_CANT; j++) {
		sup[j] = (supervision_t){ .iman_ok = true };
		(void)AS5600_Init(encoders[j].bus);
	}
}

motion_res_t Motion_Home(void)
{
	if (modo != MODO_REPOSO) {
		return MOTION_ERR_OCUPADO;
	}
	if (servos_activos && estado != MOTION_FAULT) {
		return Motion_MoverJuntas(HOME_TH1_DEG, HOME_TH2_DEG);
	}

	/* Primer HOME o salida de FAULT: los pulsos se cargan ANTES de habilitar. */
	motion_res_t r = habilitar_servos(!servos_activos);
	if (r != MOTION_OK) {
		return r;
	}
	r = Motion_MoverJuntas(HOME_TH1_DEG, HOME_TH2_DEG);
	if (r == MOTION_OK) {
		return MOTION_SERVOS_SINCRONIZADOS;
	}
	return (r == MOTION_YA_EN_DESTINO) ? MOTION_SERVOS_HABILITADOS : r;
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

	trim_iter = 0;
	return iniciar_juntas(th1_deg, th2_deg) ? MOTION_OK : MOTION_YA_EN_DESTINO;
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
	if (fabsf(fin - ini) < DIST_MIN_US) {
		return MOTION_YA_EN_DESTINO;
	}

	float us_por_grado = Servo_UsPorGrado((servo_id_t)junta);

	Perfil_Iniciar(&perfil, fabsf(fin - ini), JOINT_MOVE_DPS * us_por_grado,
				   JOINT_ACCEL_DPS2 * us_por_grado);
	trim_iter = 0;
	pul_junta = junta;
	pul_ini_us = ini;
	pul_fin_us = fin;
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
		trim_iter = 0;
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
		Perfil_Detener(&perfil);
		estado = MOTION_STOPPING;
		break;
	default:
		break;
	}
}

/**
 * Cierra un movimiento que termino de comandarse. Con encoders espera SETTLE_MS antes de dar
 * el aviso: medir apenas llega veria el retardo del servo, no el error real.
 */
static void llegar(motion_evento_t aviso)
{
	if (aviso != MOTION_EVENTO_FIN || Motion_CantEncoders() == 0) {
		terminar(MOTION_IDLE, aviso);
		return;
	}
	ms_asentar = SETTLE_MS;
	modo = MODO_ASENTANDO;
}

/** Un paso de JOINT/HOME: el perfil da la fraccion recorrida y las dos juntas la siguen. */
static void tick_juntas(void)
{
	bool llego = Perfil_Paso(&perfil, DT_S);
	/* Al llegar sin STOP se comanda el destino exacto, no el redondeo de la interpolacion. */
	bool en_destino = llego && !perfil.detenido;
	float f = perfil.s / perfil.d_total;
	float th1 = en_destino ? jnt_fin[JUNTA_HOMBRO]
						   : jnt_ini[JUNTA_HOMBRO] + (jnt_fin[JUNTA_HOMBRO] - jnt_ini[JUNTA_HOMBRO]) * f;
	float th2 = en_destino ? jnt_fin[JUNTA_CODO]
						   : jnt_ini[JUNTA_CODO] + (jnt_fin[JUNTA_CODO] - jnt_ini[JUNTA_CODO]) * f;

	if (!comandar(th1, th2)) {
		terminar(MOTION_FAULT, MOTION_EVENTO_FALLA);
	} else if (llego) {
		llegar(perfil.detenido ? MOTION_EVENTO_DETENIDO : MOTION_EVENTO_FIN);
	}
}

/** Un paso de PULSE: igual, sobre el pulso; la postura comandada sigue al pulso. */
static void tick_pulso(void)
{
	bool llego = Perfil_Paso(&perfil, DT_S);
	float us = (llego && !perfil.detenido)
			 ? pul_fin_us
			 : pul_ini_us + (pul_fin_us - pul_ini_us) * (perfil.s / perfil.d_total);

	Servo_ConfigurarPulso((servo_id_t)pul_junta, (uint32_t)(us + 0.5f));
	th_cmd[pul_junta] = Servo_JuntaComandada((servo_id_t)pul_junta);
	th_obj[pul_junta] = th_cmd[pul_junta];

	if (llego) {
		llegar(perfil.detenido ? MOTION_EVENTO_DETENIDO : MOTION_EVENTO_FIN);
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
		llegar(plan.detenido ? MOTION_EVENTO_DETENIDO : MOTION_EVENTO_FIN);
	}
}

/**
 * Correccion final (SPEC 8.5): acerca lo comandado para compensar el error medido en reposo.
 *
 * Mueve th_cmd con el mismo perfil de siempre, nunca de golpe. El proximo movimiento sale de
 * ese th_cmd corregido, asi que tampoco hay salto al arrancar.
 * @return true si inicio una correccion; false si no hace falta o no se puede.
 */
static bool corregir_error(void)
{
	float destino[JUNTA_CANT] = { th_cmd[JUNTA_HOMBRO], th_cmd[JUNTA_CODO] };
	bool hace_falta = false;

	for (int j = 0; j < JUNTA_CANT; j++) {
		if (encoders[j].habilitado && sup[j].valida && fabsf(sup[j].err_deg) > ERR_TRIM_DEG) {
			float d = th_cmd[j] + K_TRIM * sup[j].err_deg;

			if (d > th_obj[j] + TRIM_MAX_DEG) {
				d = th_obj[j] + TRIM_MAX_DEG;
			} else if (d < th_obj[j] - TRIM_MAX_DEG) {
				d = th_obj[j] - TRIM_MAX_DEG;
			}
			destino[j] = d;
			hace_falta = true;
		}
	}
	if (!hace_falta || !Cinematica_JuntasEnLimites(destino[JUNTA_HOMBRO], destino[JUNTA_CODO])) {
		return false;
	}
	trim_iter++;
	return iniciar_juntas(destino[JUNTA_HOMBRO], destino[JUNTA_CODO]);
}

/** Espera final: al terminar, corrige si corresponde y da el aviso. */
static void tick_asentando(void)
{
	if (ms_asentar > TICK_MS) {
		ms_asentar -= TICK_MS;
		return;
	}
	if (TRIM_ENABLE && trim_iter < TRIM_MAX_ITER && corregir_error()) {
		return;
	}
	terminar(MOTION_IDLE, MOTION_EVENTO_FIN);
}

/**
 * Lee cada encoder habilitado y actualiza el error. Sin encoders no gasta tiempo en el bus.
 *
 * Un error o una falta de lectura sostenidos por ERR_FAULT_MS pasan a FAULT: se corta el
 * movimiento y los servos sostienen el ultimo pulso. Si el servo no sigue a lo comandado,
 * seguir empujando es lo que lo rompe.
 */
static void supervisar(void)
{
	bool verificar_iman = false;

	if (ms_verificar_iman <= TICK_MS) {
		ms_verificar_iman = ENC_IMAN_PERIODO_MS;
		verificar_iman = true;
	} else {
		ms_verificar_iman -= TICK_MS;
	}

	for (int j = 0; j < JUNTA_CANT; j++) {
		if (!encoders[j].habilitado) {
			continue;
		}

		supervision_t *s = &sup[j];
		float th;

		if (verificar_iman) {
			s->iman_ok = AS5600_ImanOK(encoders[j].bus);
		}
		s->valida = s->iman_ok && Motion_LeerJuntaMedida((junta_id_t)j, NULL, &th);
		if (s->valida) {
			s->th_deg = th;
			s->err_deg = Cinematica_NormalizarDeg(th_obj[j] - th);
			s->ms_sin_lectura = 0U;
			s->ms_con_error = (fabsf(s->err_deg) > ERR_FAULT_DEG) ? s->ms_con_error + TICK_MS : 0U;
		} else {
			s->ms_sin_lectura += TICK_MS;
			s->ms_con_error = 0U;
		}

		if (servos_activos && estado != MOTION_FAULT
				&& (s->ms_sin_lectura >= ERR_FAULT_MS || s->ms_con_error >= ERR_FAULT_MS)) {
			terminar(MOTION_FAULT, MOTION_EVENTO_FALLA_ENCODER);
		}
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
	case MODO_ASENTANDO:
		tick_asentando();
		break;
	default:
		break;
	}

	supervisar();
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

bool Motion_Ocupado(void)
{
	return modo != MODO_REPOSO;
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

void Motion_PosturaObjetivo(float *th1_deg, float *th2_deg)
{
	if (th1_deg != NULL) {
		*th1_deg = th_obj[JUNTA_HOMBRO];
	}
	if (th2_deg != NULL) {
		*th2_deg = th_obj[JUNTA_CODO];
	}
}

void Motion_PulsosComandados(uint32_t *hombro_us, uint32_t *codo_us)
{
	if (hombro_us != NULL) {
		*hombro_us = Servo_PulsoComandado(SERVO_HOMBRO);
	}
	if (codo_us != NULL) {
		*codo_us = Servo_PulsoComandado(SERVO_CODO);
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
		*theta_deg = Cinematica_VueltaCentrada(
			Cinematica_JuntaDesdeEncoder(lectura, encoders[junta].cero_deg, encoders[junta].signo),
			encoders[junta].centro_deg);
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

int Motion_CantEncoders(void)
{
	int n = 0;

	for (int j = 0; j < JUNTA_CANT; j++) {
		if (encoders[j].habilitado) {
			n++;
		}
	}
	return n;
}

bool Motion_Medida(junta_id_t junta, float *th_deg, float *err_deg)
{
	if (junta >= JUNTA_CANT || !encoders[junta].habilitado || !sup[junta].valida) {
		return false;
	}
	if (th_deg != NULL) {
		*th_deg = sup[junta].th_deg;
	}
	if (err_deg != NULL) {
		*err_deg = sup[junta].err_deg;
	}
	return true;
}

int Motion_PosturaReal(float *th1_deg, float *th2_deg)
{
	float th[JUNTA_CANT] = { th_cmd[JUNTA_HOMBRO], th_cmd[JUNTA_CODO] };
	int medidas = 0;

	for (int j = 0; j < JUNTA_CANT; j++) {
		if (Motion_Medida((junta_id_t)j, &th[j], NULL)) {
			medidas++;
		}
	}
	if (th1_deg != NULL) {
		*th1_deg = th[JUNTA_HOMBRO];
	}
	if (th2_deg != NULL) {
		*th2_deg = th[JUNTA_CODO];
	}
	return medidas;
}
