/**
 * @file eje_z.c
 * @brief Control del eje Z y de la orientacion (SPEC 9).
 *
 * No toca registros (usa motor.h y encoder.h), asi que tambien se prueba en la PC. Todo se
 * trabaja en un marco logico donde + significa "el contador sube y el eje sube"; el signo de
 * cada motor en config.h se aplica recien al mandar el duty.
 */
#include "eje_z.h"

#include <math.h>
#include <stddef.h>

#include "config.h"
#include "encoder.h"
#include "planificador.h"

_Static_assert((int)ENCODER_A == (int)MOTOR_A && (int)ENCODER_B == (int)MOTOR_B,
			   "encoder_id_t y motor_id_t deben coincidir");

#define DT_S			((float)TICK_MS / 1000.0f)
#define TICKS_PAUSA		(MOTOR_PAUSA_INVERSION_MS / TICK_MS)

typedef enum {
	MODO_REPOSO = 0,
	MODO_TRAYECTORIA,
	MODO_PRUEBA
} modo_t;

/** Estado del lazo de un motor. */
typedef struct {
	float duty;				/**< Duty vigente, con signo; 0 si esta frenado */
	float dir_ref;			/**< Hacia donde va la referencia: +1 o -1 */
	int32_t sentido;		/**< Sentido que tiene cargado el contador */
	int32_t ticks_pausa;	/**< Ticks de frenado que faltan antes de invertir */
	bool en_pausa;
	bool llego;				/**< Entro en tolerancia al final de la trayectoria */
	int32_t ultima_cuenta;
	uint32_t ms_sin_pulsos;
} lazo_t;

static const float signo_motor[MOTOR_CANT] = { MOTOR_A_SIGN, MOTOR_B_SIGN };

static modo_t modo;
static ejez_evento_t evento;
static bool referenciado;
static lazo_t lazo[MOTOR_CANT];

/* Trayectoria: la referencia va de (z_ini, rot_ini) a (z_fin, rot_fin) segun el perfil. */
static perfil_t perfil;
static bool perfil_terminado;
static float z_ini, z_fin, z_cmd;
static float rot_ini, rot_fin, rot_cmd;
static uint32_t ms_asentando;

/* Prueba a lazo abierto. */
static motor_id_t prueba_motor;
static int32_t prueba_cuenta_ini;
static int32_t prueba_ms;

/** Pulsos que le corresponden a un motor para una altura y una orientacion. */
static float referencia_pulsos(int m, float z_mm, float rot_deg)
{
	float parte_rot = ROT_SIGN * rot_deg * PULSOS_POR_GRADO;

	return z_mm * PULSOS_POR_MM + ((m == MOTOR_A) ? parte_rot : -parte_rot);
}

static void frenar(int m)
{
	Motor_Frenar((motor_id_t)m);
	lazo[m].duty = 0.0f;
}

/** Corta los dos motores, y da el aviso. */
static void terminar(ejez_evento_t aviso)
{
	frenar(MOTOR_A);
	frenar(MOTOR_B);
	modo = MODO_REPOSO;
	evento = aviso;
}

void EjeZ_Init(void)
{
	Motor_Init();
	Encoder_Init();

	modo = MODO_REPOSO;
	evento = EJEZ_EVENTO_NINGUNO;
	referenciado = false;
	z_cmd = 0.0f;
	rot_cmd = 0.0f;
	for (int m = 0; m < MOTOR_CANT; m++) {
		lazo[m] = (lazo_t){ .sentido = 1 };
	}
}

ejez_res_t EjeZ_FijarCero(void)
{
	if (modo != MODO_REPOSO) {
		return EJEZ_ERR_OCUPADO;
	}
	Encoder_Reiniciar(ENCODER_A);
	Encoder_Reiniciar(ENCODER_B);
	z_cmd = 0.0f;
	rot_cmd = 0.0f;
	referenciado = true;
	return EJEZ_OK;
}

bool EjeZ_Homing(void)
{
	return false;
}

/**
 * Arranca una trayectoria. El perfil se arma en pulsos de un motor, las unidades que importan
 * para seguir la referencia, sea un movimiento de altura o de orientacion.
 */
static ejez_res_t iniciar(float z_destino, float rot_destino, float dist_pulsos, float v, float a)
{
	if (dist_pulsos <= MOTOR_TOL_PULSOS) {
		return EJEZ_YA_EN_DESTINO;
	}

	Perfil_Iniciar(&perfil, dist_pulsos, v, a);
	perfil_terminado = false;
	ms_asentando = 0U;
	z_ini = z_cmd;
	rot_ini = rot_cmd;
	z_fin = z_destino;
	rot_fin = rot_destino;
	for (int m = 0; m < MOTOR_CANT; m++) {
		float delta = referencia_pulsos(m, z_fin, rot_fin) - referencia_pulsos(m, z_ini, rot_ini);

		lazo[m].dir_ref = (delta >= 0.0f) ? 1.0f : -1.0f;
		lazo[m].llego = false;
		lazo[m].en_pausa = false;
		lazo[m].ms_sin_pulsos = 0U;
		lazo[m].ultima_cuenta = Encoder_Pulsos((encoder_id_t)m);
	}
	modo = MODO_TRAYECTORIA;
	return EJEZ_OK;
}

/** Condiciones comunes a todo movimiento con referencia. */
static ejez_res_t puede_moverse(void)
{
	if (modo != MODO_REPOSO) {
		return EJEZ_ERR_OCUPADO;
	}
	return referenciado ? EJEZ_OK : EJEZ_ERR_SIN_CERO;
}

ejez_res_t EjeZ_MoverAltura(float z_mm)
{
	ejez_res_t r = puede_moverse();

	if (r != EJEZ_OK) {
		return r;
	}
	/* Comparacion negada a proposito: tambien rechaza NaN. */
	if (!(z_mm >= Z_MIN_MM && z_mm <= Z_MAX_MM)) {
		return EJEZ_ERR_RANGO;
	}
	return iniciar(z_mm, rot_cmd, fabsf(z_mm - z_cmd) * PULSOS_POR_MM,
				   Z_VEL_MM_S * PULSOS_POR_MM, Z_ACEL_MM_S2 * PULSOS_POR_MM);
}

ejez_res_t EjeZ_MoverOrientacion(float rot_deg)
{
	ejez_res_t r = puede_moverse();

	if (r != EJEZ_OK) {
		return r;
	}
	if (!(rot_deg >= ROT_MIN_DEG && rot_deg <= ROT_MAX_DEG)) {
		return EJEZ_ERR_RANGO;
	}
	return iniciar(z_cmd, rot_deg, fabsf(rot_deg - rot_cmd) * PULSOS_POR_GRADO,
				   ROT_VEL_DPS * PULSOS_POR_GRADO, ROT_ACEL_DPS2 * PULSOS_POR_GRADO);
}

ejez_res_t EjeZ_Probar(motor_id_t motor, float duty_pct, int32_t ms)
{
	if (modo != MODO_REPOSO) {
		return EJEZ_ERR_OCUPADO;
	}
	if (motor >= MOTOR_CANT || !(fabsf(duty_pct) <= ZTEST_PWM_MAX_PCT) || duty_pct == 0.0f
			|| ms < 1 || ms > ZTEST_MS_MAX) {
		return EJEZ_ERR_RANGO;
	}

	int32_t sentido = (duty_pct > 0.0f) ? 1 : -1;

	/* Los pulsos de la prueba no pasan por el lazo: lo que se creia saber de la posicion ya no vale. */
	referenciado = false;
	prueba_motor = motor;
	prueba_ms = ms;
	lazo[motor].sentido = sentido;
	Encoder_FijarSentido((encoder_id_t)motor, sentido);
	prueba_cuenta_ini = Encoder_Pulsos((encoder_id_t)motor);
	lazo[motor].duty = duty_pct / 100.0f;
	Motor_ConfigurarDuty(motor, signo_motor[motor] * lazo[motor].duty);
	modo = MODO_PRUEBA;
	return EJEZ_OK;
}

void EjeZ_Detener(void)
{
	if (modo == MODO_TRAYECTORIA) {
		Perfil_Detener(&perfil);
	} else if (modo == MODO_PRUEBA) {
		terminar(EJEZ_EVENTO_PRUEBA_FIN);
	}
}

/**
 * Un paso del lazo proporcional de un motor.
 *
 * Mientras la referencia avanza el motor nunca retrocede: el piso MOTOR_PWM_MIN puede andar
 * mas rapido que una rampa lenta, y si retrocediera iria y vendria (con error de conteo en cada
 * vuelta, porque el encoder no da direccion). Si se adelanto, espera frenado.
 */
static void controlar(int m, float err)
{
	lazo_t *l = &lazo[m];
	float pedido = (err >= 0.0f) ? 1.0f : -1.0f;

	if (!perfil_terminado && pedido != l->dir_ref) {
		frenar(m);
		return;
	}

	/* Invertir: primero frenar, porque los pulsos de la inercia siguen contando con el sentido
	 * anterior, y recien despues cambiar el sentido del contador. */
	if ((int32_t)pedido != l->sentido) {
		if (!l->en_pausa) {
			frenar(m);
			l->en_pausa = true;
			l->ticks_pausa = TICKS_PAUSA;
			return;
		}
		if (--l->ticks_pausa > 0) {
			return;
		}
		l->sentido = (int32_t)pedido;
		Encoder_FijarSentido((encoder_id_t)m, l->sentido);
	}
	l->en_pausa = false;

	float modulo = MOTOR_PWM_MIN + MOTOR_KP * fabsf(err);
	float actual = fabsf(l->duty);
	float tope = ((actual < MOTOR_PWM_MIN) ? MOTOR_PWM_MIN : actual) + MOTOR_RAMPA_POR_TICK;

	if (modulo > tope) {
		modulo = tope;
	}
	if (modulo > MOTOR_PWM_MAX) {
		modulo = MOTOR_PWM_MAX;
	}
	l->duty = pedido * modulo;
	Motor_ConfigurarDuty((motor_id_t)m, signo_motor[m] * l->duty);
}

/**
 * Detecta un motor que empuja sin moverse. Cerca del destino no cuenta: ahi el duty minimo puede
 * no alcanzar para los ultimos pulsos, y eso no es un atasco.
 */
static bool atascado(int m, float err)
{
	lazo_t *l = &lazo[m];
	int32_t cuenta = Encoder_Pulsos((encoder_id_t)m);

	if (cuenta != l->ultima_cuenta || l->duty == 0.0f || fabsf(err) <= MOTOR_TOL_ATASCO_PULSOS) {
		l->ultima_cuenta = cuenta;
		l->ms_sin_pulsos = 0U;
		return false;
	}
	l->ms_sin_pulsos += TICK_MS;
	return l->ms_sin_pulsos >= MOTOR_STALL_MS;
}

static void tick_trayectoria(void)
{
	if (!perfil_terminado) {
		perfil_terminado = Perfil_Paso(&perfil, DT_S);

		bool en_destino = perfil_terminado && !perfil.detenido;
		float f = perfil.s / perfil.d_total;

		z_cmd = en_destino ? z_fin : z_ini + (z_fin - z_ini) * f;
		rot_cmd = en_destino ? rot_fin : rot_ini + (rot_fin - rot_ini) * f;
	}

	bool todos_llegaron = true;

	for (int m = 0; m < MOTOR_CANT; m++) {
		float err = referencia_pulsos(m, z_cmd, rot_cmd) - (float)Encoder_Pulsos((encoder_id_t)m);

		if (perfil_terminado && (lazo[m].llego || fabsf(err) <= MOTOR_TOL_PULSOS)) {
			if (!lazo[m].llego) {
				frenar(m);
				lazo[m].llego = true;
			}
			continue;
		}

		todos_llegaron = false;
		controlar(m, err);
		if (atascado(m, err)) {
			/* Sin finales de carrera no hay forma de saber donde quedo: hay que volver a declarar el cero. */
			referenciado = false;
			terminar(EJEZ_EVENTO_ATASCO);
			return;
		}
	}

	if (perfil_terminado) {
		ms_asentando += TICK_MS;
		if (todos_llegaron || ms_asentando >= MOTOR_ASENTAR_MS) {
			terminar(perfil.detenido ? EJEZ_EVENTO_DETENIDO : EJEZ_EVENTO_FIN);
		}
	}
}

static void tick_prueba(void)
{
	prueba_ms -= TICK_MS;
	if (prueba_ms <= 0) {
		terminar(EJEZ_EVENTO_PRUEBA_FIN);
	}
}

void EjeZ_Tick(void)
{
	if (modo == MODO_TRAYECTORIA) {
		tick_trayectoria();
	} else if (modo == MODO_PRUEBA) {
		tick_prueba();
	}
}

bool EjeZ_Ocupado(void)
{
	return modo != MODO_REPOSO;
}

bool EjeZ_Referenciado(void)
{
	return referenciado;
}

ejez_evento_t EjeZ_TomarEvento(void)
{
	ejez_evento_t e = evento;

	evento = EJEZ_EVENTO_NINGUNO;
	return e;
}

float EjeZ_Altura(void)
{
	float a = (float)Encoder_Pulsos(ENCODER_A);
	float b = (float)Encoder_Pulsos(ENCODER_B);

	return 0.5f * (a + b) / PULSOS_POR_MM;
}

float EjeZ_Orientacion(void)
{
	float a = (float)Encoder_Pulsos(ENCODER_A);
	float b = (float)Encoder_Pulsos(ENCODER_B);

	return ROT_SIGN * 0.5f * (a - b) / PULSOS_POR_GRADO;
}

float EjeZ_AlturaComandada(void)
{
	return z_cmd;
}

float EjeZ_OrientacionComandada(void)
{
	return rot_cmd;
}

uint32_t EjeZ_PulsosPrueba(void)
{
	int32_t delta = Encoder_Pulsos((encoder_id_t)prueba_motor) - prueba_cuenta_ini;

	return (uint32_t)((delta < 0) ? -delta : delta);
}

motor_id_t EjeZ_MotorPrueba(void)
{
	return prueba_motor;
}
