/**
 * @file test_supervision.c
 * @brief Test en PC de la supervision con encoders: se compila con ENC1_HABILITADO y
 *        ENC2_HABILITADO en 1 (test_supervision) y ademas con TRIM_ENABLE (test_trim).
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "console.h"
#include "gripper.h"
#include "hw_sim.h"
#include "motion.h"
#include "secuencia.h"

_Static_assert(ENC1_HABILITADO == 1 && ENC2_HABILITADO == 1,
			   "este test se compila con los dos encoders habilitados");

#define CORRER_MUCHO	40000

/** Deja los encoders leyendo una postura fija (config.h: cero 0 y signo +1). */
static void encoders_en(float th1, float th2)
{
	float q[JUNTA_CANT] = { th1, th2 };

	for (int j = 0; j < JUNTA_CANT; j++) {
		float enc = (q[j] < 0.0f) ? q[j] + 360.0f : q[j];

		enc_cuentas[j] = (uint16_t)(enc * (float)AS5600_CUENTAS_VUELTA / 360.0f + 0.5f);
	}
}

/** Arranque comun: servos habilitados en HOME, con el encoder siguiendo a cada servo. */
static void preparar(void)
{
	hw_sim_reiniciar();
	encoders_en(HOME_TH1_DEG, HOME_TH2_DEG);
	(void)tipear("home\r");
	enc_sigue_servo = true;
	(void)correr(5);
	borrar_salida();
}

/** El primer pulso sale de lo medido y de ahi va despacio a HOME. Lo pide la regla 7 de seguridad. */
static void test_arranque_desde_lo_medido(void)
{
	hw_sim_reiniciar();
	encoders_en(60.0f, 30.0f);
	assert(!pwm_activo[PWM_SERVO_HOMBRO] && !pwm_activo[PWM_SERVO_CODO]);

	assert(strstr(tipear("home\r"), "OK servos habilitados, yendo a HOME") != NULL);
	assert(pwm_activo[PWM_SERVO_HOMBRO] && pwm_activo[PWM_SERVO_CODO]);
	/* 60 -> servo 60 -> 1167 us; 30 -> servo 120 -> 1833 us (mas la resolucion del encoder). */
	assert(pwm_us[PWM_SERVO_HOMBRO] >= 1166U && pwm_us[PWM_SERVO_HOMBRO] <= 1168U);
	assert(pwm_us[PWM_SERVO_CODO] >= 1832U && pwm_us[PWM_SERVO_CODO] <= 1835U);
	assert(Motion_Estado() == MOTION_MOVING);

	/* correr() verifica el limite de velocidad en cada tick. */
	enc_sigue_servo = true;
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(strstr(salida, "DONE x=0.0 y=400.0 q1=90.0 q2=0.0") != NULL);
	assert(pwm_us[PWM_SERVO_HOMBRO] == 1500U && pwm_us[PWM_SERVO_CODO] == 1500U);
	assert(strstr(salida, "err1=") != NULL && strstr(salida, "err2=") != NULL);

	/* Ya en HOME al encender: habilita sin mover nada. */
	hw_sim_reiniciar();
	encoders_en(HOME_TH1_DEG, HOME_TH2_DEG);
	assert(strstr(tipear("home\r"), "OK servos habilitados en HOME") != NULL);
	assert(Motion_Estado() == MOTION_IDLE);
	printf("OK  el primer pulso sale de lo medido y va a HOME con rampa\n");
}

/** Sin una posicion confiable no se habilita nada: el servo saltaria. */
static void test_no_habilita_sin_medida(void)
{
	hw_sim_reiniciar();
	encoders_en(90.0f, 0.0f);

	enc_iman[I2C_BUS_1] = AS5600_IMAN_AUSENTE;
	assert(strstr(tipear("home\r"), "ERR 5 un encoder habilitado") != NULL);
	assert(!pwm_activo[PWM_SERVO_HOMBRO] && !pwm_activo[PWM_SERVO_CODO]);
	enc_iman[I2C_BUS_1] = AS5600_IMAN_OK;

	enc_presente[I2C_BUS_2] = false;
	assert(strstr(tipear("home\r"), "ERR 5 un encoder habilitado") != NULL);
	assert(!pwm_activo[PWM_SERVO_HOMBRO] && !pwm_activo[PWM_SERVO_CODO]);
	enc_presente[I2C_BUS_2] = true;

	/* Lee un angulo fuera de los limites: arrancar ahi seria ir al tope. */
	encoders_en(90.0f, 120.0f);
	assert(strstr(tipear("home\r"), "ERR 5 un encoder habilitado") != NULL);
	assert(!pwm_activo[PWM_SERVO_HOMBRO] && !pwm_activo[PWM_SERVO_CODO]);
	assert(!Motion_ServosActivos());
	printf("OK  sin posicion confiable no habilita los servos\n");
}

/** Error sostenido: FAULT, comandos rechazados y HOME rearma desde lo medido. */
static void test_fault_por_error(void)
{
	preparar();

	/* Un pico mas corto que ERR_FAULT_MS no hace nada. */
	enc_desvio_deg[JUNTA_HOMBRO] = 15.0f;
	(void)correr(1);
	assert(Motion_Estado() == MOTION_IDLE);
	for (int i = 0; i < 5; i++) {
		(void)correr(1);
	}
	assert(Motion_Estado() == MOTION_IDLE);
	enc_desvio_deg[JUNTA_HOMBRO] = 0.0f;
	for (int i = 0; i < 20; i++) {
		(void)correr(1);
	}
	assert(Motion_Estado() == MOTION_IDLE);

	/* Sostenido, en pleno movimiento: FAULT en ERR_FAULT_MS, y queda donde estaba. */
	assert(strstr(tipear("joint 60 30\r"), "OK\r\n") != NULL);
	assert(correr(40) == 40);
	enc_desvio_deg[JUNTA_CODO] = 15.0f;
	borrar_salida();
	int ticks = correr(CORRER_MUCHO);
	assert(ticks <= (int)(ERR_FAULT_MS / TICK_MS) + 2);
	assert(Motion_Estado() == MOTION_FAULT);
	assert(strstr(salida, "FAULT error o lectura de encoder") != NULL);

	uint32_t hombro = pwm_us[PWM_SERVO_HOMBRO];
	assert(strstr(tipear("joint 90 0\r"), "ERR 5 brazo en falla") != NULL);
	assert(strstr(tipear("move 0 300\r"), "ERR 5 brazo en falla") != NULL);
	assert(strstr(tipear("pick 0 300\r"), "ERR 5 brazo en falla") != NULL);
	(void)correr(20);
	assert(pwm_us[PWM_SERVO_HOMBRO] == hombro);

	/* HOME rearma: parte de lo medido, sin salto. */
	enc_desvio_deg[JUNTA_CODO] = 0.0f;
	(void)correr(1);
	assert(strstr(tipear("home\r"), "OK servos habilitados, yendo a HOME") != NULL);
	assert(Motion_Estado() == MOTION_MOVING);
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(strstr(salida, "DONE x=0.0 y=400.0") != NULL);
	assert(Motion_Estado() == MOTION_IDLE);
	printf("OK  FAULT por error sostenido y rearme con HOME\n");
}

static void test_fault_por_lectura(void)
{
	/* Un corte breve del bus no alcanza. */
	preparar();
	enc_presente[I2C_BUS_1] = false;
	for (int i = 0; i < 4; i++) {
		(void)correr(1);
	}
	assert(Motion_Estado() == MOTION_IDLE);
	enc_presente[I2C_BUS_1] = true;
	for (int i = 0; i < 20; i++) {
		(void)correr(1);
	}
	assert(Motion_Estado() == MOTION_IDLE);

	/* Encoder perdido. */
	enc_presente[I2C_BUS_1] = false;
	for (int i = 0; i < (int)(ERR_FAULT_MS / TICK_MS) + 1; i++) {
		(void)correr(1);
	}
	assert(Motion_Estado() == MOTION_FAULT);
	enc_presente[I2C_BUS_1] = true;

	/* Iman perdido: se detecta en la verificacion periodica. */
	preparar();
	enc_iman[I2C_BUS_2] = AS5600_IMAN_DEBIL;
	for (int i = 0; i < (int)((ERR_FAULT_MS + ENC_IMAN_PERIODO_MS) / TICK_MS) + 1; i++) {
		(void)correr(1);
	}
	assert(Motion_Estado() == MOTION_FAULT);
	printf("OK  FAULT por lectura o iman perdidos\n");
}

/** DONE recien despues de SETTLE_MS, y con el error medido. */
static void test_done_con_error(void)
{
	preparar();
	assert(strstr(tipear("joint 60 30\r"), "OK\r\n") != NULL);
	borrar_salida();
	int ticks = correr(CORRER_MUCHO);
	assert(ticks >= 67 + (int)(SETTLE_MS / TICK_MS) - 1);
	assert(strstr(salida, "DONE x=") != NULL);
	assert(strstr(salida, "q1med=") != NULL && strstr(salida, " e=") != NULL);
	assert(strstr(tipear("status\r"), "q1med=") != NULL);

	/* STOP no espera: no hay un destino sobre el que medir error. Solo el frenado, ~17 ticks. */
	assert(strstr(tipear("joint 90 0\r"), "OK\r\n") != NULL);
	assert(correr(30) == 30);
	assert(strstr(tipear("stop\r"), "OK\r\n") != NULL);
	borrar_salida();
	ticks = correr(CORRER_MUCHO);
	assert(ticks <= 20);
	assert(strstr(salida, "STOPPED") != NULL);

	/* TRACE: lo comandado y lo medido, en CSV. */
	assert(strstr(tipear("trace on\r"), "OK") != NULL);
	assert(strstr(tipear("joint 90 10\r"), "OK\r\n") != NULL);
	borrar_salida();
	(void)correr(5);
	const char *linea = strstr(salida, "T,");
	assert(linea != NULL);
	float t, x, y, q1, q2, q1m, q2m;
	assert(sscanf(linea, "T,%f,%f,%f,%f,%f,%f,%f", &t, &x, &y, &q1, &q2, &q1m, &q2m) == 7);
	assert(fabsf(q1m - q1) < 1.0f && fabsf(q2m - q2) < 1.0f);
	assert(strstr(tipear("trace off\r"), "OK") != NULL);
	printf("OK  DONE con error medido tras SETTLE_MS, y TRACE con lo medido\n");
}

#if !TRIM_ENABLE

/** REPEAT: un error fijo se ve en la exactitud y no en la repetibilidad. */
static void test_repeat(void)
{
	preparar();
	assert(strstr(tipear("repeat 0 300 100 300 500\r"), "ERR 6") != NULL);
	assert(strstr(tipear("repeat 0 100 100 300 3\r"), "ERR 2") != NULL);

	enc_desvio_deg[JUNTA_HOMBRO] = 1.0f;
	assert(strstr(tipear("repeat 0 300 100 300 3\r"), "OK\r\n") != NULL);
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(!Secuencia_Activa());
	assert(contar(salida, "R,") == 6);
	assert(strstr(salida, "REPEAT punto=1 n=3") != NULL);
	assert(strstr(salida, "REPEAT punto=2 n=3") != NULL);
	assert(strstr(salida, "DONE x=100.0 y=300.0") != NULL);

	/* 1 grado de hombro a 300 mm de radio = 5,2 mm, en cada llegada. */
	metricas_res_t r;
	float e_res;
	assert(Secuencia_ResultadoRepetir(0, &r, &e_res));
	assert(r.n == 3U);
	assert(fabsf(r.e_media - 5.24f) < 0.3f);
	assert(fabsf(r.exactitud - 5.24f) < 0.3f);
	assert(r.repetibilidad < 0.5f);
	/* 0,088 grados en las dos juntas, a esa distancia: del orden de medio mm. */
	assert(e_res > 0.2f && e_res < 1.0f);
	assert(!Secuencia_ResultadoRepetir(2, &r, &e_res));

	/* FAULT a mitad de una rutina: la corta. */
	preparar();
	assert(strstr(tipear("repeat 0 300 100 300 5\r"), "OK\r\n") != NULL);
	assert(correr(100) == 100);
	enc_desvio_deg[JUNTA_CODO] = 20.0f;
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(strstr(salida, "FAULT rutina cortada: brazo") != NULL);
	assert(!Secuencia_Activa());
	printf("OK  REPEAT: precision y repetibilidad, y corte por FAULT\n");
}

#else

/** Correccion final: el error baja, sin tirones (correr() lo verifica) y con tope. */
static void test_trim(void)
{
	float err;

	/* 2 grados: se corrige hasta quedar dentro de ERR_TRIM_DEG. */
	preparar();
	enc_desvio_deg[JUNTA_HOMBRO] = 2.0f;
	assert(strstr(tipear("joint 60 30\r"), "OK\r\n") != NULL);
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(strstr(salida, "DONE ") != NULL);
	assert(strstr(salida, "q1=60.0") != NULL);
	assert(Motion_Medida(JUNTA_HOMBRO, NULL, &err));
	assert(fabsf(err) <= ERR_TRIM_DEG + 0.1f);

	/* El siguiente movimiento sale sin salto de lo ya corregido. */
	enc_desvio_deg[JUNTA_HOMBRO] = 0.0f;
	(void)correr(1);
	assert(strstr(tipear("home\r"), "OK\r\n") != NULL);
	(void)correr(CORRER_MUCHO);

	/* 6 grados: no alcanza con TRIM_MAX_DEG; queda el resto, sin pasarse del tope. */
	preparar();
	enc_desvio_deg[JUNTA_CODO] = 6.0f;
	assert(strstr(tipear("joint 60 30\r"), "OK\r\n") != NULL);
	(void)correr(CORRER_MUCHO);
	assert(Motion_Estado() == MOTION_IDLE);
	assert(Motion_Medida(JUNTA_CODO, NULL, &err));
	assert(fabsf(err) >= 6.0f - TRIM_MAX_DEG - 0.1f && fabsf(err) <= 6.0f);
	float q1, q2;
	Motion_PosturaComandada(&q1, &q2);
	assert(fabsf(q2 - 30.0f) <= TRIM_MAX_DEG + 0.01f);
	printf("OK  trim: corrige, con tope y sin salto\n");
}

#endif

int main(void)
{
	printf("== test_supervision (TRIM_ENABLE=%d) ==\n", TRIM_ENABLE);
	test_arranque_desde_lo_medido();
	test_no_habilita_sin_medida();
	test_fault_por_error();
	test_fault_por_lectura();
	test_done_con_error();
#if !TRIM_ENABLE
	test_repeat();
#else
	test_trim();
#endif
	printf("test_supervision: TODO OK\n");
	return 0;
}
