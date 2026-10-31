/**
 * @file test_robot.c
 * @brief Test en PC del eje Z, la garra, el sensor de distancia y las rutinas, sin encoders
 *        habilitados (asi esta config.h hasta calibrar) y con todo el hardware simulado.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "cinematica.h"
#include "config.h"
#include "console.h"
#include "eje_z.h"
#include "gripper.h"
#include "hw_sim.h"
#include "metricas.h"
#include "motion.h"
#include "secuencia.h"

#define CORRER_MUCHO	40000

/** Arranque comun: servos habilitados en HOME y cero del eje Z declarado. */
static void preparar(void)
{
	hw_sim_reiniciar();
	assert(strstr(tipear("home\r"), "OK servos habilitados") != NULL);
	assert(strstr(tipear("setz0\r"), "OK") != NULL);
	borrar_salida();
}

/** true si @p a aparece antes que @p b en lo que respondio la consola. */
static bool antes_que(const char *a, const char *b)
{
	const char *pa = strstr(salida, a);
	const char *pb = strstr(salida, b);

	return pa != NULL && pb != NULL && pa < pb;
}

static void test_metricas(void)
{
	metricas_t m;
	metricas_res_t r;

	Metricas_Iniciar(&m);

	/* Siempre 3 mm corrido en X: error constante, exactitud mala y repetibilidad perfecta. */
	for (int i = 0; i < 10; i++) {
		const float cmd[METRICAS_EJES] = { 100.0f, 200.0f, 0.0f };
		const float med[METRICAS_EJES] = { 103.0f, 200.0f, 0.0f };

		assert(fabsf(Metricas_Agregar(&m, cmd, med) - 3.0f) < 1e-3f);
	}
	Metricas_Resultado(&m, &r);
	assert(r.n == 10U);
	assert(fabsf(r.e_media - 3.0f) < 1e-3f && fabsf(r.e_max - 3.0f) < 1e-3f);
	assert(fabsf(r.exactitud - 3.0f) < 1e-3f);
	assert(r.e_desvio < 1e-3f && r.repetibilidad < 1e-2f);

	/* Dispersion +-1 mm en Y alrededor del punto: exactitud perfecta, sigma = 1,054 con n = 10. */
	Metricas_Iniciar(&m);
	for (int i = 0; i < 10; i++) {
		const float cmd[METRICAS_EJES] = { 0.0f, 0.0f, 0.0f };
		const float med[METRICAS_EJES] = { 0.0f, (i % 2 == 0) ? 1.0f : -1.0f, 0.0f };

		(void)Metricas_Agregar(&m, cmd, med);
	}
	Metricas_Resultado(&m, &r);
	assert(r.exactitud < 1e-3f);
	assert(fabsf(r.repetibilidad - 3.0f * 1.0541f) < 1e-2f);
	assert(fabsf(r.e_media - 1.0f) < 1e-3f);

	recta_t recta;
	float p, o;

	Recta_Iniciar(&recta);
	assert(!Recta_Ajustar(&recta, &p, &o));
	Recta_Agregar(&recta, 1.0f, 5.0f);
	Recta_Agregar(&recta, 1.0f, 7.0f);
	assert(!Recta_Ajustar(&recta, &p, &o));		/* mismo x: la pendiente no esta definida */
	Recta_Iniciar(&recta);
	Recta_Agregar(&recta, 60.0f, -100.0f);
	Recta_Agregar(&recta, 90.0f, -160.0f);
	Recta_Agregar(&recta, 120.0f, -220.0f);
	assert(Recta_Ajustar(&recta, &p, &o));
	assert(fabsf(p + 2.0f) < 1e-3f && fabsf(o - 20.0f) < 1e-2f);
	printf("OK  metricas y ajuste de recta\n");
}

static void test_gripper(void)
{
	hw_sim_reiniciar();
	assert(!pwm_activo[PWM_SERVO_GRIPPER]);

	/* Fuera de rango: ni habilita el PWM. */
	assert(strstr(tipear("gripper 10\r"), "ERR 6") != NULL);
	assert(strstr(tipear("gripper 70\r"), "ERR 6") != NULL);
	assert(strstr(tipear("gripper abc\r"), "ERR 1") != NULL);
	assert(!pwm_activo[PWM_SERVO_GRIPPER]);

	/* El primer comando habilita en la apertura supuesta, que es la abierta. */
	assert(strstr(tipear("gripper open\r"), "OK ya esta en el destino") != NULL);
	assert(pwm_activo[PWM_SERVO_GRIPPER]);
	assert(pwm_us[PWM_SERVO_GRIPPER] == (uint32_t)GRIPPER_PULSO_MAX_MM_US);

	delta_us_visto = 0;
	assert(strstr(tipear("gripper close\r"), "OK") != NULL);
	assert(strstr(tipear("gripper 30\r"), "ERR 4 garra") != NULL);	/* ocupada */
	borrar_salida();
	assert(correr(CORRER_MUCHO) > 10);
	assert(strstr(salida, "DONE gripper=15.0") != NULL);
	assert(pwm_us[PWM_SERVO_GRIPPER] == (uint32_t)GRIPPER_PULSO_MIN_MM_US);
	assert(delta_us_visto <= 8);	/* GRIPPER_DPS = 30 grados/s = 6,7 us por tick */

	/* En la mitad del recorrido de apertura, la mitad del pulso. */
	assert(strstr(tipear("gripper 37.5\r"), "OK") != NULL);
	(void)correr(CORRER_MUCHO);
	assert(pwm_us[PWM_SERVO_GRIPPER] == 1500U);

	/* Pulso crudo para calibrar, y STOP con rampa. */
	assert(strstr(tipear("pulse 3 2300\r"), "OK") != NULL);
	assert(correr(20) == 20);
	assert(strstr(tipear("stop\r"), "OK") != NULL);
	(void)correr(CORRER_MUCHO);
	assert(pwm_us[PWM_SERVO_GRIPPER] > 1500U && pwm_us[PWM_SERVO_GRIPPER] < 2300U);
	printf("OK  gripper: rangos, rampa, pulso crudo y STOP\n");
}

static void test_eje_z(void)
{
	hw_sim_reiniciar();

	/* Sin SETZ0 no se mueve. */
	assert(strstr(tipear("z 10\r"), "ERR 7 eje Z sin referencia") != NULL);
	assert(strstr(tipear("orient 10\r"), "ERR 7") != NULL);
	assert(motor_frenado[MOTOR_A] && motor_frenado[MOTOR_B]);
	assert(strstr(tipear("status\r"), "z0=NO") != NULL);

	assert(strstr(tipear("setz0\r"), "OK") != NULL);
	assert(strstr(tipear("status\r"), "z0=SI z=0.0 rot=0.0") != NULL);
	assert(strstr(tipear("z 500\r"), "ERR 6") != NULL);
	assert(strstr(tipear("z -500\r"), "ERR 6") != NULL);
	assert(strstr(tipear("z 0\r"), "OK ya esta") != NULL);

	/* Sube 20 mm = 200 pulsos en cada motor. */
	assert(strstr(tipear("z 20\r"), "OK\r\n") != NULL);
	assert(strstr(tipear("z 30\r"), "ERR 4 eje Z") != NULL);
	borrar_salida();
	assert(correr(CORRER_MUCHO) > 50);
	assert(strstr(salida, "DONE z=20.") != NULL || strstr(salida, "DONE z=19.") != NULL);
	assert(motor_frenado[MOTOR_A] && motor_frenado[MOTOR_B]);
	assert(fabsf(EjeZ_Altura() - 20.0f) <= 0.3f);
	assert(fabsf(EjeZ_Orientacion()) <= 3.0f);
	assert(fabsf(motor_pos[MOTOR_A] - (float)encoder_pulsos[MOTOR_A]) <= 3.0f);

	/* Baja: invierte el sentido frenando antes, asi el conteo sigue siendo valido. */
	assert(strstr(tipear("z -20\r"), "OK\r\n") != NULL);
	(void)correr(CORRER_MUCHO);
	assert(motor_inversiones_sin_freno == 0);
	assert(fabsf(EjeZ_Altura() + 20.0f) <= 0.3f);
	assert(fabsf(motor_pos[MOTOR_A] - (float)encoder_pulsos[MOTOR_A]) <= 10.0f);
	assert(fabsf(motor_pos[MOTOR_B] - (float)encoder_pulsos[MOTOR_B]) <= 10.0f);

	/* Orientar: los motores van en sentidos opuestos y la altura no cambia. */
	float z_antes = EjeZ_Altura();
	assert(strstr(tipear("orient 45\r"), "OK\r\n") != NULL);
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(fabsf(EjeZ_Orientacion() - 45.0f) <= 1.5f);
	assert(fabsf(EjeZ_Altura() - z_antes) <= 0.3f);
	assert(strstr(tipear("orient 500\r"), "ERR 6") != NULL);

	/* STOP frena con rampa, sin llegar. */
	assert(strstr(tipear("z 80\r"), "OK\r\n") != NULL);
	assert(correr(100) == 100);
	assert(strstr(tipear("stop\r"), "OK") != NULL);
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(strstr(salida, "STOPPED z=") != NULL);
	assert(EjeZ_Altura() > -15.0f && EjeZ_Altura() < 70.0f);
	printf("OK  eje Z: referencia, rango, altura, orientacion y STOP\n");
}

static void test_atasco(void)
{
	preparar();

	/* El motor A no gira: se corta y se pierde la referencia. */
	motor_trabado[MOTOR_A] = true;
	assert(strstr(tipear("z 50\r"), "OK\r\n") != NULL);
	borrar_salida();
	int ticks = correr(CORRER_MUCHO);
	assert(strstr(salida, "FAULT eje Z atascado") != NULL);
	assert(motor_frenado[MOTOR_A] && motor_frenado[MOTOR_B]);
	/* Tarda lo que tarda el error en superar la tolerancia, mas MOTOR_STALL_MS. */
	assert(ticks * TICK_MS < 2000);
	assert(strstr(tipear("z 10\r"), "ERR 7") != NULL);
	assert(strstr(tipear("status\r"), "z0=NO") != NULL);

	/* Se rearma declarando el cero otra vez. */
	motor_trabado[MOTOR_A] = false;
	assert(strstr(tipear("setz0\r"), "OK") != NULL);
	assert(strstr(tipear("z 10\r"), "OK\r\n") != NULL);
	(void)correr(CORRER_MUCHO);
	assert(fabsf(EjeZ_Altura() - 10.0f) <= 0.3f);
	printf("OK  eje Z: deteccion de atasco\n");
}

static void test_ztest(void)
{
	hw_sim_reiniciar();

	assert(strstr(tipear("ztest c 40 200\r"), "ERR 1") != NULL);
	assert(strstr(tipear("ztest a 95 200\r"), "ERR 6") != NULL);
	assert(strstr(tipear("ztest a 0 200\r"), "ERR 6") != NULL);
	assert(strstr(tipear("ztest a 40 99999\r"), "ERR 6") != NULL);

	assert(strstr(tipear("ztest b 50 200\r"), "OK\r\n") != NULL);
	assert(EjeZ_Ocupado());
	borrar_salida();
	assert(correr(CORRER_MUCHO) == 10);		/* 200 ms */
	assert(strstr(salida, "DONE ztest motor=B pulsos=") != NULL);
	assert(encoder_pulsos[MOTOR_B] > 10 && encoder_pulsos[MOTOR_A] == 0);
	assert(motor_frenado[MOTOR_B]);

	/* Hacia atras cuenta para abajo. */
	assert(strstr(tipear("ztest a -50 200\r"), "OK\r\n") != NULL);
	(void)correr(CORRER_MUCHO);
	assert(encoder_pulsos[MOTOR_A] < -10);

	/* Pierde la referencia, aunque hubiera. */
	assert(strstr(tipear("setz0\r"), "OK") != NULL);
	assert(strstr(tipear("ztest a 50 100\r"), "OK\r\n") != NULL);
	assert(strstr(tipear("stop\r"), "OK") != NULL);
	assert(!EjeZ_Ocupado() && motor_frenado[MOTOR_A]);
	assert(strstr(tipear("z 10\r"), "ERR 7") != NULL);
	printf("OK  ZTEST\n");
}

static void test_dist(void)
{
	hw_sim_reiniciar();
	vl53_mm = 123U;
	vl53_z_ref = 0.0f;

	/* Los dos sensores, y la respuesta sale cuando midieron los dos. */
	assert(strstr(tipear("dist\r"), "OK\r\n") != NULL);
	assert(strstr(tipear("dist\r"), "ERR 4 medicion") != NULL);
	borrar_salida();
	for (int i = 0; i < 12; i++) {
		(void)correr(1);
	}
	assert(strstr(salida, "DONE dist=123 dist2=123") != NULL);
	assert(contar(salida, "DONE") == 1);

	vl53_ve_objeto = false;
	assert(strstr(tipear("dist\r"), "OK\r\n") != NULL);
	borrar_salida();
	for (int i = 0; i < 12; i++) {
		(void)correr(1);
	}
	assert(strstr(salida, "DONE dist=SIN_OBJETO dist2=SIN_OBJETO") != NULL);

	/* Sirve con uno solo de los dos. */
	vl53_ve_objeto = true;
	vl53_presente[VL53_A] = false;
	assert(strstr(tipear("status\r"), "sensor=NO sensor2=OK") != NULL);
	assert(strstr(tipear("dist\r"), "OK\r\n") != NULL);
	borrar_salida();
	for (int i = 0; i < 12; i++) {
		(void)correr(1);
	}
	assert(strstr(salida, "DONE dist2=123") != NULL);

	vl53_presente[VL53_A] = true;
	vl53_presente[VL53_B] = false;
	assert(strstr(tipear("status\r"), "sensor=OK sensor2=NO") != NULL);
	assert(strstr(tipear("dist\r"), "OK\r\n") != NULL);
	borrar_salida();
	for (int i = 0; i < 12; i++) {
		(void)correr(1);
	}
	assert(strstr(salida, "DONE dist=123") != NULL && strstr(salida, "dist2") == NULL);

	vl53_presente[VL53_A] = false;
	assert(strstr(tipear("dist\r"), "ERR 8") != NULL);
	assert(strstr(tipear("status\r"), "sensor=NO sensor2=NO") != NULL);
	printf("OK  DIST con los dos sensores y con uno\n");
}

static void test_move_xyz(void)
{
	/* Faltan los servos, o falta el cero: en cualquier caso, nada se mueve. */
	hw_sim_reiniciar();
	assert(strstr(tipear("home\r"), "OK") != NULL);
	assert(strstr(tipear("move 0 300 30\r"), "ERR 7") != NULL);
	hw_sim_reiniciar();
	assert(strstr(tipear("setz0\r"), "OK") != NULL);
	assert(strstr(tipear("move 0 300 30\r"), "ERR 4 servos sin habilitar") != NULL);
	assert(strstr(tipear("home\r"), "OK") != NULL);

	/* Rechazos: nada se mueve. */
	assert(strstr(tipear("move 0 100 30\r"), "ERR 2") != NULL);
	assert(strstr(tipear("move 0 300 500\r"), "ERR 6") != NULL);
	assert(!Secuencia_Activa());

	/* Sube: primero Z, despues el traslado. */
	assert(strstr(tipear("move 0 300 30\r"), "OK\r\n") != NULL);
	assert(Secuencia_Activa());
	assert(strstr(tipear("joint 90 0\r"), "ERR 4 rutina en curso") != NULL);
	assert(strstr(tipear("z 0\r"), "ERR 4 rutina en curso") != NULL);
	assert(strstr(tipear("status\r"), "OK estado=") != NULL);	/* lo que no mueve sigue andando */
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(!Secuencia_Activa());
	assert(antes_que("SEQ paso=0 Z a=30.0", "SEQ paso=1 XY a=0.0 b=300.0"));
	assert(contar(salida, "DONE") == 1);
	assert(strstr(salida, "DONE x=0.0 y=300.0") != NULL);
	assert(fabsf(EjeZ_Altura() - 30.0f) <= 0.3f);

	/* Baja: primero el traslado, despues Z. */
	assert(strstr(tipear("move 0 350 -30\r"), "OK\r\n") != NULL);
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(antes_que("SEQ paso=0 XY a=0.0 b=350.0", "SEQ paso=1 Z a=-30.0"));
	assert(strstr(salida, "DONE x=0.0 y=350.0") != NULL);
	assert(fabsf(EjeZ_Altura() + 30.0f) <= 0.3f);
	printf("OK  MOVE X Y Z: orden segun suba o baje, rechazos antes de mover\n");
}

/** Diferencia entre dos direcciones sin sentido: una caja girada 180 grados es la misma caja. */
static float diferencia_de_eje(float a, float b)
{
	float d = fabsf(Cinematica_NormalizarDeg(a - b));

	return (d > 90.0f) ? 180.0f - d : d;
}

static void test_pick(void)
{
	preparar();
	vl53_mm = 100U;

	/* Con la garra en Z_SEGURA = 50 y el objeto a 100 mm, se acerca de a poco hasta tenerlo a 50,
	 * vuelve a medir y baja hasta dejarlo a PICK_DIST_TOMA = 30. */
	assert(strstr(tipear("pick 0 300 45 25\r"), "OK\r\n") != NULL);
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(!Secuencia_Activa());
	assert(antes_que("SEQ paso=0 Z a=50.0", "SEQ paso=1 XY a=0.0 b=300.0"));
	assert(antes_que("SEQ paso=1 XY", "SEQ paso=2 ORIENT a=45.0"));
	assert(antes_que("SEQ paso=2 ORIENT", "SEQ paso=3 GRIPPER a=60.0"));
	assert(antes_que("SEQ paso=3 GRIPPER", "SEQ paso=4 DIST"));
	assert(antes_que("SEQ paso=4 DIST", "SEQ objeto dist=100.0 z_toma=-20.0"));
	assert(antes_que("SEQ objeto dist=100.0", "SEQ paso=5 Z_APROX"));
	assert(antes_que("SEQ paso=5 Z_APROX", "SEQ paso=6 DIST"));
	assert(antes_que("SEQ paso=6 DIST", "SEQ objeto dist=75.0 z_toma=-20.0"));
	assert(antes_que("SEQ objeto dist=75.0", "SEQ paso=7 Z_APROX"));
	assert(antes_que("SEQ paso=7 Z_APROX", "SEQ paso=8 DIST"));
	assert(antes_que("SEQ paso=8 DIST", "SEQ objeto dist=50.0 z_toma=-20.0"));
	assert(antes_que("SEQ objeto dist=50.0", "SEQ paso=9 Z_TOMA"));
	assert(antes_que("SEQ paso=9 Z_TOMA", "SEQ paso=10 GRIPPER a=25.0"));
	assert(antes_que("SEQ paso=10 GRIPPER", "SEQ paso=12 Z a=50.0"));
	assert(contar(salida, "DONE") == 1);
	assert(strstr(salida, "DONE x=0.0 y=300.0") != NULL);
	assert(vl53_mediciones >= 3 * VL53_CANT * DIST_MUESTRAS);
	assert(fabsf(EjeZ_Altura() - 50.0f) <= 0.3f);
	assert(fabsf(Gripper_Apertura() - 25.0f) <= 0.1f);

	/* R es el angulo en la mesa, no respecto del antebrazo. */
	float q1, q2;

	Motion_PosturaComandada(&q1, &q2);
	assert(diferencia_de_eje(Cinematica_YawGarra(q1, q2, EjeZ_Orientacion()), 45.0f) <= 1.5f);

	/* Sin orientacion ni apertura: no orienta y sujeta con GRIPPER_TOMA_MM. */
	preparar();
	assert(strstr(tipear("pick 0 300\r"), "OK\r\n") != NULL);
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(strstr(salida, "ORIENT") == NULL);
	assert(fabsf(Gripper_Apertura() - GRIPPER_TOMA_MM) <= 0.1f);

	/* Con un solo sensor tambien toma. */
	preparar();
	vl53_presente[VL53_B] = false;
	assert(strstr(tipear("pick 0 300\r"), "OK\r\n") != NULL);
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(contar(salida, "DONE") == 1);
	assert(strstr(salida, "FAULT") == NULL);
	printf("OK  PICK: orden del Anexo B, dos etapas y altura calculada con el sensor\n");
}

static void test_pick_sin_objeto(void)
{
	/* El sensor no ve nada: corta antes de bajar, con la garra arriba. */
	preparar();
	vl53_ve_objeto = false;
	assert(strstr(tipear("pick 0 300\r"), "OK\r\n") != NULL);
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(strstr(salida, "FAULT rutina cortada: sin objeto") != NULL);
	assert(strstr(salida, "Z_APROX") == NULL);
	assert(strstr(salida, "Z_TOMA") == NULL);
	assert(fabsf(EjeZ_Altura() - 50.0f) <= 0.3f);
	assert(!Secuencia_Activa());

	/* Ve algo, pero mas lejos de lo que se considera un objeto. */
	preparar();
	vl53_mm = (uint16_t)(DIST_OBJETO_MAX_MM + 50.0f);
	assert(strstr(tipear("pick 0 300\r"), "OK\r\n") != NULL);
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(strstr(salida, "FAULT rutina cortada: sin objeto") != NULL);

	/* Ve el objeto, pero alcanzarlo sacaria al eje Z de su recorrido. */
	preparar();
	vl53_mm = 250U;
	assert(strstr(tipear("pick 0 300\r"), "OK\r\n") != NULL);
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(strstr(salida, "FAULT rutina cortada: objeto fuera del recorrido") != NULL);

	/* Sin ningun sensor: se rechaza con el brazo quieto. */
	preparar();
	vl53_presente[VL53_A] = false;
	vl53_presente[VL53_B] = false;
	assert(strstr(tipear("pick 0 300\r"), "ERR 8") != NULL);
	assert(!Secuencia_Activa() && !Motion_Ocupado());

	/* Apertura fuera de rango: tambien antes de mover. */
	preparar();
	assert(strstr(tipear("pick 0 300 45 5\r"), "ERR 6 apertura") != NULL);
	assert(!Secuencia_Activa());
	printf("OK  PICK: sin objeto, objeto lejos, sin sensor\n");
}

static void test_pickplace(void)
{
	preparar();
	assert(strstr(tipear("place 100 300\r"), "OK\r\n") != NULL);
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(antes_que("SEQ paso=1 XY a=100.0 b=300.0", "SEQ paso=2 Z a=-50.0"));
	assert(antes_que("SEQ paso=2 Z", "SEQ paso=3 GRIPPER a=60.0"));
	assert(antes_que("SEQ paso=3 GRIPPER", "SEQ paso=5 Z a=50.0"));

	/* El diagrama completo: termina en HOME, con la garra abierta y arriba. */
	preparar();
	assert(strstr(tipear("pickplace 0 300 100 300 45 25\r"), "OK\r\n") != NULL);
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(!Secuencia_Activa());
	assert(antes_que("SEQ objeto", "XY a=100.0 b=300.0"));
	assert(antes_que("XY a=100.0 b=300.0", "HOME"));
	/* Suelta a la altura a la que tomo, un poco mas arriba. */
	assert(strstr(salida, "Z a=-17.0") != NULL);
	assert(contar(salida, "DONE") == 1);
	assert(strstr(salida, "DONE x=0.0 y=400.0 q1=90.0 q2=0.0") != NULL);
	assert(fabsf(Gripper_Apertura() - GRIPPER_MAX_MM) <= 0.1f);
	assert(fabsf(EjeZ_Altura() - Z_SEGURA_MM) <= 0.3f);

	/* El segundo punto invalido se detecta antes de mover hacia el primero. */
	preparar();
	assert(strstr(tipear("pickplace 0 300 0 100\r"), "ERR 2") != NULL);
	assert(!Secuencia_Activa() && !Motion_Ocupado());
	printf("OK  PLACE y PICKPLACE\n");
}

static void test_dibujo(void)
{
	preparar();
	assert(strstr(tipear("pen x\r"), "ERR 1") != NULL);
	assert(strstr(tipear("pen down\r"), "OK\r\n") != NULL);
	(void)correr(CORRER_MUCHO);
	assert(fabsf(EjeZ_Altura() - Z_LAPIZ_ABAJO_MM) <= 0.3f);
	assert(strstr(tipear("pen up\r"), "OK\r\n") != NULL);
	(void)correr(CORRER_MUCHO);
	assert(fabsf(EjeZ_Altura() - Z_LAPIZ_ARRIBA_MM) <= 0.3f);

	/* Rectangulo: va a la esquina con el marcador arriba, lo apoya, da la vuelta y lo levanta. */
	assert(strstr(tipear("draw rect 0 300 40 30\r"), "OK\r\n") != NULL);
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(antes_que("XY a=0.0 b=300.0", "Z a=-50.0"));
	assert(antes_que("Z a=-50.0", "XY a=40.0 b=300.0"));
	assert(antes_que("XY a=40.0 b=300.0", "XY a=40.0 b=330.0"));
	assert(antes_que("XY a=40.0 b=330.0", "XY a=0.0 b=330.0"));
	assert(contar(salida, " XY ") == 5);
	assert(strstr(salida, "DONE x=0.0 y=300.0") != NULL);
	assert(fabsf(EjeZ_Altura() - Z_LAPIZ_ARRIBA_MM) <= 0.3f);

	/* Circulo de 20 mm de radio: 126 mm de perimetro en tramos de 5 mm = 26 tramos + ir al inicio. */
	assert(strstr(tipear("draw circle 0 330 20\r"), "OK\r\n") != NULL);
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(!Secuencia_Activa());
	assert(contar(salida, " XY ") == 27);
	assert(strstr(salida, "DONE x=20.0 y=330.0") != NULL);
	assert(fabsf(EjeZ_Altura() - Z_LAPIZ_ARRIBA_MM) <= 0.3f);

	/* Rechazos antes de mover nada. */
	assert(strstr(tipear("draw circle 0 100 20\r"), "ERR 2") != NULL);
	assert(strstr(tipear("draw circle 0 330 0\r"), "ERR 6") != NULL);
	assert(strstr(tipear("draw circle 0 330 5000\r"), "ERR 6") != NULL);
	assert(strstr(tipear("draw rect 0 300 500 0\r"), "ERR 2") != NULL);
	assert(!Secuencia_Activa());
	printf("OK  PEN y DRAW\n");
}

static void test_stop_rutina(void)
{
	preparar();
	assert(strstr(tipear("pickplace 0 300 100 300\r"), "OK\r\n") != NULL);
	assert(correr(100) == 100);		/* subiendo a Z_SEGURA */
	assert(EjeZ_Ocupado());
	assert(strstr(tipear("stop\r"), "OK\r\n") != NULL);
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(contar(salida, "STOPPED") == 1);
	assert(contar(salida, "DONE") == 0);
	assert(!Secuencia_Activa() && !EjeZ_Ocupado() && !Motion_Ocupado());
	assert(motor_frenado[MOTOR_A] && motor_frenado[MOTOR_B]);

	/* Queda libre para otra cosa. */
	assert(strstr(tipear("z 0\r"), "OK") != NULL);
	(void)correr(CORRER_MUCHO);

	/* Un atasco en mitad de una rutina la corta y frena tambien el brazo. */
	preparar();
	motor_trabado[MOTOR_B] = true;
	assert(strstr(tipear("pick 0 300\r"), "OK\r\n") != NULL);
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(strstr(salida, "FAULT rutina cortada: eje Z (paso 0)") != NULL);
	assert(!Secuencia_Activa());
	printf("OK  STOP y atasco en una rutina\n");
}

/**
 * Con el codo como esta hoy, el anillo de trabajo tiene 95 mm: el barrido entra y encuentra la caja,
 * pero las pasadas lentas sobre ella no. Tiene que decirlo antes de ir hacia la caja.
 */
static void test_scan_anillo_actual(void)
{
	preparar();
	mundo_activo = true;
	mundo_cajas = 1;
	mundo_caja[0] = (mundo_caja_t){ 0.0f, 330.0f, 0.0f, CAJA_LARGO_MM, CAJA_ANCHO_MM, 80.0f };

	/* Fuera del anillo se rechaza con el brazo quieto. */
	assert(strstr(tipear("scan -100 200 200 120\r"), "ERR 2") != NULL);
	assert(!Secuencia_Activa() && !Motion_Ocupado());

	assert(strstr(tipear("scan -60 305 120 45\r"), "OK\r\n") != NULL);
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(!Secuencia_Activa());
	assert(strstr(salida, "SEQ mapa cajas=1") != NULL);
	assert(strstr(salida, "FAULT rutina cortada: las pasadas sobre la caja salen del espacio de trabajo") != NULL);
	printf("OK  SCAN con el anillo actual: avisa que las pasadas no entran\n");
}

static void test_repeat_sin_encoders(void)
{
	preparar();
	assert(strstr(tipear("repeat 0 300 100 300 3\r"), "ERR 5 hace falta") != NULL);
	assert(strstr(tipear("repeat 0 300 100 300 0\r"), "ERR 6") != NULL);
	assert(!Secuencia_Activa());
	printf("OK  REPEAT sin encoders habilitados: rechazado\n");
}

/** Lee un float de lo que respondio la consola, por ejemplo despues de "residuo=". */
static float valor_de(const char *clave)
{
	const char *p = strstr(salida, clave);
	float v;

	assert(p != NULL);
	assert(sscanf(p + strlen(clave), "%f", &v) == 1);
	return v;
}

/** CALIB tiene que encontrar el signo, el cero y la ganancia reales del encoder y del servo. */
static void test_calib(void)
{
	hw_sim_reiniciar();
	assert(strstr(tipear("calib\r"), "ERR 4 servos sin habilitar") != NULL);
	assert(strstr(tipear("calib 3\r"), "ERR 6") != NULL);
	assert(strstr(tipear("home\r"), "OK") != NULL);

	enc_sigue_servo = true;
	enc_signo_real[JUNTA_HOMBRO] = -1.0f;
	enc_cero_real[JUNTA_HOMBRO] = 200.0f;
	enc_ganancia_real[JUNTA_HOMBRO] = 0.95f;
	(void)correr(2);

	assert(strstr(tipear("calib 1\r"), "OK\r\n") != NULL);
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(!Secuencia_Activa());
	assert(contar(salida, "\nC,1,") + contar(salida, "KC,1,") == CALIB_CANT_PASOS);
	assert(strstr(salida, "CALIB junta=1 ENC_SIGN=-1") != NULL);
	assert(fabsf(valor_de("ENC_ZERO_DEG=") - 200.0f) < 0.3f);
	assert(fabsf(valor_de("ganancia=") - 0.95f) < 0.005f);
	assert(fabsf(valor_de("RANGE_DEG=") - 0.95f * SHOULDER_RANGE_DEG) < 1.0f);
	assert(valor_de("residuo=") < 0.3f);
	assert(strstr(salida, "DONE x=0.0 y=400.0") != NULL);	/* vuelve a HOME */

	/* Las dos juntas, y la lectura cruza el 0/360 durante el recorrido del codo. */
	enc_signo_real[JUNTA_HOMBRO] = 1.0f;
	enc_cero_real[JUNTA_HOMBRO] = 0.0f;
	enc_ganancia_real[JUNTA_HOMBRO] = 1.0f;
	enc_cero_real[JUNTA_CODO] = 5.0f;
	(void)correr(2);
	assert(strstr(tipear("calib\r"), "OK\r\n") != NULL);
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(strstr(salida, "CALIB junta=1 ENC_SIGN=1") != NULL);
	assert(strstr(salida, "CALIB junta=2 ENC_SIGN=1 ENC_ZERO_DEG=5.0") != NULL);

	/* Encoder quemado: corta. */
	enc_presente[JUNTA_CODO] = false;
	assert(strstr(tipear("calib 2\r"), "OK\r\n") != NULL);
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(strstr(salida, "FAULT rutina cortada: encoder") != NULL);
	printf("OK  CALIB\n");
}

int main(void)
{
	printf("== test_robot ==\n");
	test_metricas();
	test_gripper();
	test_eje_z();
	test_atasco();
	test_ztest();
	test_dist();
	test_move_xyz();
	test_pick();
	test_pick_sin_objeto();
	test_pickplace();
	test_dibujo();
	test_stop_rutina();
	test_scan_anillo_actual();
	test_repeat_sin_encoders();
	test_calib();
	printf("test_robot: TODO OK\n");
	return 0;
}
