/**
 * @file test_consola.c
 * @brief Test en PC de la consola y del movimiento del brazo, con todo el hardware simulado.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cinematica.h"
#include "config.h"
#include "console.h"
#include "hw_sim.h"
#include "motion.h"

/* ---- Tests --------------------------------------------------------------- */

static void test_arranque(void)
{
	hw_sim_reiniciar();
	assert(strstr(salida, "consola lista") != NULL);
	assert(strstr(salida, "SCARA>") != NULL);
	assert(strstr(salida, "WARN") == NULL);

	salida_len = 0U;
	Consola_Init(false);
	assert(strstr(salida, "WARN reloj") != NULL);
	Consola_Init(true);
	printf("OK  arranque y aviso de reloj\n");
}

static void test_help(void)
{
	const char *r = tipear("help\r");

	assert(strstr(r, "MOVE X Y") != NULL);
	assert(strstr(r, "\r\nOK\r\n") != NULL);
	assert(contar(r, "ERR") == 0);
	printf("OK  HELP\n");
}

static void test_status(void)
{
	Motion_Init();
	const char *r = tipear("STATUS\r");

	assert(strstr(r, "OK estado=IDLE speed=30.0 accel=80.0") != NULL);
	assert(strstr(r, "reloj=HSE") != NULL);
	/* Antes del primer HOME: servos sin pulsos y la postura que se SUPONE (HOME). */
	assert(strstr(r, "servos=OFF q1=90.0 q2=0.0 x=0.0 y=400.0") != NULL);
	assert(strstr(r, "enc1=OK enc2=OK\r\n") != NULL);
	assert(contar(r, "OK estado") == 1);
	const char *linea = strstr(r, "OK estado");
	printf("OK  STATUS: %.*s\n", (int)strcspn(linea, "\r\n"), linea);
}

static void test_speed_accel(void)
{
	Motion_Init();
	assert(strstr(tipear("speed 50\r"), "OK speed=50.0") != NULL);
	assert(strstr(tipear("SPEED 12.5\r"), "OK speed=12.5") != NULL);
	assert(strstr(tipear("speed 9999\r"), "ERR 6 speed fuera de [") != NULL);
	assert(strstr(tipear("speed 0\r"), "ERR 6") != NULL);
	assert(strstr(tipear("speed\r"), "ERR 1") != NULL);
	assert(strstr(tipear("speed rapido\r"), "ERR 1") != NULL);
	assert(Motion_Velocidad() == 12.5f);	/* los rechazos no tocaron el valor */

	assert(strstr(tipear("accel 120\r"), "OK accel=120.0") != NULL);
	assert(strstr(tipear("accel 1\r"), "ERR 6 accel fuera de") != NULL);
	assert(Motion_Aceleracion() == 120.0f);

	assert(strstr(tipear("status\r"), "speed=12.5 accel=120.0") != NULL);
	printf("OK  SPEED y ACCEL con topes\n");
}

static void test_errores(void)
{
	assert(strstr(tipear("hola\r"), "ERR 1 comando desconocido") != NULL);
	assert(strstr(tipear("move 300\r"), "ERR 1 cantidad de argumentos") != NULL);

	/* Linea vacia: solo vuelve el prompt, sin OK ni ERR. */
	const char *r = tipear("\r");
	assert(contar(r, "OK") == 0 && contar(r, "ERR") == 0);
	assert(strstr(r, "SCARA>") != NULL);
	printf("OK  errores\n");
}

/** Enter como CR (PuTTY), CR+LF (Windows) o LF (Linux): siempre un solo comando. */
static void test_fines_de_linea(void)
{
	assert(contar(tipear("status\r"), "OK estado") == 1);
	assert(contar(tipear("status\r\n"), "OK estado") == 1);
	assert(contar(tipear("status\n"), "OK estado") == 1);

	const char *r = tipear("status\r\nstatus\r\n");
	assert(contar(r, "OK estado") == 2);
	assert(contar(r, "SCARA>") == 2);
	printf("OK  fines de linea CR, LF y CR+LF\n");
}

/** Pegar de una vez varias lineas largas: no se pierde ni se mezcla nada. */
static void test_pegar(void)
{
	char texto[600];
	size_t n = 0U;

	/* Cinco lineas de 100 caracteres: "speed 0042.000...0" (100 chars) + CR. */
	for (int k = 0; k < 5; k++) {
		const char *cabeza = "speed 0042.";
		memcpy(texto + n, cabeza, strlen(cabeza));
		n += strlen(cabeza);
		while ((n % 101U) < 100U) {
			texto[n++] = '0';
		}
		texto[n++] = '\r';
	}
	texto[n] = '\0';

	/* Demasiados decimales para el parser: 5 rechazos prolijos, uno por linea. */
	const char *r = tipear(texto);
	assert(contar(r, "ERR 1 argumento invalido") == 5);

	/* Una linea de 100 caracteres valida (con espacios de relleno) se ejecuta entera. */
	char linea[128];
	memset(linea, ' ', sizeof(linea));
	memcpy(linea, "speed 44", 8);
	linea[100] = '\r';
	linea[101] = '\0';
	assert(strstr(tipear(linea), "OK speed=44.0") != NULL);
	printf("OK  pegar 5 lineas de 100 caracteres\n");
}

static void test_log_asincrono(void)
{
	/* El usuario tipeo "spe" (sin Enter) y llega un mensaje asincrono. */
	tipear("spe");
	salida_len = 0U;
	Consola_ImprimirLog("DONE x=300.0 y=0.0");
	assert(strstr(salida, "\r\033[KDONE x=300.0 y=0.0\r\n") != NULL);
	assert(strstr(salida, "SCARA>\033[0m spe") != NULL);

	/* Lo tipeado sigue en la linea: completar el comando funciona. */
	assert(strstr(tipear("ed 20\r"), "OK speed=20.0") != NULL);
	printf("OK  mensaje asincrono sin romper lo tipeado\n");
}

/** ANGLE, RAW e I2CSCAN con los encoders simulados. */
static void test_encoders(void)
{
	char esperado[128];

	/* 1024 cuentas = 90 grados; 3072 = 270. El angulo de junta sale de ENCx_ZERO/SIGN. */
	enc_cuentas[I2C_BUS_1] = 1024U;
	enc_cuentas[I2C_BUS_2] = 3072U;
	float q1, q2;
	assert(Motion_LeerJuntaMedida(JUNTA_HOMBRO, NULL, &q1));
	assert(Motion_LeerJuntaMedida(JUNTA_CODO, NULL, &q2));
	(void)snprintf(esperado, sizeof(esperado), "OK enc1=90.00 q1=%.2f enc2=270.00 q2=%.2f\r\n",
				   (double)q1, (double)q2);
	assert(strstr(tipear("angle\r"), esperado) != NULL);
	assert(strstr(tipear("RAW\r"), "OK raw1=1024 iman1=OK raw2=3072 iman2=OK\r\n") != NULL);
	assert(strstr(tipear("i2cscan\r"), "OK i2c1=0x36 i2c2=0x36\r\n") != NULL);

	/* Iman flojo en el hombro: RAW lo muestra, ANGLE no entrega un angulo dudoso. */
	enc_iman[I2C_BUS_1] = AS5600_IMAN_DEBIL;
	assert(strstr(tipear("angle\r"), "ERR 5 encoder 1 (hombro, I2C1): DEBIL\r\n") != NULL);
	assert(strstr(tipear("raw\r"), "ERR 5 raw1=1024 iman1=DEBIL raw2=3072 iman2=OK\r\n") != NULL);
	assert(strstr(tipear("status\r"), "enc1=DEBIL enc2=OK") != NULL);
	enc_iman[I2C_BUS_1] = AS5600_IMAN_OK;

	/* Encoder del codo desconectado. */
	enc_presente[I2C_BUS_2] = false;
	assert(strstr(tipear("angle\r"), "ERR 5 encoder 2 (codo, I2C2): SIN_RESPUESTA\r\n") != NULL);
	assert(strstr(tipear("raw\r"), "ERR 5 raw1=1024 iman1=OK raw2=? iman2=SIN_RESPUESTA\r\n") != NULL);
	assert(strstr(tipear("i2cscan\r"), "OK i2c1=0x36 i2c2=ninguno\r\n") != NULL);
	assert(strstr(tipear("status\r"), "enc1=OK enc2=SIN_RESPUESTA") != NULL);
	enc_presente[I2C_BUS_2] = true;

	/* Bus trabado: el escaneo corta en la primera direccion en vez de insistir. */
	bus_trabado[I2C_BUS_1] = true;
	assert(strstr(tipear("i2cscan\r"), "OK i2c1=BUS_ERROR i2c2=0x36\r\n") != NULL);
	bus_trabado[I2C_BUS_1] = false;

	printf("OK  ANGLE, RAW e I2CSCAN\n");
}

/** Texto de STATUS desde "servos=" (postura comandada). */
static const char *estado_servos(void)
{
	const char *r = strstr(tipear("status\r"), "servos=");

	assert(r != NULL);
	return r;
}

/** Sin el primer HOME no hay pulsos y no se acepta ningun movimiento. */
static void test_arranque_servos(void)
{
	Motion_Init();
	Consola_Init(true);

	assert(!pwm_activo[PWM_SERVO_HOMBRO] && !pwm_activo[PWM_SERVO_CODO]);
	assert(strstr(tipear("joint 60 30\r"), "ERR 4 servos sin habilitar") != NULL);
	assert(strstr(tipear("move 0 300\r"), "ERR 4 servos sin habilitar") != NULL);
	assert(strstr(tipear("pulse 1 1500\r"), "ERR 4 servos sin habilitar") != NULL);
	assert(!pwm_activo[PWM_SERVO_HOMBRO] && !pwm_activo[PWM_SERVO_CODO]);
	assert(correr(5) == 1);

	/* Primer HOME: habilita con los dos servos en 90 (1500 us) = brazo extendido en X0 Y400. */
	assert(strstr(tipear("home\r"),
				  "OK servos habilitados en HOME x=0.0 y=400.0 q1=90.0 q2=0.0 servo1=90.0(1500us) servo2=90.0(1500us)\r\n") != NULL);
	assert(pwm_activo[PWM_SERVO_HOMBRO] && pwm_activo[PWM_SERVO_CODO]);
	assert(pwm_us[PWM_SERVO_HOMBRO] == 1500U && pwm_us[PWM_SERVO_CODO] == 1500U);
	assert(!pwm_activo[PWM_SERVO_GRIPPER]);
	assert(strncmp(estado_servos(), "servos=ON q1=90.0 q2=0.0 x=0.0 y=400.0", 38) == 0);
	assert(strstr(estado_servos(), "servo1=90.0(1500us) servo2=90.0(1500us)") != NULL);

	/* Un segundo HOME estando en HOME no mueve nada. */
	assert(strstr(tipear("home\r"), "OK ya esta en el destino") != NULL);
	printf("OK  arranque: sin pulsos hasta el primer HOME\n");
}

/** JOINT, HOME y STOP en movimientos articulares. */
static void test_joint(void)
{
	/* 30 grados en cada junta: crucero a JOINT_MOVE_DPS = 30 grados/s con rampas de
	 * JOINT_ACCEL_DPS2 = 90 grados/s2 => d/v + v/a = 1,33 s = 67 ticks. */
	delta_us_visto = 0;
	acel_us_vista = 0;
	assert(strstr(tipear("joint 60 30\r"), "OK\r\n") != NULL);
	assert(Motion_Estado() == MOTION_MOVING);
	salida_len = 0U;
	int ticks = correr(1000);
	assert(ticks >= 66 && ticks <= 68);
	assert(strstr(salida, "DONE ") != NULL && strstr(salida, "q1=60.0 q2=30.0") != NULL);
	assert(Motion_Estado() == MOTION_IDLE);
	/* theta1 = 60 -> servo 60 -> 1167 us; theta2 = 30 -> servo 120 -> 1833 us. */
	assert(pwm_us[PWM_SERVO_HOMBRO] == 1167U && pwm_us[PWM_SERVO_CODO] == 1833U);
	assert(delta_us_visto >= 6 && delta_us_visto <= 8);	/* 30 grados/s = 6,7 us por tick */
	/* Arranca y frena con rampa: la velocidad nunca salta mas que JOINT_ACCEL_DPS2 por tick. */
	assert(acel_us_vista >= 1 && acel_us_vista <= ACEL_US_MAX);

	/* Fuera de limites: no se mueve. */
	assert(strstr(tipear("joint 200 0\r"), "ERR 3 limite articular") != NULL);
	assert(strstr(tipear("joint 90 120\r"), "ERR 3 limite articular") != NULL);
	assert(Motion_Estado() == MOTION_IDLE);

	/* Ocupado, y STOP a mitad de camino. */
	assert(strstr(tipear("joint 120 -30\r"), "OK\r\n") != NULL);
	assert(correr(10) == 10);
	assert(strstr(tipear("joint 90 0\r"), "ERR 4 movimiento en curso") != NULL);
	assert(strstr(tipear("move 0 300\r"), "ERR 4 movimiento en curso") != NULL);
	assert(strstr(tipear("raw live\r"), "ERR 4 movimiento en curso") != NULL);
	assert(correr(20) == 20);						/* ya va a velocidad de crucero */
	uint32_t hombro = pwm_us[PWM_SERVO_HOMBRO];
	acel_us_vista = 0;
	assert(strstr(tipear("stop\r"), "OK\r\n") != NULL);
	assert(Motion_Estado() == MOTION_STOPPING);
	salida_len = 0U;
	int frenado = correr(1000);
	/* Frena con rampa (30 grados/s a 90 grados/s2 = 0,33 s = 17 ticks), no en seco. */
	assert(frenado >= 12 && frenado <= 20);
	assert(strstr(salida, "STOPPED ") != NULL);
	assert(acel_us_vista <= ACEL_US_MAX);
	/* Siguio 5 grados mas (v^2/2a) hacia el destino y quedo lejos de el (theta1 = 120 -> 1833 us). */
	assert(pwm_us[PWM_SERVO_HOMBRO] > hombro + 30U && pwm_us[PWM_SERVO_HOMBRO] < 1700U);

	/* HOME con los servos ya habilitados: vuelve despacio. */
	assert(strstr(tipear("home\r"), "OK\r\n") != NULL);
	salida_len = 0U;
	assert(correr(1000) > 5);
	assert(strstr(salida, "DONE x=0.0 y=400.0 q1=90.0 q2=0.0") != NULL);
	assert(pwm_us[PWM_SERVO_HOMBRO] == 1500U && pwm_us[PWM_SERVO_CODO] == 1500U);
	printf("OK  JOINT, HOME y STOP articular (limite de velocidad verificado tick a tick)\n");
}

/** MOVE: recta, DONE, rechazos, STOP con rampa y TRACE. */
static void test_move(void)
{
	/* Desde HOME (0, 400) hacia adentro por el eje Y, con traza. */
	assert(strstr(tipear("trace on\r"), "OK trace=ON") != NULL);
	delta_us_visto = 0;
	assert(strstr(tipear("move 0 300\r"), "OK\r\n") != NULL);
	salida_len = 0U;
	int ticks = correr(5000);
	assert(strstr(salida, "DONE x=0.0 y=300.0") != NULL);
	assert(contar(salida, "T,") == ticks);		/* una linea de traza por tick */
	assert(delta_us_visto > 0);
	assert(strstr(tipear("trace off\r"), "OK trace=OFF") != NULL);

	/* La postura comandada cae en el destino. */
	float q1, q2, x, y;
	Motion_PosturaComandada(&q1, &q2);
	assert(Cinematica_Directa(q1, q2, &x, &y));
	assert(fabsf(x) < 1e-2f && fabsf(y - 300.0f) < 1e-2f);

	/* Rechazos: nada se mueve y el estado sigue en IDLE. */
	uint32_t hombro = pwm_us[PWM_SERVO_HOMBRO];
	assert(strstr(tipear("move 0 100\r"), "ERR 2 fuera del espacio de trabajo") != NULL);
	assert(strstr(tipear("move 0 500\r"), "ERR 2 fuera del espacio de trabajo") != NULL);
	assert(strstr(tipear("move 0 -300\r"), "ERR 2 fuera del espacio de trabajo") != NULL);
	assert(strstr(tipear("move 0 350 10\r"), "ERR 7 eje Z sin referencia") != NULL);
	assert(strstr(tipear("move 0 300\r"), "OK ya esta en el destino") != NULL);
	assert(Motion_Estado() == MOTION_IDLE && pwm_us[PWM_SERVO_HOMBRO] == hombro);

	/* Sin traza no sale ninguna linea "T,". */
	assert(strstr(tipear("move 100 300\r"), "OK\r\n") != NULL);
	assert(correr(30) == 30);
	assert(strstr(tipear("stop\r"), "OK\r\n") != NULL);
	assert(Motion_Estado() == MOTION_STOPPING);
	salida_len = 0U;
	int frenado = correr(5000);
	assert(frenado > 3 && frenado < 40);		/* freno con rampa, no de golpe */
	assert(strstr(salida, "STOPPED ") != NULL && contar(salida, "T,") == 0);
	Motion_PosturaComandada(&q1, &q2);
	assert(Cinematica_Directa(q1, q2, &x, &y));
	assert(x > 1.0f && x < 99.0f && fabsf(y - 300.0f) < 1e-2f);	/* sobre la recta, sin llegar */

	printf("OK  MOVE: recta, DONE, rechazos, STOP con rampa y TRACE\n");
}

/** PULSE: pulso crudo, saturado y con el mismo limite de velocidad. */
static void test_pulse(void)
{
	assert(strstr(tipear("home\r"), "OK\r\n") != NULL);
	(void)correr(5000);

	delta_us_visto = 0;
	assert(strstr(tipear("pulse 2 1600\r"), "OK\r\n") != NULL);
	salida_len = 0U;
	/* 100 us = 9 grados: no llega a crucero, sube y baja (perfil triangular, ~0,63 s). */
	int ticks = correr(1000);
	assert(ticks >= 28 && ticks <= 36);
	assert(strstr(salida, "DONE ") != NULL);
	assert(pwm_us[PWM_SERVO_CODO] == 1600U && pwm_us[PWM_SERVO_HOMBRO] == 1500U);
	assert(delta_us_visto <= 8);
	/* 100 us = 9 grados de servo: la postura comandada acompana al pulso. */
	assert(strstr(estado_servos(), "q1=90.0 q2=9.0") != NULL);

	/* Un pulso fuera de rango se satura, no se rechaza ni sale tal cual. */
	assert(strstr(tipear("pulse 1 9999\r"), "OK\r\n") != NULL);
	(void)correr(5000);
	assert(pwm_us[PWM_SERVO_HOMBRO] == (uint32_t)PULSE_MAX_US);

	assert(strstr(tipear("pulse 4 1500\r"), "ERR 6") != NULL);
	assert(strstr(tipear("pulse 0 1500\r"), "ERR 6") != NULL);
	assert(strstr(tipear("pulse 1 1500.5\r"), "ERR 1") != NULL);

	/* Despues de un PULSE se puede seguir con JOINT desde donde quedo. */
	assert(strstr(tipear("home\r"), "OK\r\n") != NULL);
	(void)correr(5000);
	assert(pwm_us[PWM_SERVO_HOMBRO] == 1500U && pwm_us[PWM_SERVO_CODO] == 1500U);
	printf("OK  PULSE saturado y con limite de velocidad\n");
}

/** RAW LIVE no bloquea: el tick sigue corriendo y cualquier tecla sale, sin llegar a microrl. */
static void test_raw_en_vivo(void)
{
	assert(strstr(tipear("raw live\r"), "RAW en vivo") != NULL);
	borrar_salida();
	for (int i = 0; i < 16; i++) {
		(void)correr(1);
	}
	assert(contar(salida, "raw1=") >= 2);
	assert(strstr(tipear("x"), "OK fin") != NULL);

	borrar_salida();
	(void)correr(1);
	assert(contar(salida, "raw1=") == 0);
	assert(strstr(tipear("status\r"), "OK estado=") != NULL);
	printf("OK  RAW LIVE sin bloquear el tick\n");
}

static void test_tick(void)
{
	Motion_Init();
	for (int i = 0; i < 10; i++) {
		ms_simulado += TICK_MS;
		Motion_Tick();
	}
	ms_simulado += 35U;		/* un tick atrasado */
	Motion_Tick();
	assert(Motion_PeriodoTickMaxMs(true) == 35U);
	ms_simulado += TICK_MS;
	Motion_Tick();
	assert(Motion_PeriodoTickMaxMs(false) == (uint32_t)TICK_MS);
	printf("OK  medicion del periodo del tick\n");
}

int main(void)
{
	printf("== test_consola ==\n");
	test_arranque();
	test_help();
	test_status();
	test_speed_accel();
	test_errores();
	test_fines_de_linea();
	test_pegar();
	test_log_asincrono();
	test_encoders();
	test_arranque_servos();
	test_joint();
	test_move();
	test_pulse();
	test_raw_en_vivo();
	test_tick();
	printf("test_consola: TODO OK\n");
	return 0;
}
