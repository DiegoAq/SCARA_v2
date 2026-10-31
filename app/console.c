/**
 * @file console.c
 * @brief Consola de comandos: microrl + parser + respuestas OK/ERR.
 */
#include "console.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "busqueda.h"
#include "cinematica.h"
#include "config.h"
#include "eje_z.h"
#include "gripper.h"
#include "i2c.h"
#include "laseres.h"
#include "mapa.h"
#include "microrl.h"
#include "motion.h"
#include "parser.h"
#include "secuencia.h"
#include "servo.h"
#include "systick.h"
#include "uart.h"

#define LARGO_RESPUESTA		192

/** Una fila del mapa: encabezado y hasta MAPA_COLUMNAS_MAX alturas de hasta 4 caracteres y coma. */
#define LARGO_FILA_MAPA		(16 + MAPA_COLUMNAS_MAX * 5)

/** Por debajo de esto una coordenada se muestra como 0 (es menos que lo que se imprime). */
#define CASI_CERO_MM		0.005f

/** Periodo entre lineas del modo en vivo de RAW (diagnostico, no es un parametro fisico). */
#define RAW_LIVE_PERIODO_MS	150U

/* Rango de direcciones I2C de 7 bits validas (las demas estan reservadas por la norma). */
#define I2C_DIR_MIN			0x08U
#define I2C_DIR_MAX			0x77U

/** Numero de servo del comando PULSE para la garra; 1 y 2 son las juntas. */
#define PULSE_SERVO_GRIPPER	3

static microrl_t rl;
static bool reloj_con_hse;
static uint8_t ultimo_rx;
static bool traza_activa;
static bool raw_en_vivo;
static uint32_t raw_ms;
static bool midiendo_distancia;
static bool dist_vista[VL53_CANT];
static laser_lectura_t dist_lectura[VL53_CANT];
static int mapa_fila;			/**< Proxima fila de MAP por imprimir; -1 si no hay ninguna */

static const char texto_ayuda[] =
	"Comandos (no distingue mayusculas; mm y grados):" ENDL
	"  HELP                 esta lista" ENDL
	"  STATUS               estado y parametros activos" ENDL
	"  SPEED v | ACCEL a    velocidad (mm/s) y aceleracion (mm/s2) de MOVE" ENDL
	"  HOME                 1ra vez: HABILITA los servos. Despues: vuelve a HOME. Rearma tras un FAULT" ENDL
	"  MOVE X Y [Z]         recta al punto; con Z, el traslado es a la mayor de las dos alturas" ENDL
	"  JOINT q1 q2          juntas a angulos theta (grados)" ENDL
	"  PULSE n us           pulso crudo: n = 1 hombro, 2 codo, 3 garra; sin limites articulares" ENDL
	"  STOP                 frena todo con rampa y corta la rutina en curso" ENDL
	"  TRACE ON|OFF         linea CSV por tick: T,t_ms,x,y,q1,q2,q1med,q2med,z" ENDL
	"Eje Z y garra:" ENDL
	"  SETZ0                declara la posicion actual como Z = 0 y orientacion = 0" ENDL
	"  Z mm | ORIENT deg    mueve el eje Z o gira la garra; ORIENT es relativo al antebrazo" ENDL
	"  ZTEST A|B duty ms    un motor a lazo abierto (duty en %); informa los pulsos. Pierde el cero" ENDL
	"  GRIPPER OPEN|CLOSE|mm  abre, cierra o lleva a una apertura" ENDL
	"  DIST                 distancia de cada sensor al objeto" ENDL
	"Rutinas:" ENDL
	"  SCAN x y ancho alto  busca la caja en esa area: x y es una esquina respecto de la base" ENDL
	"  MAP                  el mapa del ultimo SCAN: alturas sobre la mesa, por fila" ENDL
	"  PICK                 toma la caja que encontro SCAN" ENDL
	"  PICK X Y [R] [A]     toma un objeto: R = angulo del lado largo en la mesa, A = apertura" ENDL
	"  PLACE X Y            suelta el objeto" ENDL
	"  PICKPLACE x1 y1 x2 y2 [rot] [ancho]  ciclo completo y vuelta a HOME" ENDL
	"  PEN UP|DOWN          levanta o apoya el marcador" ENDL
	"  DRAW RECT x y ancho alto | DRAW CIRCLE x y r  dibuja con el marcador" ENDL
	"  REPEAT x1 y1 x2 y2 n  va y vuelve n veces y mide precision y repetibilidad" ENDL
	"Encoders:" ENDL
	"  ANGLE                angulo de cada encoder y de cada junta (grados)" ENDL
	"  RAW [LIVE]           cuentas y estado del iman; LIVE repite en vivo hasta otra tecla" ENDL
	"  CALIB [1|2]          recorre una junta o las dos y sugiere la calibracion" ENDL
	"  I2CSCAN              direcciones que contestan en cada bus I2C" ENDL;

static void imprimir(const char *texto)
{
	UART_EnviarCadena(texto);
}

/** Responde `OK` o `OK <detalle>`. */
static void responder_ok(const char *detalle)
{
	imprimir("OK");
	if (detalle != NULL && detalle[0] != '\0') {
		imprimir(" ");
		imprimir(detalle);
	}
	imprimir(ENDL);
}

/** Responde `ERR <codigo> <texto>`. */
static void responder_error(consola_err_t codigo, const char *texto)
{
	char prefijo[24];

	/* El texto se manda aparte en vez de copiarlo a otro buffer con "%s": asi no hay
	 * truncamiento posible, sea cual sea su largo. */
	(void)snprintf(prefijo, sizeof(prefijo), "ERR %d ", (int)codigo);
	imprimir(prefijo);
	imprimir(texto);
	imprimir(ENDL);
}

static const char *nombre_iman(as5600_iman_t iman)
{
	switch (iman) {
	case AS5600_IMAN_OK:		return "OK";
	case AS5600_IMAN_AUSENTE:	return "AUSENTE";
	case AS5600_IMAN_DEBIL:		return "DEBIL";
	case AS5600_IMAN_FUERTE:	return "FUERTE";
	default:					return "SIN_RESPUESTA";
	}
}

/**
 * Postura objetivo y su posicion cartesiana. Los valores casi nulos se pasan a 0 exacto:
 * la FK de un brazo sobre un eje da cosas como -0,00002 y se imprimiria "-0.0".
 */
static void postura_actual(float *q1, float *q2, float *x, float *y)
{
	Motion_PosturaObjetivo(q1, q2);
	(void)Cinematica_Directa(*q1, *q2, x, y);
	if (fabsf(*x) < CASI_CERO_MM) {
		*x = 0.0f;
	}
	if (fabsf(*y) < CASI_CERO_MM) {
		*y = 0.0f;
	}
}

/**
 * Arma " servo1=..(..us) servo2=..(..us)": el angulo de SERVO y el pulso de cada uno.
 * q1/q2 son angulos de JUNTA (q2 = 0 es el codo recto, que hoy es el servo en 90): mostrar
 * tambien lo que realmente se le manda al servo evita confundir uno con otro.
 */
static void texto_pulsos(char *linea, size_t largo)
{
	uint32_t us1, us2;

	Motion_PulsosComandados(&us1, &us2);
	(void)snprintf(linea, largo, " servo1=%.1f(%luus) servo2=%.1f(%luus)",
				   (double)Servo_AnguloDePulso(SERVO_HOMBRO, us1), (unsigned long)us1,
				   (double)Servo_AnguloDePulso(SERVO_CODO, us2), (unsigned long)us2);
}

/** Arma " q1med=.. err1=.." con lo que midio un encoder, o nada si no aplica. */
static void texto_medida(char *linea, size_t largo, junta_id_t junta)
{
	float th, err;

	linea[0] = '\0';
	if (Motion_Medida(junta, &th, &err)) {
		int n = (int)junta + 1;

		(void)snprintf(linea, largo, " q%dmed=%.2f err%d=%.2f", n, (double)th, n, (double)err);
	}
}

/** Arma " e=.." con el error de posicion medido, o nada si no hay ningun encoder. */
static void texto_error_posicion(char *linea, size_t largo)
{
	float q1, q2, q1m, q2m, x, y, xm, ym;

	linea[0] = '\0';
	if (Motion_PosturaReal(&q1m, &q2m) > 0) {
		postura_actual(&q1, &q2, &x, &y);
		(void)Cinematica_Directa(q1m, q2m, &xm, &ym);
		(void)snprintf(linea, largo, " e=%.2f", (double)sqrtf((x - xm) * (x - xm) + (y - ym) * (y - ym)));
	}
}

static void comando_status(void)
{
	char linea[LARGO_RESPUESTA];

	/* El maximo del tick se lee antes de consultar los encoders (si uno no contesta, la
	 * consulta tarda) y se reinicia: muestra lo peor desde el STATUS anterior. */
	(void)snprintf(linea, sizeof(linea),
				   "OK estado=%s speed=%.1f accel=%.1f tick_max_ms=%lu rx_perdidos=%lu tx_perdidos=%lu reloj=%s",
				   Motion_NombreEstado(Motion_Estado()),
				   (double)Motion_Velocidad(), (double)Motion_Aceleracion(),
				   (unsigned long)Motion_PeriodoTickMaxMs(true),
				   (unsigned long)UART_BytesPerdidosRx(), (unsigned long)UART_BytesPerdidosTx(),
				   reloj_con_hse ? "HSE" : "HSI");
	imprimir(linea);

	float q1, q2, x, y;
	postura_actual(&q1, &q2, &x, &y);
	(void)snprintf(linea, sizeof(linea), " servos=%s q1=%.1f q2=%.1f x=%.1f y=%.1f",
				   Motion_ServosActivos() ? "ON" : "OFF",
				   (double)q1, (double)q2, (double)x, (double)y);
	imprimir(linea);
	texto_pulsos(linea, sizeof(linea));
	imprimir(linea);

	for (int j = 0; j < JUNTA_CANT; j++) {
		texto_medida(linea, sizeof(linea), (junta_id_t)j);
		imprimir(linea);
	}

	(void)snprintf(linea, sizeof(linea), " z0=%s z=%.1f rot=%.1f grip=%.1f sensor=%s sensor2=%s",
				   EjeZ_Referenciado() ? "SI" : "NO",
				   (double)EjeZ_Altura(), (double)EjeZ_Orientacion(), (double)Gripper_Apertura(),
				   VL53L0X_Listo(VL53_A) ? "OK" : "NO", VL53L0X_Listo(VL53_B) ? "OK" : "NO");
	imprimir(linea);

	imprimir(" enc1=");
	imprimir(nombre_iman(Motion_EstadoIman(JUNTA_HOMBRO)));
	imprimir(" enc2=");
	imprimir(nombre_iman(Motion_EstadoIman(JUNTA_CODO)));
	imprimir(ENDL);
}

/** Arma "x=.. y=.. q1=.. q2=.." con la postura comandada. */
static void texto_postura(char *linea, size_t largo)
{
	float q1, q2, x, y;

	postura_actual(&q1, &q2, &x, &y);
	(void)snprintf(linea, largo, "x=%.1f y=%.1f q1=%.1f q2=%.1f",
				   (double)x, (double)y, (double)q1, (double)q2);
}

/** Traduce el resultado de un comando de movimiento a la respuesta de la consola. */
static void responder_movimiento(motion_res_t r)
{
	char linea[LARGO_RESPUESTA];

	switch (r) {
	case MOTION_OK:
		responder_ok(NULL);
		break;
	case MOTION_YA_EN_DESTINO:
		responder_ok("ya esta en el destino");
		break;
	case MOTION_SERVOS_HABILITADOS:
		texto_postura(linea, sizeof(linea));
		imprimir("OK servos habilitados en HOME ");
		imprimir(linea);
		texto_pulsos(linea, sizeof(linea));
		imprimir(linea);
		imprimir(ENDL);
		break;
	case MOTION_SERVOS_SINCRONIZADOS:
		responder_ok("servos habilitados, yendo a HOME");
		break;
	case MOTION_ERR_RADIO:
		responder_error(ERR_RADIO, "fuera del espacio de trabajo");
		break;
	case MOTION_ERR_LIMITE:
		responder_error(ERR_LIMITE, "limite articular");
		break;
	case MOTION_ERR_OCUPADO:
		responder_error(ERR_OCUPADO, "movimiento en curso (STOP para detenerlo)");
		break;
	case MOTION_ERR_APAGADO:
		responder_error(ERR_OCUPADO, "servos sin habilitar: ejecutar HOME primero");
		break;
	case MOTION_ERR_FALLA:
		responder_error(ERR_ENCODER, "brazo en falla: ejecutar HOME para rearmar");
		break;
	case MOTION_ERR_ENCODER:
		responder_error(ERR_ENCODER, "un encoder habilitado no da una posicion valida: no se habilitan los servos");
		break;
	default:
		responder_error(ERR_RANGO, "parametro fuera de rango");
		break;
	}
}

static void responder_eje_z(ejez_res_t r)
{
	switch (r) {
	case EJEZ_OK:
		responder_ok(NULL);
		break;
	case EJEZ_YA_EN_DESTINO:
		responder_ok("ya esta en el destino");
		break;
	case EJEZ_ERR_OCUPADO:
		responder_error(ERR_OCUPADO, "eje Z en movimiento (STOP para detenerlo)");
		break;
	case EJEZ_ERR_SIN_CERO:
		responder_error(ERR_EJE_Z, "eje Z sin referencia: ejecutar SETZ0");
		break;
	default:
		responder_error(ERR_RANGO, "parametro fuera de rango");
		break;
	}
}

static void responder_gripper(gripper_res_t r)
{
	switch (r) {
	case GRIPPER_OK:
		responder_ok(NULL);
		break;
	case GRIPPER_YA_EN_DESTINO:
		responder_ok("ya esta en el destino");
		break;
	case GRIPPER_ERR_OCUPADO:
		responder_error(ERR_OCUPADO, "garra en movimiento");
		break;
	default:
		responder_error(ERR_RANGO, "apertura fuera de rango");
		break;
	}
}

static void responder_secuencia(seq_res_t r)
{
	switch (r) {
	case SEQ_OK:
		/* PICK usa el sensor por su cuenta: un DIST recien pedido ya no tiene quien lo lea. */
		midiendo_distancia = false;
		responder_ok(NULL);
		break;
	case SEQ_ERR_OCUPADO:
		responder_error(ERR_OCUPADO, "movimiento en curso (STOP para detenerlo)");
		break;
	case SEQ_ERR_APAGADO:
		responder_error(ERR_OCUPADO, "servos sin habilitar: ejecutar HOME primero");
		break;
	case SEQ_ERR_FALLA:
		responder_error(ERR_ENCODER, "brazo en falla: ejecutar HOME para rearmar");
		break;
	case SEQ_ERR_RADIO:
		responder_error(ERR_RADIO, "fuera del espacio de trabajo");
		break;
	case SEQ_ERR_LIMITE:
		responder_error(ERR_LIMITE, "limite articular");
		break;
	case SEQ_ERR_Z_RANGO:
		responder_error(ERR_RANGO, "altura u orientacion fuera de rango");
		break;
	case SEQ_ERR_Z_SIN_CERO:
		responder_error(ERR_EJE_Z, "eje Z sin referencia: ejecutar SETZ0");
		break;
	case SEQ_ERR_GRIPPER_RANGO:
		responder_error(ERR_RANGO, "apertura fuera de rango");
		break;
	case SEQ_ERR_ENCODER:
		responder_error(ERR_ENCODER, "hace falta al menos un encoder habilitado en config.h");
		break;
	case SEQ_ERR_SENSOR:
		responder_error(ERR_SENSOR, "sensor de distancia sin respuesta (STATUS muestra cual)");
		break;
	case SEQ_ERR_SIN_CAJA:
		responder_error(ERR_SIN_CAJA, "no hay una caja encontrada: ejecutar SCAN");
		break;
	default:
		responder_error(ERR_RANGO, "parametro fuera de rango");
		break;
	}
}

static void comando_move(const comando_t *cmd)
{
	if (cmd->tiene_z) {
		responder_secuencia(Secuencia_MoverXYZ(cmd->x_mm, cmd->y_mm, cmd->z_mm));
		return;
	}
	responder_movimiento(Motion_MoverLineal(cmd->x_mm, cmd->y_mm));
}

static void comando_pulse(const comando_t *cmd)
{
	if (cmd->servo < 1 || cmd->servo > PULSE_SERVO_GRIPPER || cmd->pulso_us < 0) {
		responder_error(ERR_RANGO, "uso: PULSE n us, con n = 1 (hombro), 2 (codo) o 3 (garra)");
		return;
	}
	if (cmd->servo == PULSE_SERVO_GRIPPER) {
		responder_gripper(Gripper_MoverPulso((uint32_t)cmd->pulso_us));
		return;
	}
	responder_movimiento(Motion_MoverPulso((junta_id_t)(cmd->servo - 1), (uint32_t)cmd->pulso_us));
}

static void comando_gripper(const comando_t *cmd)
{
	float mm = cmd->valor;

	if (cmd->gripper == GRIPPER_ABRIR) {
		mm = GRIPPER_MAX_MM;
	} else if (cmd->gripper == GRIPPER_CERRAR) {
		mm = GRIPPER_MIN_MM;
	}
	responder_gripper(Gripper_MoverApertura(mm));
}

static void comando_dist(void)
{
	if (midiendo_distancia) {
		responder_error(ERR_OCUPADO, "medicion en curso");
	} else if (!Laseres_Arrancar()) {
		responder_error(ERR_SENSOR, "sensor de distancia sin respuesta");
	} else {
		midiendo_distancia = true;
		for (int i = 0; i < VL53_CANT; i++) {
			dist_vista[i] = false;
		}
		responder_ok(NULL);
	}
}

/** MAP: encabezado ahora; las filas salen de a una por tick, para no llenar el buffer de la UART. */
static void comando_map(void)
{
	char linea[LARGO_RESPUESTA];
	float mesa;

	if (Mapa_Filas() == 0 || !Busqueda_MesaDelMapa(&mesa)) {
		responder_error(ERR_SIN_CAJA, "no hay un mapa: ejecutar SCAN");
		return;
	}
	(void)snprintf(linea, sizeof(linea), "celda=%.1f filas=%d columnas=%d mesa=%.0f",
				   (double)MAPA_CELDA_MM, Mapa_Filas(), Mapa_Columnas(), (double)mesa);
	responder_ok(linea);
	mapa_fila = 0;
}

static void comando_draw(const comando_t *cmd)
{
	if (cmd->figura == FIGURA_RECT) {
		responder_secuencia(Secuencia_DibujarRect(cmd->x_mm, cmd->y_mm, cmd->ancho_mm, cmd->alto_mm));
	} else {
		responder_secuencia(Secuencia_DibujarCirculo(cmd->x_mm, cmd->y_mm, cmd->radio_mm));
	}
}

/** ANGLE: angulo de cada encoder y de cada junta. Exige iman OK en los dos. */
static void comando_angle(void)
{
	char linea[LARGO_RESPUESTA];
	float enc[JUNTA_CANT];
	float theta[JUNTA_CANT];

	for (int j = 0; j < JUNTA_CANT; j++) {
		as5600_iman_t iman = Motion_EstadoIman((junta_id_t)j);

		if (iman != AS5600_IMAN_OK
				|| !Motion_LeerJuntaMedida((junta_id_t)j, &enc[j], &theta[j])) {
			(void)snprintf(linea, sizeof(linea), "encoder %d (%s): %s", j + 1,
						   (j == JUNTA_HOMBRO) ? "hombro, I2C1" : "codo, I2C2",
						   (iman == AS5600_IMAN_OK) ? "SIN_RESPUESTA" : nombre_iman(iman));
			responder_error(ERR_ENCODER, linea);
			return;
		}
	}

	(void)snprintf(linea, sizeof(linea), "enc1=%.2f q1=%.2f enc2=%.2f q2=%.2f",
				   (double)enc[JUNTA_HOMBRO], (double)theta[JUNTA_HOMBRO],
				   (double)enc[JUNTA_CODO], (double)theta[JUNTA_CODO]);
	responder_ok(linea);
}

/** Una lectura de los dos canales; comun a RAW y a RAW LIVE.
 *  @return true si los dos imanes estan OK (linea para responder_ok); false si alguno
 *  no esta OK (linea para responder_error). */
static bool leer_raw(char *linea, size_t largo)
{
	char parte[JUNTA_CANT][48];
	bool todo_ok = true;

	for (int j = 0; j < JUNTA_CANT; j++) {
		uint16_t cuentas = 0U;
		as5600_iman_t iman = Motion_EstadoIman((junta_id_t)j);

		if (iman == AS5600_SIN_RESPUESTA || !Motion_LeerEncoderCrudo((junta_id_t)j, &cuentas)) {
			(void)snprintf(parte[j], sizeof(parte[j]), "raw%d=? iman%d=SIN_RESPUESTA", j + 1, j + 1);
			todo_ok = false;
		} else {
			(void)snprintf(parte[j], sizeof(parte[j]), "raw%d=%u iman%d=%s", j + 1,
						   (unsigned)cuentas, j + 1, nombre_iman(iman));
			todo_ok = todo_ok && (iman == AS5600_IMAN_OK);
		}
	}

	(void)snprintf(linea, largo, "%s %s", parte[JUNTA_HOMBRO], parte[JUNTA_CODO]);
	return todo_ok;
}

/**
 * RAW: cuentas de 12 bits y estado del iman de cada encoder, para diagnostico.
 *
 * RAW LIVE: igual, pero repite una linea cada RAW_LIVE_PERIODO_MS hasta que llega cualquier
 * byte. No bloquea el loop. Se rechaza en movimiento porque la primera tecla solo sale del
 * modo en vivo: un STOP tipeado ahi no frenaria nada.
 */
static void comando_raw(bool en_vivo)
{
	char linea[LARGO_RESPUESTA];

	if (!en_vivo) {
		if (leer_raw(linea, sizeof(linea))) {
			responder_ok(linea);
		} else {
			responder_error(ERR_ENCODER, linea);
		}
		return;
	}
	if (Motion_Ocupado() || EjeZ_Ocupado() || Secuencia_Activa()) {
		responder_error(ERR_OCUPADO, "movimiento en curso: RAW LIVE solo con el brazo quieto");
		return;
	}

	imprimir("RAW en vivo, cualquier tecla para salir" ENDL);
	raw_en_vivo = true;
	raw_ms = RAW_LIVE_PERIODO_MS;	/* imprime la primera en el proximo tick */
}

/**
 * I2CSCAN: direcciones que contestan en cada bus. Es un comando de diagnostico: recorre
 * 112 direcciones por bus y puede atrasar algun tick; no usarlo con el brazo en movimiento.
 */
static void comando_i2cscan(void)
{
	char texto[16];

	imprimir("OK");
	for (int bus = 0; bus < I2C_CANT_BUSES; bus++) {
		int encontrados = 0;

		(void)snprintf(texto, sizeof(texto), " i2c%d=", bus + 1);
		imprimir(texto);

		for (unsigned dir = I2C_DIR_MIN; dir <= I2C_DIR_MAX; dir++) {
			i2c_res_t r = I2C_Sondear((i2c_bus_t)bus, (uint8_t)dir);

			if (r == I2C_OK) {
				(void)snprintf(texto, sizeof(texto), "%s0x%02X", (encontrados > 0) ? "," : "", dir);
				imprimir(texto);
				encontrados++;
			} else if (r != I2C_ERR_NACK) {
				/* Timeout o error de bus: seguir probando serian 2 ms por direccion. */
				imprimir((encontrados > 0) ? ",BUS_ERROR" : "BUS_ERROR");
				encontrados = -1;
				break;
			}
		}
		if (encontrados == 0) {
			imprimir("ninguno");
		}
	}
	imprimir(ENDL);
}

static void comando_speed(float valor)
{
	char linea[LARGO_RESPUESTA];

	if (!Motion_ConfigurarVelocidad(valor)) {
		(void)snprintf(linea, sizeof(linea), "speed fuera de [%.1f, %.1f] mm/s",
					   (double)SPEED_MIN_MM_S, (double)SPEED_MAX_MM_S);
		responder_error(ERR_RANGO, linea);
		return;
	}
	(void)snprintf(linea, sizeof(linea), "speed=%.1f", (double)Motion_Velocidad());
	responder_ok(linea);
}

static void comando_accel(float valor)
{
	char linea[LARGO_RESPUESTA];

	if (!Motion_ConfigurarAceleracion(valor)) {
		(void)snprintf(linea, sizeof(linea), "accel fuera de [%.1f, %.1f] mm/s2",
					   (double)ACCEL_MIN_MM_S2, (double)ACCEL_MAX_MM_S2);
		responder_error(ERR_RANGO, linea);
		return;
	}
	(void)snprintf(linea, sizeof(linea), "accel=%.1f", (double)Motion_Aceleracion());
	responder_ok(linea);
}

/** STOP frena todo lo que este en movimiento. */
static void comando_stop(void)
{
	if (!Secuencia_Abortar()) {
		Motion_Detener();
		EjeZ_Detener();
		Gripper_Detener();
	}
	responder_ok(NULL);
}

/** true para los comandos que no se pueden aceptar con una rutina en curso. */
static bool necesita_estar_libre(cmd_id_t id)
{
	switch (id) {
	case CMD_MOVE:
	case CMD_JOINT:
	case CMD_HOME:
	case CMD_PULSE:
	case CMD_CALIB:
	case CMD_GRIPPER:
	case CMD_ORIENT:
	case CMD_ZTEST:
	case CMD_SETZ0:
	case CMD_Z:
	case CMD_DIST:
	case CMD_PICK:
	case CMD_PLACE:
	case CMD_PICKPLACE:
	case CMD_PEN:
	case CMD_DRAW:
	case CMD_REPEAT:
	case CMD_SCAN:
	case CMD_MAP:
		return true;
	default:
		return false;
	}
}

/** Callback de microrl al presionar Enter. */
static int ejecutar(int argc, const char *const *argv)
{
	comando_t cmd;
	parser_res_t res = Parser_Interpretar(argc, argv, &cmd);

	switch (res) {
	case PARSER_OK:
		break;
	case PARSER_VACIO:
		return 0;
	case PARSER_ERR_COMANDO:
		responder_error(ERR_SINTAXIS, "comando desconocido (HELP para ver la lista)");
		return 0;
	case PARSER_ERR_CANT_ARGS:
		responder_error(ERR_SINTAXIS, "cantidad de argumentos incorrecta");
		return 0;
	default:
		responder_error(ERR_SINTAXIS, "argumento invalido");
		return 0;
	}

	if (Secuencia_Activa() && necesita_estar_libre(cmd.id)) {
		responder_error(ERR_OCUPADO, "rutina en curso (STOP para detenerla)");
		return 0;
	}

	switch (cmd.id) {
	case CMD_HELP:
		imprimir(texto_ayuda);
		responder_ok(NULL);
		break;
	case CMD_STATUS:
		comando_status();
		break;
	case CMD_SPEED:
		comando_speed(cmd.valor);
		break;
	case CMD_ACCEL:
		comando_accel(cmd.valor);
		break;
	case CMD_ANGLE:
		comando_angle();
		break;
	case CMD_RAW:
		comando_raw(cmd.activar);
		break;
	case CMD_I2CSCAN:
		comando_i2cscan();
		break;
	case CMD_HOME:
		responder_movimiento(Motion_Home());
		break;
	case CMD_JOINT:
		responder_movimiento(Motion_MoverJuntas(cmd.q1_deg, cmd.q2_deg));
		break;
	case CMD_MOVE:
		comando_move(&cmd);
		break;
	case CMD_PULSE:
		comando_pulse(&cmd);
		break;
	case CMD_STOP:
		comando_stop();
		break;
	case CMD_TRACE:
		traza_activa = cmd.activar;
		responder_ok(traza_activa ? "trace=ON" : "trace=OFF");
		break;
	case CMD_SETZ0:
		/* La altura de la mesa esta medida con el cero anterior. */
		Busqueda_OlvidarMesa();
		responder_eje_z(EjeZ_FijarCero());
		break;
	case CMD_Z:
		responder_eje_z(EjeZ_MoverAltura(cmd.valor));
		break;
	case CMD_ORIENT:
		responder_eje_z(EjeZ_MoverOrientacion(cmd.valor));
		break;
	case CMD_ZTEST:
		responder_eje_z(EjeZ_Probar((motor_id_t)cmd.motor, cmd.valor, cmd.numero));
		break;
	case CMD_GRIPPER:
		comando_gripper(&cmd);
		break;
	case CMD_DIST:
		comando_dist();
		break;
	case CMD_PICK:
		if (cmd.usar_caja) {
			responder_secuencia(Secuencia_TomarCaja());
			break;
		}
		responder_secuencia(Secuencia_Tomar(cmd.x_mm, cmd.y_mm, cmd.rot_deg, cmd.tiene_rot,
											cmd.tiene_ancho ? cmd.ancho_mm : GRIPPER_TOMA_MM));
		break;
	case CMD_SCAN:
		responder_secuencia(Secuencia_Buscar(cmd.x_mm, cmd.y_mm, cmd.ancho_mm, cmd.alto_mm));
		break;
	case CMD_MAP:
		comando_map();
		break;
	case CMD_PLACE:
		responder_secuencia(Secuencia_Dejar(cmd.x_mm, cmd.y_mm));
		break;
	case CMD_PICKPLACE:
		responder_secuencia(Secuencia_Ciclo(cmd.x_mm, cmd.y_mm, cmd.x2_mm, cmd.y2_mm,
											cmd.rot_deg, cmd.tiene_rot,
											cmd.tiene_ancho ? cmd.ancho_mm : GRIPPER_TOMA_MM));
		break;
	case CMD_PEN:
		responder_secuencia(Secuencia_Lapiz(cmd.activar));
		break;
	case CMD_DRAW:
		comando_draw(&cmd);
		break;
	case CMD_REPEAT:
		responder_secuencia(Secuencia_Repetir(cmd.x_mm, cmd.y_mm, cmd.x2_mm, cmd.y2_mm, cmd.numero));
		break;
	case CMD_CALIB:
		responder_secuencia(Secuencia_Calibrar(cmd.numero));
		break;
	default:
		responder_error(ERR_SINTAXIS, "comando no disponible");
		break;
	}
	return 0;
}

void Consola_Init(bool reloj_hse)
{
	reloj_con_hse = reloj_hse;
	ultimo_rx = 0U;
	traza_activa = false;
	raw_en_vivo = false;
	midiendo_distancia = false;
	mapa_fila = -1;

	microrl_init(&rl, imprimir);
	microrl_set_execute_callback(&rl, ejecutar);

	imprimir(ENDL "SCARA v2 - consola lista. HELP para ver los comandos." ENDL);
	if (!reloj_hse) {
		imprimir("WARN reloj: el cristal HSE no arranco, se usa el HSI (+-1 %)" ENDL);
	}
	imprimir(rl.prompt_str);
}

void Consola_Procesar(void)
{
	uint8_t byte;

	for (uint32_t n = 0U; n < CONSOLA_BYTES_POR_VUELTA && UART_LeerByte(&byte); n++) {
		/* En RAW LIVE la tecla solo sirve para salir: no llega a microrl. */
		if (raw_en_vivo) {
			raw_en_vivo = false;
			Consola_ImprimirLog("OK fin");
			continue;
		}

		/* microrl ejecuta con LF. PuTTY/Tera Term mandan CR al apretar Enter y otros
		 * terminales CR+LF o solo LF: se normaliza todo a un unico LF. */
		if (byte == '\r') {
			microrl_insert_char(&rl, '\n');
		} else if (byte == '\n') {
			if (ultimo_rx != '\r') {
				microrl_insert_char(&rl, '\n');
			}
		} else {
			microrl_insert_char(&rl, byte);
		}
		ultimo_rx = byte;
	}
}

/** Linea de TRACE: lo comandado, lo medido (vacio si esa junta no tiene encoder) y Z. */
static void imprimir_traza(void)
{
	char linea[LARGO_RESPUESTA];
	char med[JUNTA_CANT][16];
	float q1, q2, x, y;

	for (int j = 0; j < JUNTA_CANT; j++) {
		float th;

		med[j][0] = '\0';
		if (Motion_Medida((junta_id_t)j, &th, NULL)) {
			(void)snprintf(med[j], sizeof(med[j]), "%.2f", (double)th);
		}
	}
	postura_actual(&q1, &q2, &x, &y);
	(void)snprintf(linea, sizeof(linea), "T,%lu,%.2f,%.2f,%.2f,%.2f,%s,%s,%.1f",
				   (unsigned long)Millis(), (double)x, (double)y, (double)q1, (double)q2,
				   med[JUNTA_HOMBRO], med[JUNTA_CODO], (double)EjeZ_Altura());
	Consola_ImprimirLog(linea);
}

/** Avisa el fin de un movimiento del brazo fuera de una rutina. */
static void informar_motion(motion_evento_t evento)
{
	char postura[LARGO_RESPUESTA / 2];
	char medida[JUNTA_CANT][40];
	char error[24];
	char linea[LARGO_RESPUESTA + 64];
	const char *titulo;

	if (evento == MOTION_EVENTO_NINGUNO) {
		return;
	}
	switch (evento) {
	case MOTION_EVENTO_FIN:				titulo = "DONE"; break;
	case MOTION_EVENTO_DETENIDO:		titulo = "STOPPED"; break;
	case MOTION_EVENTO_FALLA_ENCODER:	titulo = "FAULT error o lectura de encoder: HOME para rearmar, en"; break;
	default:							titulo = "FAULT movimiento abortado en"; break;
	}
	texto_postura(postura, sizeof(postura));
	texto_medida(medida[JUNTA_HOMBRO], sizeof(medida[JUNTA_HOMBRO]), JUNTA_HOMBRO);
	texto_medida(medida[JUNTA_CODO], sizeof(medida[JUNTA_CODO]), JUNTA_CODO);
	texto_error_posicion(error, sizeof(error));
	(void)snprintf(linea, sizeof(linea), "%s %s%s%s%s", titulo, postura,
				   medida[JUNTA_HOMBRO], medida[JUNTA_CODO], error);
	Consola_ImprimirLog(linea);
}

/** Avisa el fin de un movimiento del eje Z fuera de una rutina. */
static void informar_eje_z(void)
{
	char linea[LARGO_RESPUESTA];

	switch (EjeZ_TomarEvento()) {
	case EJEZ_EVENTO_FIN:
		(void)snprintf(linea, sizeof(linea), "DONE z=%.1f rot=%.1f",
					   (double)EjeZ_Altura(), (double)EjeZ_Orientacion());
		break;
	case EJEZ_EVENTO_DETENIDO:
		(void)snprintf(linea, sizeof(linea), "STOPPED z=%.1f rot=%.1f",
					   (double)EjeZ_Altura(), (double)EjeZ_Orientacion());
		break;
	case EJEZ_EVENTO_ATASCO:
		(void)snprintf(linea, sizeof(linea), "FAULT eje Z atascado: motores cortados, SETZ0 para rearmar");
		break;
	case EJEZ_EVENTO_PRUEBA_FIN:
		(void)snprintf(linea, sizeof(linea), "DONE ztest motor=%c pulsos=%lu",
					   (EjeZ_MotorPrueba() == MOTOR_A) ? 'A' : 'B', (unsigned long)EjeZ_PulsosPrueba());
		break;
	default:
		return;
	}
	Consola_ImprimirLog(linea);
}

/** Avisa el fin de un movimiento de la garra fuera de una rutina. */
static void informar_gripper(void)
{
	char linea[LARGO_RESPUESTA];

	if (Gripper_TomarFin()) {
		(void)snprintf(linea, sizeof(linea), "DONE gripper=%.1f pulso=%lu",
					   (double)Gripper_Apertura(), (unsigned long)Gripper_Pulso());
		Consola_ImprimirLog(linea);
	}
}

/** Agrega a una linea el resultado de un sensor de DIST. */
static size_t texto_distancia(char *linea, size_t largo, size_t usado, const char *nombre, vl53_id_t id)
{
	int n;

	if (!VL53L0X_Listo(id)) {
		return usado;
	}
	if (dist_lectura[id].con_retorno) {
		n = snprintf(linea + usado, largo - usado, " %s=%.0f", nombre, (double)dist_lectura[id].mm);
	} else {
		n = snprintf(linea + usado, largo - usado, " %s=SIN_OBJETO", nombre);
	}
	return (n > 0) ? usado + (size_t)n : usado;
}

/** Junta la medicion de cada sensor que pidio el comando DIST. */
static void informar_distancia(void)
{
	char linea[LARGO_RESPUESTA];
	laser_lectura_t l;

	if (!midiendo_distancia) {
		return;
	}
	switch (Laseres_Tick(&l)) {
	case LASER_NADA:
		return;
	case LASER_LECTURA:
		break;
	default:
		Laseres_Detener();
		midiendo_distancia = false;
		Consola_ImprimirLog("FAULT sensor de distancia");
		return;
	}

	dist_lectura[l.sensor] = l;
	dist_vista[l.sensor] = true;
	for (int i = 0; i < VL53_CANT; i++) {
		if (VL53L0X_Listo((vl53_id_t)i) && !dist_vista[i]) {
			return;
		}
	}
	Laseres_Detener();
	midiendo_distancia = false;

	size_t usado = (size_t)snprintf(linea, sizeof(linea), "DONE");

	usado = texto_distancia(linea, sizeof(linea), usado, "dist", VL53_A);
	(void)texto_distancia(linea, sizeof(linea), usado, "dist2", VL53_B);
	Consola_ImprimirLog(linea);
}

/** Imprime una fila de MAP: M,fila,altura,altura,... con el campo vacio si la celda no tiene datos. */
static void imprimir_fila_mapa(void)
{
	char linea[LARGO_FILA_MAPA];
	float mesa = 0.0f;

	if (mapa_fila < 0) {
		return;
	}
	if (mapa_fila >= Mapa_Filas() || !Busqueda_MesaDelMapa(&mesa)) {
		mapa_fila = -1;
		Consola_ImprimirLog("DONE map");
		return;
	}

	size_t usado = (size_t)snprintf(linea, sizeof(linea), "M,%d", mapa_fila);

	for (int c = 0; c < Mapa_Columnas() && usado < sizeof(linea); c++) {
		float d;
		int n;

		if (Mapa_Celda(mapa_fila, c, &d)) {
			n = snprintf(linea + usado, sizeof(linea) - usado, ",%.0f", (double)(mesa - d));
		} else {
			n = snprintf(linea + usado, sizeof(linea) - usado, ",");
		}
		usado += (n > 0) ? (size_t)n : 0U;
	}
	mapa_fila++;
	Consola_ImprimirLog(linea);
}

static const char *nombre_motivo(seq_motivo_t motivo)
{
	switch (motivo) {
	case SEQ_MOTIVO_BRAZO:			return "brazo";
	case SEQ_MOTIVO_EJE_Z:			return "eje Z";
	case SEQ_MOTIVO_GRIPPER:		return "garra";
	case SEQ_MOTIVO_SENSOR:			return "sensor de distancia";
	case SEQ_MOTIVO_NO_ALINEADO:	return "los dos sensores no ven lo mismo";
	case SEQ_MOTIVO_SIN_CAJA:		return "no se encontro ninguna caja";
	case SEQ_MOTIVO_MUY_BAJO:		return "un sensor quedo muy cerca: subir Z_BARRIDO_MM";
	case SEQ_MOTIVO_CENTRADO:		return "no se encontraron los bordes de la caja";
	case SEQ_MOTIVO_CENTRADO_LEJOS:	return "las pasadas sobre la caja salen del espacio de trabajo";
	case SEQ_MOTIVO_CAJA_RARA:		return "lo medido no coincide con las medidas de la caja";
	case SEQ_MOTIVO_SIN_OBJETO:		return "sin objeto al alcance del sensor";
	case SEQ_MOTIVO_OBJETO_LEJOS:	return "objeto fuera del recorrido del eje Z";
	case SEQ_MOTIVO_ENCODER:		return "encoder";
	default:						return "?";
	}
}

/** Lineas de resultado de REPEAT, una por punto. */
static void informar_repeticion(void)
{
	char linea[LARGO_RESPUESTA];
	metricas_res_t r;
	float e_res;

	for (int punto = 0; punto < SECUENCIA_PUNTOS_REPEAT; punto++) {
		if (Secuencia_ResultadoRepetir(punto, &r, &e_res)) {
			(void)snprintf(linea, sizeof(linea),
						   "REPEAT punto=%d n=%lu e_media=%.3f e_desvio=%.3f e_max=%.3f exactitud=%.3f repetibilidad=%.3f e_resolucion=%.3f",
						   punto + 1, (unsigned long)r.n, (double)r.e_media, (double)r.e_desvio,
						   (double)r.e_max, (double)r.exactitud, (double)r.repetibilidad, (double)e_res);
			Consola_ImprimirLog(linea);
		}
	}
}

/** Avisos de la rutina en curso. */
static void informar_secuencia(void)
{
	char postura[LARGO_RESPUESTA / 2];
	char linea[LARGO_RESPUESTA + 64];
	seq_evento_t ev;

	if (!Secuencia_TomarEvento(&ev)) {
		return;
	}

	switch (ev.tipo) {
	case SEQ_EV_PASO:
		(void)snprintf(linea, sizeof(linea), "SEQ paso=%lu %s a=%.1f b=%.1f",
					   (unsigned long)ev.paso, ev.nombre, (double)ev.a, (double)ev.b);
		break;
	case SEQ_EV_DISTANCIA:
		(void)snprintf(linea, sizeof(linea), "SEQ objeto dist=%.1f z_toma=%.1f", (double)ev.a, (double)ev.b);
		break;
	case SEQ_EV_MEDIDA:
		(void)snprintf(linea, sizeof(linea), "R,%lu,%d,%.2f,%.2f,%.2f,%.2f,%.3f",
					   (unsigned long)ev.ciclo, ev.punto + 1, (double)ev.x_cmd, (double)ev.y_cmd,
					   (double)ev.x_med, (double)ev.y_med, (double)ev.e);
		break;
	case SEQ_EV_CALIB_PUNTO:
		(void)snprintf(linea, sizeof(linea), "C,%d,%.2f,%.2f", ev.junta, (double)ev.cmd_deg, (double)ev.enc_deg);
		break;
	case SEQ_EV_CALIB_AJUSTE:
		(void)snprintf(linea, sizeof(linea),
					   "CALIB junta=%d ENC_SIGN=%.0f ENC_ZERO_DEG=%.2f ganancia=%.4f RANGE_DEG=%.1f ZERO_SERVO_DEG=%.1f residuo=%.2f",
					   ev.junta, (double)ev.signo, (double)ev.cero_deg, (double)ev.ganancia,
					   (double)ev.rango_deg, (double)ev.zero_servo_deg, (double)ev.residuo_deg);
		break;
	case SEQ_EV_MAPA:
		if (ev.cantidad > 0) {
			(void)snprintf(linea, sizeof(linea), "SEQ mapa cajas=%d x=%.1f y=%.1f rot=%.1f largo=%.0f ancho=%.0f mesa=%.0f",
						   ev.cantidad, (double)ev.mancha.x_mm, (double)ev.mancha.y_mm, (double)ev.mancha.yaw_deg,
						   (double)ev.mancha.largo_mm, (double)ev.mancha.ancho_mm, (double)ev.a);
		} else {
			(void)snprintf(linea, sizeof(linea), "SEQ mapa cajas=0 mesa=%.0f", (double)ev.a);
		}
		break;
	case SEQ_EV_CENTRADO:
		(void)snprintf(linea, sizeof(linea), "SEQ eje x=%.1f y=%.1f rot=%.1f ancho=%.1f",
					   (double)ev.x_cmd, (double)ev.y_cmd, (double)ev.a, (double)ev.b);
		break;
	case SEQ_EV_CAJA:
		(void)snprintf(linea, sizeof(linea), "CAJA x=%.1f y=%.1f rot=%.1f largo=%.1f ancho=%.1f alto=%.1f tipo=%d",
					   (double)ev.caja.x_mm, (double)ev.caja.y_mm, (double)ev.caja.yaw_deg,
					   (double)ev.caja.largo_mm, (double)ev.caja.ancho_mm, (double)ev.caja.alto_mm, ev.caja.tipo);
		break;
	case SEQ_EV_FIN:
		informar_repeticion();
		texto_postura(postura, sizeof(postura));
		(void)snprintf(linea, sizeof(linea), "DONE %s z=%.1f rot=%.1f grip=%.1f", postura,
					   (double)EjeZ_Altura(), (double)EjeZ_Orientacion(), (double)Gripper_Apertura());
		break;
	case SEQ_EV_DETENIDA:
		texto_postura(postura, sizeof(postura));
		(void)snprintf(linea, sizeof(linea), "STOPPED %s", postura);
		break;
	case SEQ_EV_FALLA:
		(void)snprintf(linea, sizeof(linea), "FAULT rutina cortada: %s (paso %lu)",
					   nombre_motivo(ev.motivo), (unsigned long)ev.paso);
		break;
	default:
		return;
	}
	Consola_ImprimirLog(linea);
}

void Consola_DespuesDelTick(void)
{
	bool hay_rutina = Secuencia_Activa();
	motion_evento_t evento = hay_rutina ? MOTION_EVENTO_NINGUNO : Motion_TomarEvento();
	bool en_movimiento = Motion_Ocupado() || EjeZ_Ocupado();

	/* La traza incluye el tick en que termina el movimiento (trae el punto final). */
	if (traza_activa && (en_movimiento || evento != MOTION_EVENTO_NINGUNO)) {
		imprimir_traza();
	}

	informar_secuencia();
	if (!hay_rutina && !Secuencia_Activa()) {
		informar_motion(evento);
		informar_eje_z();
		informar_gripper();
		informar_distancia();
		imprimir_fila_mapa();
	}

	if (raw_en_vivo) {
		raw_ms += TICK_MS;
		if (raw_ms >= RAW_LIVE_PERIODO_MS) {
			char linea[LARGO_RESPUESTA];

			raw_ms = 0U;
			(void)leer_raw(linea, sizeof(linea));
			Consola_ImprimirLog(linea);
		}
	}
}

void Consola_ImprimirLog(const char *mensaje)
{
	char linea[_COMMAND_LINE_LEN + 1];
	char cursor[16];

	/* Volver al inicio, borrar la linea que se estaba tipeando e imprimir el mensaje. */
	imprimir("\r\033[K");
	imprimir(mensaje);
	imprimir(ENDL);

	/* Redibujar prompt + lo tipeado. microrl guarda los espacios como '\0'. */
	imprimir(rl.prompt_str);
	if (rl.cmdlen > 0) {
		int i;
		for (i = 0; i < rl.cmdlen && i < _COMMAND_LINE_LEN; i++) {
			linea[i] = (rl.cmdline[i] == '\0') ? ' ' : rl.cmdline[i];
		}
		linea[i] = '\0';
		imprimir(linea);
	}

	/* Devolver el cursor a donde estaba el usuario. */
	int atras = rl.cmdlen - rl.cursor;
	if (atras > 0) {
		(void)snprintf(cursor, sizeof(cursor), "\033[%dD", atras);
		imprimir(cursor);
	}
}
