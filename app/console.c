/**
 * @file console.c
 * @brief Consola de comandos: microrl + parser + respuestas OK/ERR.
 */
#include "console.h"

#include <stdint.h>
#include <stdio.h>

#include "config.h"
#include "i2c.h"
#include "microrl.h"
#include "motion.h"
#include "parser.h"
#include "uart.h"

#define LARGO_RESPUESTA		160

/* Rango de direcciones I2C de 7 bits validas (las demas estan reservadas por la norma). */
#define I2C_DIR_MIN			0x08U
#define I2C_DIR_MAX			0x77U

static microrl_t rl;
static bool reloj_con_hse;
static uint8_t ultimo_rx;

static const char texto_ayuda[] =
	"Comandos (no distingue mayusculas):" ENDL
	"  HELP                 esta lista" ENDL
	"  STATUS               estado y parametros activos" ENDL
	"  SPEED v              velocidad de MOVE (mm/s)" ENDL
	"  ACCEL a              aceleracion de MOVE (mm/s2)" ENDL
	"  MOVE X Y [Z]         recta al punto (mm)               [E5]" ENDL
	"  JOINT q1 q2          juntas a angulos theta (grados)   [E3]" ENDL
	"  HOME | STOP          ir a HOME | frenar con rampa      [E5]" ENDL
	"  PULSE n us           pulso crudo a un servo            [E3]" ENDL
	"  ANGLE                angulo de cada encoder y de cada junta (grados)" ENDL
	"  RAW                  cuentas del ADC (0..4095) y ruido de cada encoder" ENDL
	"  I2CSCAN              direcciones que contestan en cada bus I2C" ENDL
	"  CALIB                calibracion guiada                [E4]" ENDL
	"  TRACE ON|OFF         CSV por tick                      [E5]" ENDL
	"  GRIPPER | ORIENT | ZTEST | SETZ0                       [E7]" ENDL;

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

static void comando_status(void)
{
	char linea[LARGO_RESPUESTA];

	/* El maximo del tick se reinicia en cada STATUS: muestra lo peor desde el anterior. */
	(void)snprintf(linea, sizeof(linea),
				   "estado=%s speed=%.1f accel=%.1f tick_max_ms=%lu rx_perdidos=%lu tx_perdidos=%lu reloj=%s",
				   Motion_NombreEstado(Motion_Estado()),
				   (double)Motion_Velocidad(), (double)Motion_Aceleracion(),
				   (unsigned long)Motion_PeriodoTickMaxMs(true),
				   (unsigned long)UART_BytesPerdidosRx(), (unsigned long)UART_BytesPerdidosTx(),
				   reloj_con_hse ? "HSE" : "HSI");
	responder_ok(linea);
}

/** ANGLE: angulo de cada encoder (0..360) y de cada junta. */
static void comando_angle(void)
{
	char linea[LARGO_RESPUESTA];
	float enc[JUNTA_CANT];
	float theta[JUNTA_CANT];

	for (int j = 0; j < JUNTA_CANT; j++) {
		if (!Motion_LeerJuntaMedida((junta_id_t)j, &enc[j], &theta[j])) {
			responder_error(ERR_ENCODER, "el ADC no responde");
			return;
		}
	}

	(void)snprintf(linea, sizeof(linea), "enc1=%.2f q1=%.2f enc2=%.2f q2=%.2f",
				   (double)enc[JUNTA_HOMBRO], (double)theta[JUNTA_HOMBRO],
				   (double)enc[JUNTA_CODO], (double)theta[JUNTA_CODO]);
	responder_ok(linea);
}

/**
 * RAW: cuentas del ADC (0..4095) de cada encoder y el ruido de la lectura (maximo menos
 * minimo de las conversiones promediadas). Por ADC no hay otra forma de ver si el sensor
 * anda bien: un cable suelto o un iman mal puesto se nota como ruido alto.
 */
static void comando_raw(void)
{
	char linea[LARGO_RESPUESTA];
	uint16_t cuentas[JUNTA_CANT];
	uint16_t ruido[JUNTA_CANT];

	for (int j = 0; j < JUNTA_CANT; j++) {
		if (!Motion_LeerEncoderCrudo((junta_id_t)j, &cuentas[j], &ruido[j])) {
			responder_error(ERR_ENCODER, "el ADC no responde");
			return;
		}
	}

	(void)snprintf(linea, sizeof(linea), "raw1=%u ruido1=%u raw2=%u ruido2=%u",
				   (unsigned)cuentas[JUNTA_HOMBRO], (unsigned)ruido[JUNTA_HOMBRO],
				   (unsigned)cuentas[JUNTA_CODO], (unsigned)ruido[JUNTA_CODO]);
	responder_ok(linea);
}

/**
 * I2CSCAN: direcciones que contestan en cada bus. Los encoders ya no usan I2C; queda como
 * diagnostico para el VL53L0X. Recorre 112 direcciones por bus y puede atrasar algun tick:
 * no usarlo con el brazo en movimiento. El bus se inicializa aca porque nadie mas lo usa.
 */
static void comando_i2cscan(void)
{
	char texto[16];

	imprimir("OK");
	for (int bus = 0; bus < I2C_CANT_BUSES; bus++) {
		int encontrados = 0;

		(void)snprintf(texto, sizeof(texto), " i2c%d=", bus + 1);
		imprimir(texto);
		if (I2C_Init((i2c_bus_t)bus) != I2C_OK) {
			imprimir("BUS_ERROR");
			continue;
		}

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
		comando_raw();
		break;
	case CMD_I2CSCAN:
		comando_i2cscan();
		break;
	default:
		responder_error(ERR_SINTAXIS, "comando no disponible todavia en esta etapa");
		break;
	}
	return 0;
}

void Consola_Init(bool reloj_hse)
{
	reloj_con_hse = reloj_hse;
	ultimo_rx = 0U;

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
