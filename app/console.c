/**
 * @file console.c
 * @brief Consola de comandos: microrl + parser + respuestas OK/ERR.
 */
#include "console.h"

#include <stdint.h>
#include <stdio.h>

#include "config.h"
#include "microrl.h"
#include "motion.h"
#include "parser.h"
#include "uart.h"

#define LARGO_RESPUESTA		160

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
	"  ANGLE | RAW          lectura de los AS5600             [E4]" ENDL
	"  CALIB | I2CSCAN      calibracion | escaneo I2C         [E4]" ENDL
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
	char linea[LARGO_RESPUESTA];

	(void)snprintf(linea, sizeof(linea), "ERR %d %s" ENDL, (int)codigo, texto);
	imprimir(linea);
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
