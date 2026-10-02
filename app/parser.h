/**
 * @file parser.h
 * @brief Interprete de comandos de la consola (nucleo portable, sin hardware).
 *
 * Recibe la linea ya separada en palabras (argc/argv, como la entrega microrl) y la
 * convierte en un comando_t. Solo valida *sintaxis*: los rangos (espacio de trabajo,
 * topes de velocidad, etc.) los valida quien ejecuta el comando.
 */
#ifndef PARSER_H
#define PARSER_H

#include <stdbool.h>
#include <stdint.h>

/** Comandos de la consola (SPEC 10). */
typedef enum {
	CMD_NINGUNO = 0,
	CMD_HELP,
	CMD_MOVE,
	CMD_JOINT,
	CMD_HOME,
	CMD_STOP,
	CMD_STATUS,
	CMD_SPEED,
	CMD_ACCEL,
	CMD_ANGLE,
	CMD_RAW,
	CMD_PULSE,
	CMD_CALIB,
	CMD_TRACE,
	CMD_I2CSCAN,
	CMD_GRIPPER,
	CMD_ORIENT,
	CMD_ZTEST,
	CMD_SETZ0
} cmd_id_t;

/** Variantes del comando GRIPPER. */
typedef enum {
	GRIPPER_ABRIR = 0,	/**< GRIPPER OPEN */
	GRIPPER_CERRAR,		/**< GRIPPER CLOSE */
	GRIPPER_APERTURA	/**< GRIPPER <mm> */
} gripper_modo_t;

/** Resultado del analisis. Todo lo que no es PARSER_OK se responde como ERR 1. */
typedef enum {
	PARSER_OK = 0,
	PARSER_VACIO,			/**< Linea sin palabras: no se responde nada */
	PARSER_ERR_COMANDO,		/**< Comando desconocido */
	PARSER_ERR_CANT_ARGS,	/**< Faltan o sobran argumentos */
	PARSER_ERR_ARGUMENTO	/**< Un argumento no es un numero/palabra valido */
} parser_res_t;

/** Comando ya interpretado. Solo son validos los campos del comando indicado. */
typedef struct {
	cmd_id_t id;
	float x_mm, y_mm, z_mm;		/**< MOVE */
	bool  tiene_z;				/**< MOVE: se indico Z */
	float q1_deg, q2_deg;		/**< JOINT */
	float valor;				/**< SPEED (mm/s), ACCEL (mm/s2), ORIENT (grados), GRIPPER (mm) */
	int32_t servo;				/**< PULSE: numero de servo */
	int32_t pulso_us;			/**< PULSE: ancho de pulso (us) */
	bool  activar;				/**< TRACE: ON = true */
	gripper_modo_t gripper;		/**< GRIPPER */
} comando_t;

/**
 * @brief Interpreta una linea de comando ya separada en palabras.
 *
 * No distingue mayusculas. En MOVE cada coordenada acepta `20` (posicional) o `X20`
 * (por nombre, en cualquier orden).
 *
 * @param argc Cantidad de palabras.
 * @param argv Palabras; argv[0] es el comando.
 * @param cmd Salida. Con cualquier resultado distinto de PARSER_OK queda id = CMD_NINGUNO.
 * @return PARSER_OK o el motivo del error.
 */
parser_res_t Parser_Interpretar(int argc, const char *const *argv, comando_t *cmd);

/**
 * @brief Convierte texto decimal a float: `[+-]digitos[.digitos]`, sin exponente.
 *
 * Implementacion propia para no arrastrar strtod (double por software) al firmware.
 * @return false si hay caracteres sobrantes, ningun digito o demasiados digitos.
 */
bool Parser_LeerFloat(const char *txt, float *valor);

/**
 * @brief Convierte texto a entero con signo: `[+-]digitos`.
 * @return false si hay caracteres sobrantes, ningun digito o mas de 9 digitos.
 */
bool Parser_LeerEntero(const char *txt, int32_t *valor);

#endif /* PARSER_H */
