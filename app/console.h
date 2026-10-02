/**
 * @file console.h
 * @brief Consola de comandos sobre microrl (SPEC 10).
 *
 * Cada comando responde UNA linea que empieza con `OK` o `ERR <codigo> <texto>`
 * (HELP manda antes la lista de comandos). Pensado para parsearse despues desde una HMI.
 */
#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdbool.h>

/** Codigos de error de la consola (SPEC 10). */
typedef enum {
	ERR_SINTAXIS = 1,	/**< Sintaxis o argumento invalido */
	ERR_RADIO = 2,		/**< Fuera del espacio de trabajo */
	ERR_LIMITE = 3,		/**< Limite articular */
	ERR_OCUPADO = 4,	/**< Hay un movimiento en curso */
	ERR_ENCODER = 5,	/**< Falla de encoder o iman */
	ERR_RANGO = 6		/**< Parametro fuera de rango */
} consola_err_t;

/**
 * @brief Inicializa microrl e imprime el saludo y el prompt.
 *
 * Llamar despues de UART_Init().
 * @param reloj_hse Resultado de Clock_Init(): si es false se avisa con un WARN.
 */
void Consola_Init(bool reloj_hse);

/**
 * @brief Pasa a microrl los bytes recibidos (hasta CONSOLA_BYTES_POR_VUELTA por llamada).
 *
 * Llamar en cada vuelta del loop principal. Ejecuta los comandos completos.
 */
void Consola_Procesar(void);

/**
 * @brief Imprime un mensaje asincrono (DONE, WARN...) sin romper la linea que se esta tipeando.
 *
 * Borra la linea actual, imprime el mensaje y vuelve a dibujar el prompt con lo tipeado
 * y el cursor donde estaba. Adaptado de CLI_ImprimirLog del TP2.
 * @param mensaje Texto sin salto de linea final.
 */
void Consola_ImprimirLog(const char *mensaje);

#endif /* CONSOLE_H */
