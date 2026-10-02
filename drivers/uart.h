/**
 * @file uart.h
 * @brief Consola serie por USART1 (PA9 TX / PA10 RX), 8N1, sin bloqueos.
 *
 * RX y TX por interrupcion con ring buffers. Ninguna funcion espera al hardware: si
 * el buffer de TX esta lleno, los bytes que no entran se descartan y se cuentan.
 * Las funciones de envio se llaman solo desde el loop principal, nunca desde una ISR.
 */
#ifndef UART_H
#define UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Configura USART1 a UART_BAUDIOS, 8N1, y habilita la interrupcion de RX.
 *
 * El divisor se calcula con SystemCoreClock (APB2 /1): llamar despues de Clock_Init().
 */
void UART_Init(void);

/**
 * @brief Saca un byte recibido del buffer de RX.
 * @param byte Salida.
 * @return false si no hay bytes pendientes.
 */
bool UART_LeerByte(uint8_t *byte);

/**
 * @brief Encola una cadena terminada en '\0' para transmitir. No bloquea.
 * @param texto Cadena (sin el '\0').
 */
void UART_EnviarCadena(const char *texto);

/**
 * @brief Encola un bloque de bytes para transmitir. No bloquea.
 * @param datos Bytes a enviar.
 * @param longitud Cantidad de bytes. Lo que no entre en el buffer se descarta.
 */
void UART_EnviarDatos(const uint8_t *datos, size_t longitud);

/**
 * @brief Bytes recibidos que se perdieron (buffer de RX lleno u overrun del hardware).
 */
uint32_t UART_BytesPerdidosRx(void);

/**
 * @brief Bytes a transmitir que se descartaron por buffer de TX lleno.
 */
uint32_t UART_BytesPerdidosTx(void);

#endif /* UART_H */
