/**
 * @file encoder.h
 * @brief Encoders opticos del eje Z: un canal por motor, contados por interrupcion.
 *
 * Encoder A: PB4 (EXTI4). Encoder B: PB5 (EXTI5).
 *
 * LIMITACION: con un solo canal no hay direccion. El contador suma o resta segun el sentido
 * que le informa quien manda el motor, asi que si el eje se mueve solo, o sigue por inercia
 * despues de invertir, el error es de algunos pulsos. Por eso eje_z frena antes de invertir.
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/** Encoders, uno por motor. */
typedef enum {
	ENCODER_A = 0,
	ENCODER_B,
	ENCODER_CANT
} encoder_id_t;

/**
 * @brief Configura los pines, las interrupciones y el contador de ciclos del filtro.
 *        Los contadores arrancan en 0, con sentido positivo.
 */
void Encoder_Init(void);

/**
 * @brief Pulsos acumulados de un encoder, con signo.
 */
int32_t Encoder_Pulsos(encoder_id_t id);

/**
 * @brief Fija si los proximos pulsos suman o restan.
 * @param sentido +1 suma; -1 resta.
 */
void Encoder_FijarSentido(encoder_id_t id, int32_t sentido);

/**
 * @brief Pone el contador en 0. Llamar con el motor quieto.
 */
void Encoder_Reiniciar(encoder_id_t id);

#endif /* ENCODER_H */
