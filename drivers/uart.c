/**
 * @file uart.c
 * @brief USART1 por interrupcion con ring buffers de RX y TX (CMSIS, sin HAL).
 *
 * Cada ring buffer tiene un solo productor y un solo consumidor:
 *  - RX: la ISR escribe rx_cabeza, el loop escribe rx_cola.
 *  - TX: el loop escribe tx_cabeza, la ISR escribe tx_cola.
 * Cada indice lo modifica un solo lado y son de 16 bits (lectura atomica en el M4),
 * asi que no hace falta deshabilitar interrupciones.
 */
#include "uart.h"

#include "config.h"
#include "stm32f4xx.h"

#define RX_MASCARA		(UART_RX_BUF_BYTES - 1U)
#define TX_MASCARA		(UART_TX_BUF_BYTES - 1U)

_Static_assert((UART_RX_BUF_BYTES & RX_MASCARA) == 0U, "UART_RX_BUF_BYTES debe ser potencia de 2");
_Static_assert((UART_TX_BUF_BYTES & TX_MASCARA) == 0U, "UART_TX_BUF_BYTES debe ser potencia de 2");
_Static_assert(UART_TX_BUF_BYTES <= 32768U, "los indices son de 16 bits");

/* PA9 = USART1_TX, PA10 = USART1_RX, ambos AF7. */
#define PIN_TX			9U
#define PIN_RX			10U
#define AF_USART1		7U
#define MODER_AF		2U
#define PUPDR_PULLUP	1U

static volatile uint8_t rx_buf[UART_RX_BUF_BYTES];
static volatile uint16_t rx_cabeza;
static volatile uint16_t rx_cola;
static volatile uint32_t rx_perdidos;

static volatile uint8_t tx_buf[UART_TX_BUF_BYTES];
static volatile uint16_t tx_cabeza;
static volatile uint16_t tx_cola;
static uint32_t tx_perdidos;		/* solo lo toca el loop */

void UART_Init(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	(void)RCC->APB2ENR;		/* el periferico tarda un ciclo en quedar habilitado */

	GPIOA->MODER = (GPIOA->MODER & ~((3UL << (PIN_TX * 2U)) | (3UL << (PIN_RX * 2U))))
				 | (MODER_AF << (PIN_TX * 2U)) | (MODER_AF << (PIN_RX * 2U));
	GPIOA->AFR[1] = (GPIOA->AFR[1] & ~((0xFUL << ((PIN_TX - 8U) * 4U)) | (0xFUL << ((PIN_RX - 8U) * 4U))))
				  | (AF_USART1 << ((PIN_TX - 8U) * 4U)) | (AF_USART1 << ((PIN_RX - 8U) * 4U));
	/* Pull-up en RX: con el adaptador USB-serie desconectado la linea queda en reposo
	 * (alto) en vez de flotar y generar bytes basura. */
	GPIOA->PUPDR = (GPIOA->PUPDR & ~(3UL << (PIN_RX * 2U))) | (PUPDR_PULLUP << (PIN_RX * 2U));

	rx_cabeza = rx_cola = 0U;
	tx_cabeza = tx_cola = 0U;
	rx_perdidos = 0U;
	tx_perdidos = 0U;

	/* Oversampling x16: BRR = f_PCLK2 / baudios, redondeado. APB2 va /1, asi que
	 * f_PCLK2 = SystemCoreClock (tambien si Clock_Init cayo al HSI). 96 MHz -> 833,
	 * 115 246 baudios reales (+0,04 %). */
	USART1->CR1 = 0U;
	USART1->BRR = (SystemCoreClock + (UART_BAUDIOS / 2U)) / UART_BAUDIOS;
	USART1->CR2 = 0U;	/* 1 bit de stop */
	USART1->CR3 = 0U;	/* sin control de flujo */
	USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

	NVIC_SetPriority(USART1_IRQn, UART_PRIORIDAD_IRQ);
	NVIC_EnableIRQ(USART1_IRQn);
}

bool UART_LeerByte(uint8_t *byte)
{
	uint16_t cola = rx_cola;

	if (byte == NULL || cola == rx_cabeza) {
		return false;
	}
	*byte = rx_buf[cola];
	rx_cola = (uint16_t)((cola + 1U) & RX_MASCARA);
	return true;
}

void UART_EnviarDatos(const uint8_t *datos, size_t longitud)
{
	if (datos == NULL) {
		return;
	}

	uint16_t cabeza = tx_cabeza;
	size_t i;

	for (i = 0U; i < longitud; i++) {
		uint16_t siguiente = (uint16_t)((cabeza + 1U) & TX_MASCARA);
		if (siguiente == tx_cola) {
			break;		/* lleno: descartar en vez de esperar */
		}
		tx_buf[cabeza] = datos[i];
		cabeza = siguiente;
	}
	tx_perdidos += (uint32_t)(longitud - i);

	/* Publicar los bytes antes de habilitar TXE, para que la ISR los vea. */
	tx_cabeza = cabeza;
	if (i > 0U) {
		/* Leer-modificar-escribir de CR1 en competencia con la ISR, que solo apaga TXEIE
		 * cuando se vacia el buffer: en el peor caso queda una interrupcion de mas que
		 * encuentra el buffer vacio y lo vuelve a apagar. */
		USART1->CR1 |= USART_CR1_TXEIE;
	}
}

void UART_EnviarCadena(const char *texto)
{
	if (texto == NULL) {
		return;
	}

	size_t n = 0U;
	while (texto[n] != '\0') {
		n++;
	}
	UART_EnviarDatos((const uint8_t *)texto, n);
}

uint32_t UART_BytesPerdidosRx(void)
{
	return rx_perdidos;
}

uint32_t UART_BytesPerdidosTx(void)
{
	return tx_perdidos;
}

/** ISR de la USART1: solo mueve bytes entre el periferico y los buffers. */
void USART1_IRQHandler(void)
{
	uint32_t sr = USART1->SR;

	if ((sr & (USART_SR_RXNE | USART_SR_ORE)) != 0U) {
		/* Leer DR despues de SR limpia RXNE y tambien ORE/FE/NE. */
		uint8_t dato = (uint8_t)USART1->DR;

		if ((sr & USART_SR_ORE) != 0U) {
			rx_perdidos++;	/* llego un byte antes de leer el anterior */
		}
		if ((sr & USART_SR_RXNE) != 0U) {
			uint16_t siguiente = (uint16_t)((rx_cabeza + 1U) & RX_MASCARA);
			if (siguiente == rx_cola) {
				rx_perdidos++;
			} else {
				rx_buf[rx_cabeza] = dato;
				rx_cabeza = siguiente;
			}
		}
	}

	if ((sr & USART_SR_TXE) != 0U && (USART1->CR1 & USART_CR1_TXEIE) != 0U) {
		if (tx_cola != tx_cabeza) {
			USART1->DR = tx_buf[tx_cola];
			tx_cola = (uint16_t)((tx_cola + 1U) & TX_MASCARA);
		} else {
			USART1->CR1 &= ~USART_CR1_TXEIE;
		}
	}
}
