/**
 * @file encoder.c
 * @brief Conteo de los encoders opticos por EXTI (CMSIS, sin HAL).
 */
#include "encoder.h"

#include "config.h"
#include "stm32f4xx.h"

#define PIN_A			4U		/* PB4: sale de reset como NJTRST */
#define PIN_B			5U		/* PB5 */
#define EXTICR_PUERTO_B	1UL
#define PUPDR_PULLUP	1UL

/* Compartidos entre las ISR y el loop. 32 bits: en el M4 se leen de forma atomica. */
static volatile int32_t cuenta[ENCODER_CANT];
static volatile int32_t sentido_actual[ENCODER_CANT];
static volatile uint32_t ciclo_ultimo_flanco[ENCODER_CANT];

static uint32_t ciclos_filtro;

/** Entrada con pull-up. Pasar a modo entrada es lo que le saca el pin al JTAG. */
static void configurar_pin(uint32_t pin)
{
	GPIOB->MODER &= ~(3UL << (pin * 2UL));
	GPIOB->PUPDR = (GPIOB->PUPDR & ~(3UL << (pin * 2UL))) | (PUPDR_PULLUP << (pin * 2UL));
}

void Encoder_Init(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	(void)RCC->APB2ENR;		/* el periferico tarda un ciclo en quedar habilitado */

	for (int i = 0; i < ENCODER_CANT; i++) {
		cuenta[i] = 0;
		sentido_actual[i] = 1;
		ciclo_ultimo_flanco[i] = 0U;
	}

	/* DWT: contador de ciclos, para medir el tiempo entre flancos sin gastar un timer. */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0U;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	ciclos_filtro = ENCODER_FILTRO_US * (SystemCoreClock / 1000000U);

	configurar_pin(PIN_A);
	configurar_pin(PIN_B);

	/* Las lineas 4 y 5 estan en EXTICR[1]. Las otras lineas del grupo 9_5 quedan apagadas en
	 * IMR: PB8 y PB9, que son del I2C, no disparan esta interrupcion. */
	SYSCFG->EXTICR[1] = (SYSCFG->EXTICR[1] & ~(SYSCFG_EXTICR2_EXTI4 | SYSCFG_EXTICR2_EXTI5))
					  | (EXTICR_PUERTO_B << SYSCFG_EXTICR2_EXTI4_Pos)
					  | (EXTICR_PUERTO_B << SYSCFG_EXTICR2_EXTI5_Pos);
	EXTI->RTSR |= EXTI_RTSR_TR4 | EXTI_RTSR_TR5;
	EXTI->PR = EXTI_PR_PR4 | EXTI_PR_PR5;
	EXTI->IMR |= EXTI_IMR_MR4 | EXTI_IMR_MR5;

	NVIC_SetPriority(EXTI4_IRQn, ENCODER_PRIORIDAD_IRQ);
	NVIC_SetPriority(EXTI9_5_IRQn, ENCODER_PRIORIDAD_IRQ);
	NVIC_EnableIRQ(EXTI4_IRQn);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
}

int32_t Encoder_Pulsos(encoder_id_t id)
{
	return (id < ENCODER_CANT) ? cuenta[id] : 0;
}

void Encoder_FijarSentido(encoder_id_t id, int32_t sentido)
{
	if (id < ENCODER_CANT) {
		sentido_actual[id] = (sentido >= 0) ? 1 : -1;
	}
}

void Encoder_Reiniciar(encoder_id_t id)
{
	if (id < ENCODER_CANT) {
		cuenta[id] = 0;
	}
}

/** Cuenta un flanco, salvo que llegue demasiado cerca del anterior (rebote de la ranura). */
static void contar_flanco(encoder_id_t id)
{
	uint32_t ahora = DWT->CYCCNT;

	if ((uint32_t)(ahora - ciclo_ultimo_flanco[id]) >= ciclos_filtro) {
		ciclo_ultimo_flanco[id] = ahora;
		cuenta[id] += sentido_actual[id];
	}
}

/** ISR de PB4 (nombre fijado por la tabla de vectores del startup). */
void EXTI4_IRQHandler(void)
{
	EXTI->PR = EXTI_PR_PR4;
	contar_flanco(ENCODER_A);
}

/** ISR de PB5. */
void EXTI9_5_IRQHandler(void)
{
	EXTI->PR = EXTI_PR_PR5;
	contar_flanco(ENCODER_B);
}
