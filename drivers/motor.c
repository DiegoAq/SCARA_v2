/**
 * @file motor.c
 * @brief PWM de los motores DC con TIM2 y sentido por GPIO (CMSIS, sin HAL).
 */
#include "motor.h"

#include <stdbool.h>
#include <stdint.h>

#include "clock.h"
#include "config.h"
#include "stm32f4xx.h"

/* Sin prescaler: 96 MHz / 10 kHz = 9600 cuentas por periodo, sobra resolucion de duty. */
#define PERIODO_CUENTAS		(CLOCK_TIM_APB1_HZ / MOTOR_PWM_HZ)

#define MODER_SALIDA		1UL
#define MODER_AF			2UL
#define AF_TIM2				1UL
#define OCM_PWM1			6UL		/* salida en alto mientras CNT < CCR */

/** Pines de cada motor (todos en GPIOA). */
typedef struct {
	uint32_t pin_en;
	uint32_t pin_in1;
	uint32_t pin_in2;
} motor_cfg_t;

static const motor_cfg_t motores[MOTOR_CANT] = {
	[MOTOR_A] = { 0UL, 2UL, 3UL },
	[MOTOR_B] = { 1UL, 4UL, 5UL },
};

/** Registro de comparacion de cada enable. */
static volatile uint32_t *registro_ccr(motor_id_t id)
{
	return (id == MOTOR_A) ? &TIM2->CCR1 : &TIM2->CCR2;
}

static void modo_pin(uint32_t pin, uint32_t modo)
{
	GPIOA->MODER = (GPIOA->MODER & ~(3UL << (pin * 2UL))) | (modo << (pin * 2UL));
}

/** Pone IN1 e IN2. BSRR: escritura atomica, sin leer-modificar-escribir sobre ODR. */
static void escribir_sentido(const motor_cfg_t *m, bool in1, bool in2)
{
	GPIOA->BSRR = ((in1 ? 1UL : 1UL << 16) << m->pin_in1)
				| ((in2 ? 1UL : 1UL << 16) << m->pin_in2);
}

void Motor_Init(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	(void)RCC->APB1ENR;		/* el periferico tarda un ciclo en quedar habilitado */

	TIM2->CR1 = 0U;
	TIM2->PSC = 0U;
	TIM2->ARR = PERIODO_CUENTAS - 1UL;
	TIM2->CCMR1 = (OCM_PWM1 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE
				| (OCM_PWM1 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
	TIM2->CCR1 = 0U;
	TIM2->CCR2 = 0U;
	TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
	TIM2->CR1 = TIM_CR1_ARPE;
	TIM2->EGR = TIM_EGR_UG;		/* carga PSC, ARR y los CCR antes de arrancar */
	TIM2->CR1 |= TIM_CR1_CEN;

	/* Sentido en bajo antes de pasar a salida, y el enable al timer recien con CCR = 0: asi
	 * el L298N nunca ve un enable alto con los IN sin definir. */
	for (int i = 0; i < MOTOR_CANT; i++) {
		const motor_cfg_t *m = &motores[i];
		uint32_t idx = m->pin_en >> 3;
		uint32_t pos = (m->pin_en & 7UL) * 4UL;

		escribir_sentido(m, false, false);
		modo_pin(m->pin_in1, MODER_SALIDA);
		modo_pin(m->pin_in2, MODER_SALIDA);
		GPIOA->AFR[idx] = (GPIOA->AFR[idx] & ~(0xFUL << pos)) | (AF_TIM2 << pos);
		modo_pin(m->pin_en, MODER_AF);
	}
}

void Motor_ConfigurarDuty(motor_id_t id, float duty)
{
	if (id >= MOTOR_CANT) {
		return;
	}

	bool adelante = duty >= 0.0f;
	float modulo = adelante ? duty : -duty;

	if (!(modulo <= 1.0f)) {	/* tambien descarta NaN */
		modulo = 1.0f;
	}
	escribir_sentido(&motores[id], adelante, !adelante);
	*registro_ccr(id) = (uint32_t)(modulo * (float)PERIODO_CUENTAS);
}

void Motor_Frenar(motor_id_t id)
{
	if (id >= MOTOR_CANT) {
		return;
	}
	escribir_sentido(&motores[id], false, false);
	*registro_ccr(id) = PERIODO_CUENTAS;
}
