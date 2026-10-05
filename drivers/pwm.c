/**
 * @file pwm.c
 * @brief PWM de servos con TIM3 y TIM4 (CMSIS, sin HAL).
 */
#include "pwm.h"

#include "clock.h"
#include "config.h"
#include "stm32f4xx.h"

/* 1 tick = 1 us: los timers de APB1 reciben 96 MHz (el prescaler de APB1 distinto de 1
 * duplica los 48 MHz del bus). Periodo de 20 000 ticks = 20 ms = 50 Hz. */
#define TICK_HZ				1000000UL
#define PRESCALER			((CLOCK_TIM_APB1_HZ / TICK_HZ) - 1UL)	/* 95 */
#define PERIODO_US			20000UL

#define MODER_AF			2UL
#define AF_TIM3_TIM4		2UL
#define OCM_PWM1			6UL		/* salida en alto mientras CNT < CCR */

/* Sin pulsos: en PWM modo 1, CCR = 0 deja la salida siempre en bajo. Se usa esto en vez
 * de apagar el canal (CCxE) porque asi el pin sigue manejado (no flota) y, gracias al
 * preload, pasar de "sin pulsos" a un pulso siempre empieza con un pulso entero. */
#define CCR_SIN_PULSOS		0UL

static uint32_t pulso_us[PWM_CANT];
static bool habilitado[PWM_CANT];

/** Registro de comparacion de cada salida. */
static volatile uint32_t *registro_ccr(pwm_id_t id)
{
	switch (id) {
	case PWM_SERVO_HOMBRO:	return &TIM3->CCR1;
	case PWM_SERVO_CODO:	return &TIM3->CCR2;
	default:				return &TIM4->CCR1;
	}
}

static void pin_af(GPIO_TypeDef *puerto, uint32_t pin)
{
	uint32_t idx = pin >> 3;
	uint32_t pos = (pin & 7UL) * 4UL;

	puerto->AFR[idx] = (puerto->AFR[idx] & ~(0xFUL << pos)) | (AF_TIM3_TIM4 << pos);
	puerto->MODER = (puerto->MODER & ~(3UL << (pin * 2UL))) | (MODER_AF << (pin * 2UL));
}

/** Base de tiempo comun: 1 us por tick, 20 ms de periodo, con preload de ARR. */
static void configurar_base(TIM_TypeDef *tim)
{
	tim->CR1 = 0U;
	tim->PSC = PRESCALER;
	tim->ARR = PERIODO_US - 1UL;
	tim->CR1 = TIM_CR1_ARPE;
}

void PWM_Init(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;
	(void)RCC->APB1ENR;		/* el periferico tarda un ciclo en quedar habilitado */

	for (int i = 0; i < PWM_CANT; i++) {
		pulso_us[i] = (PULSE_MIN_US + PULSE_MAX_US) / 2U;
		habilitado[i] = false;
	}

	configurar_base(TIM3);
	configurar_base(TIM4);

	/* PWM modo 1 con preload del CCR (OCxPE) en cada canal; todos arrancan sin pulsos. */
	TIM3->CCMR1 = (OCM_PWM1 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE
				| (OCM_PWM1 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
	TIM4->CCMR1 = (OCM_PWM1 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
	TIM3->CCR1 = CCR_SIN_PULSOS;
	TIM3->CCR2 = CCR_SIN_PULSOS;
	TIM4->CCR1 = CCR_SIN_PULSOS;
	TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
	TIM4->CCER = TIM_CCER_CC1E;

	/* UG carga PSC, ARR y los CCR en sus registros activos antes de arrancar. */
	TIM3->EGR = TIM_EGR_UG;
	TIM4->EGR = TIM_EGR_UG;
	TIM3->CR1 |= TIM_CR1_CEN;
	TIM4->CR1 |= TIM_CR1_CEN;

	/* Los pines pasan al timer recien ahora, con la salida ya en bajo. */
	pin_af(GPIOA, 6UL);
	pin_af(GPIOA, 7UL);
	pin_af(GPIOB, 6UL);
}

void PWM_ConfigurarPulsoUs(pwm_id_t id, uint32_t us)
{
	if (id >= PWM_CANT) {
		return;
	}
	if (us < PULSE_MIN_US) {
		us = PULSE_MIN_US;
	} else if (us > PULSE_MAX_US) {
		us = PULSE_MAX_US;
	}
	pulso_us[id] = us;
	if (habilitado[id]) {
		*registro_ccr(id) = us;
	}
}

void PWM_Habilitar(pwm_id_t id, bool habilitar)
{
	if (id >= PWM_CANT) {
		return;
	}
	habilitado[id] = habilitar;
	*registro_ccr(id) = habilitar ? pulso_us[id] : CCR_SIN_PULSOS;
}
