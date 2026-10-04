/**
 * @file adc.c
 * @brief ADC1 por polling (CMSIS, sin HAL).
 */
#include "adc.h"

#include <stddef.h>

#include "config.h"
#include "stm32f4xx.h"
#include "systick.h"

/* PB0 = ADC1_IN8, PB1 = ADC1_IN9. */
#define PIN_IN8			0U
#define PIN_IN9			1U
#define MODER_ANALOGICO	3UL

/* Reloj del ADC = PCLK2 / 4 = 24 MHz (maximo 36 MHz). */
#define ADCPRE_DIV4		ADC_CCR_ADCPRE_0

/* Tiempo de muestreo de 144 ciclos (6 us): margen de sobra para la salida del AS5600,
 * que es un DAC con buffer. Conversion completa: (144 + 12) / 24 MHz = 6,5 us. */
#define SMP_144_CICLOS	6UL

/* Tiempo de estabilizacion del ADC tras encenderlo (tSTAB = 3 us; se espera de mas). */
#define ESTABILIZACION_MS	2U

void ADC_Init(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	(void)RCC->APB2ENR;		/* el periferico tarda un ciclo en quedar habilitado */

	/* Modo analogico, sin pull-up ni pull-down (cargarian la salida del sensor). */
	GPIOB->PUPDR &= ~((3UL << (PIN_IN8 * 2U)) | (3UL << (PIN_IN9 * 2U)));
	GPIOB->MODER |= (MODER_ANALOGICO << (PIN_IN8 * 2U)) | (MODER_ANALOGICO << (PIN_IN9 * 2U));

	ADC1_COMMON->CCR = (ADC1_COMMON->CCR & ~ADC_CCR_ADCPRE) | ADCPRE_DIV4;

	ADC1->CR1 = 0U;		/* 12 bits, sin scan ni interrupciones */
	ADC1->CR2 = 0U;		/* conversion simple, alineacion a derecha */
	ADC1->SMPR2 = (ADC1->SMPR2 & ~(ADC_SMPR2_SMP8 | ADC_SMPR2_SMP9))
				| (SMP_144_CICLOS << ADC_SMPR2_SMP8_Pos) | (SMP_144_CICLOS << ADC_SMPR2_SMP9_Pos);
	ADC1->SQR1 = 0U;	/* secuencia de una sola conversion */
	ADC1->CR2 = ADC_CR2_ADON;

	/* Unica espera activa del driver, y solo en la inicializacion. */
	uint32_t t0 = Millis();
	while ((uint32_t)(Millis() - t0) < ESTABILIZACION_MS) {
	}
}

bool ADC_LeerCanal(uint8_t canal, uint16_t *cuentas)
{
	if (cuentas == NULL || (canal != ADC_CANAL_PB0 && canal != ADC_CANAL_PB1)) {
		return false;
	}

	ADC1->SQR3 = canal;
	ADC1->SR = ~ADC_SR_EOC;		/* descarta un fin de conversion viejo */
	ADC1->CR2 |= ADC_CR2_SWSTART;

	uint32_t t0 = Millis();
	while ((ADC1->SR & ADC_SR_EOC) == 0U) {
		if ((uint32_t)(Millis() - t0) > ADC_TIMEOUT_MS) {
			return false;
		}
	}
	*cuentas = (uint16_t)(ADC1->DR & ADC_CUENTAS_MAX);	/* leer DR limpia EOC */
	return true;
}
