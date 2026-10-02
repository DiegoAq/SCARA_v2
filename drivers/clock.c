/**
 * @file clock.c
 * @brief Configuracion del arbol de relojes (CMSIS, sin HAL).
 */
#include "clock.h"

#include "stm32f4xx.h"

/* VCO in = fuente / M = 1 MHz; VCO out = 1 MHz * N = 192 MHz; SYSCLK = VCO / P = 96 MHz;
 * salida de 48 MHz (USB, sin uso) = VCO / Q. */
#define PLL_M_HSE		25U		/* cristal de 25 MHz de la Black Pill */
#define PLL_M_HSI		16U		/* HSI de 16 MHz */
#define PLL_N			192U
#define PLL_P			2U
#define PLL_Q			4U

/* Regulador en escala 1 (VOS = 0b11): el F411 sale de reset en escala 2, que solo
 * garantiza hasta 84 MHz (RM0383, PWR_CR). */
#define PWR_VOS_ESCALA1	PWR_CR_VOS

/* Vueltas de espera. Todavia no hay SysTick; a 16 MHz esto es del orden de 200 ms,
 * muy por encima del arranque de un cristal (pocos ms). */
#define TIMEOUT_VUELTAS	400000U

/**
 * @brief Espera a que (registro & mascara) == valor, con timeout.
 * @return false si se agoto el tiempo.
 */
static bool esperar(const volatile uint32_t *registro, uint32_t mascara, uint32_t valor)
{
	for (uint32_t i = 0U; i < TIMEOUT_VUELTAS; i++) {
		if ((*registro & mascara) == valor) {
			return true;
		}
	}
	return false;
}

bool Clock_Init(void)
{
	bool con_hse;

	/* Partir siempre de HSI con el PLL apagado: PLLCFGR no se puede escribir con el
	 * PLL andando (p. ej. si se llega aca desde un bootloader o un reset por software). */
	RCC->CR |= RCC_CR_HSION;
	(void)esperar(&RCC->CR, RCC_CR_HSIRDY, RCC_CR_HSIRDY);
	RCC->CFGR &= ~RCC_CFGR_SW;
	(void)esperar(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_HSI);
	RCC->CR &= ~RCC_CR_PLLON;
	(void)esperar(&RCC->CR, RCC_CR_PLLRDY, 0U);

	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	(void)RCC->APB1ENR;		/* el periferico tarda un ciclo en quedar habilitado */
	PWR->CR |= PWR_VOS_ESCALA1;

	RCC->CR |= RCC_CR_HSEON;
	con_hse = esperar(&RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY);
	if (!con_hse) {
		RCC->CR &= ~RCC_CR_HSEON;
	}

	/* Leer-modificar-escribir: PLLCFGR tiene bits reservados que no valen 0 en reset. */
	RCC->PLLCFGR = (RCC->PLLCFGR & ~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP
									 | RCC_PLLCFGR_PLLQ | RCC_PLLCFGR_PLLSRC))
				 | ((con_hse ? PLL_M_HSE : PLL_M_HSI) << RCC_PLLCFGR_PLLM_Pos)
				 | (PLL_N << RCC_PLLCFGR_PLLN_Pos)
				 | (((PLL_P / 2U) - 1U) << RCC_PLLCFGR_PLLP_Pos)
				 | (PLL_Q << RCC_PLLCFGR_PLLQ_Pos)
				 | (con_hse ? RCC_PLLCFGR_PLLSRC_HSE : RCC_PLLCFGR_PLLSRC_HSI);

	RCC->CR |= RCC_CR_PLLON;
	if (!esperar(&RCC->CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY)) {
		/* Sin PLL se sigue con el HSI a 16 MHz; la flash no necesita wait states. */
		RCC->CR &= ~RCC_CR_PLLON;
		SystemCoreClockUpdate();
		return false;
	}

	/* Antes de subir la frecuencia: a 96 MHz y 3,3 V la flash necesita 3 wait states. */
	FLASH->ACR = FLASH_ACR_LATENCY_3WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
	(void)esperar(&FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_3WS);

	/* AHB /1, APB1 /2 (maximo 50 MHz), APB2 /1. */
	RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2))
			  | RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1;

	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
	if (!esperar(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL)) {
		SystemCoreClockUpdate();
		return false;
	}

	/* SystemCoreClockUpdate() lo calcula leyendo los registros (con HSE_VALUE o
	 * HSI_VALUE segun la fuente): sirve de verificacion contra CLOCK_SYSCLK_HZ. */
	SystemCoreClockUpdate();
	return con_hse && (SystemCoreClock == CLOCK_SYSCLK_HZ);
}
