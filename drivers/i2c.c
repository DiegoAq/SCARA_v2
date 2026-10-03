/**
 * @file i2c.c
 * @brief I2C maestro por polling para el STM32F411 (CMSIS, sin HAL).
 *
 * El periferico I2C del F4 exige secuencias distintas para recibir 1, 2 o mas bytes
 * (RM0383, "Master receiver"); estan implementadas tal cual en recibir().
 */
#include "i2c.h"

#include <stdbool.h>

#include "config.h"
#include "stm32f4xx.h"
#include "systick.h"

/** Pines y periferico de un bus. Todos los pines estan en GPIOB. */
typedef struct {
	I2C_TypeDef *i2c;
	uint32_t rcc_apb1;		/**< Bit de habilitacion en RCC->APB1ENR */
	uint8_t pin_scl;
	uint8_t af_scl;
	uint8_t pin_sda;
	uint8_t af_sda;
} bus_cfg_t;

static const bus_cfg_t buses[I2C_CANT_BUSES] = {
	[I2C_BUS_1] = { I2C1, RCC_APB1ENR_I2C1EN, 8U, 4U, 9U, 4U },
	/* PB3 sale de reset como JTDO y como I2C2_SDA es AF9, no AF4. */
	[I2C_BUS_2] = { I2C2, RCC_APB1ENR_I2C2EN, 10U, 4U, 3U, 9U },
};

#define MODER_SALIDA	1UL
#define MODER_AF		2UL
#define OSPEEDR_RAPIDO	2UL
#define PUPDR_PULLUP	1UL

/* Tiempo de subida maximo de la norma: 300 ns en modo rapido, 1000 ns en estandar. */
#define T_SUBIDA_RAPIDO_NS		300U
#define T_SUBIDA_ESTANDAR_NS	1000U
#define FRECUENCIA_ESTANDAR_HZ	100000U
#define PULSOS_RECUPERACION		9

/** Frecuencia de APB1 leida de los registros (48 MHz en funcionamiento normal). */
static uint32_t pclk1_hz(void)
{
	uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;

	/* PPRE1: 0xx = /1, 100 = /2, 101 = /4, 110 = /8, 111 = /16. */
	return (ppre1 < 4U) ? SystemCoreClock : (SystemCoreClock >> (ppre1 - 3U));
}

static void modo_pin(uint8_t pin, uint32_t modo)
{
	GPIOB->MODER = (GPIOB->MODER & ~(3UL << (pin * 2U))) | (modo << (pin * 2U));
}

static void configurar_pin(uint8_t pin, uint8_t af)
{
	uint32_t idx = pin >> 3;
	uint32_t pos = (pin & 7U) * 4U;

	GPIOB->BSRR = 1UL << pin;		/* en alto antes de soltarlo como salida */
	GPIOB->OTYPER |= 1UL << pin;	/* drenador abierto */
	GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~(3UL << (pin * 2U))) | (OSPEEDR_RAPIDO << (pin * 2U));
	/* Pull-up interno (~40 k): no reemplaza a los de la placa del sensor, pero evita
	 * que la linea flote si el modulo esta desconectado. */
	GPIOB->PUPDR = (GPIOB->PUPDR & ~(3UL << (pin * 2U))) | (PUPDR_PULLUP << (pin * 2U));
	GPIOB->AFR[idx] = (GPIOB->AFR[idx] & ~(0xFUL << pos)) | ((uint32_t)af << pos);
	modo_pin(pin, MODER_AF);
}

/** Reinicia el periferico y carga la temporizacion. */
static void configurar_periferico(I2C_TypeDef *i2c)
{
	uint32_t pclk = pclk1_hz();
	uint32_t pclk_mhz = pclk / 1000000U;

	/* SWRST tambien libera un BUSY que quedo pegado. */
	i2c->CR1 = I2C_CR1_SWRST;
	i2c->CR1 = 0U;

	i2c->CR2 = pclk_mhz & I2C_CR2_FREQ;
#if I2C_FRECUENCIA_HZ > FRECUENCIA_ESTANDAR_HZ
	/* Modo rapido, DUTY = 0: T = 3 * CCR * Tpclk. Con 48 MHz y 400 kHz: CCR = 40, TRISE = 15. */
	i2c->CCR = I2C_CCR_FS | ((pclk / (3U * I2C_FRECUENCIA_HZ)) & I2C_CCR_CCR);
	i2c->TRISE = (pclk_mhz * T_SUBIDA_RAPIDO_NS) / 1000U + 1U;
#else
	/* Modo estandar: T = 2 * CCR * Tpclk. */
	i2c->CCR = (pclk / (2U * I2C_FRECUENCIA_HZ)) & I2C_CCR_CCR;
	i2c->TRISE = (pclk_mhz * T_SUBIDA_ESTANDAR_NS) / 1000U + 1U;
#endif
	i2c->CR1 = I2C_CR1_PE;
}

/** Espera corta entre flancos al manejar los pines a mano (solo en la recuperacion). */
static void pausa(void)
{
	for (volatile uint32_t i = 0U; i < I2C_RECUP_VUELTAS; i++) {
	}
}

/**
 * @brief Destraba el bus: un esclavo que quedo a mitad de un byte mantiene SDA en bajo
 *        hasta recibir los pulsos de reloj que le faltan.
 *
 * Maneja SCL/SDA como GPIO: hasta 9 pulsos de SCL, una condicion de STOP, y reinicia el
 * periferico. Es el unico busy-wait fuera de la inicializacion: dura menos de 0,2 ms y
 * solo corre cuando el bus ya fallo.
 */
static void recuperar_bus(const bus_cfg_t *b)
{
	uint32_t scl = 1UL << b->pin_scl;
	uint32_t sda = 1UL << b->pin_sda;

	b->i2c->CR1 &= ~I2C_CR1_PE;
	GPIOB->BSRR = scl | sda;
	modo_pin(b->pin_scl, MODER_SALIDA);
	modo_pin(b->pin_sda, MODER_SALIDA);
	pausa();

	for (int i = 0; i < PULSOS_RECUPERACION && (GPIOB->IDR & sda) == 0U; i++) {
		GPIOB->BSRR = scl << 16;
		pausa();
		GPIOB->BSRR = scl;
		pausa();
	}

	/* STOP: SDA sube con SCL en alto. */
	GPIOB->BSRR = sda << 16;
	pausa();
	GPIOB->BSRR = sda;
	pausa();

	modo_pin(b->pin_scl, MODER_AF);
	modo_pin(b->pin_sda, MODER_AF);
	configurar_periferico(b->i2c);
}

/**
 * @brief Espera una bandera de SR1 con timeout, atendiendo los errores.
 * @return I2C_OK cuando la bandera esta en 1.
 */
static i2c_res_t esperar(I2C_TypeDef *i2c, uint32_t bandera)
{
	uint32_t t0 = Millis();

	for (;;) {
		uint32_t sr1 = i2c->SR1;

		if ((sr1 & bandera) != 0U) {
			return I2C_OK;
		}
		if ((sr1 & I2C_SR1_AF) != 0U) {
			return I2C_ERR_NACK;
		}
		if ((sr1 & (I2C_SR1_BERR | I2C_SR1_ARLO)) != 0U) {
			return I2C_ERR_BUS;
		}
		if ((uint32_t)(Millis() - t0) > I2C_TIMEOUT_MS) {
			return I2C_ERR_TIMEOUT;
		}
	}
}

/** Espera a que el hardware termine de generar el STOP (el bit se limpia solo). */
static i2c_res_t esperar_stop(I2C_TypeDef *i2c)
{
	uint32_t t0 = Millis();

	while ((i2c->CR1 & I2C_CR1_STOP) != 0U) {
		if ((uint32_t)(Millis() - t0) > I2C_TIMEOUT_MS) {
			return I2C_ERR_TIMEOUT;
		}
	}
	return I2C_OK;
}

/** Limpia ADDR: lectura de SR1 seguida de SR2. */
static void limpiar_addr(I2C_TypeDef *i2c)
{
	(void)i2c->SR1;
	(void)i2c->SR2;
}

/**
 * @brief Deja el bus libre despues de una falla y devuelve el mismo error.
 *
 * Con NACK alcanza con un STOP. Con timeout o error de bus el periferico puede haber
 * quedado a mitad de una secuencia: se destraba y se reinicia.
 */
static i2c_res_t abortar(const bus_cfg_t *b, i2c_res_t motivo)
{
	I2C_TypeDef *i2c = b->i2c;

	if (motivo == I2C_ERR_NACK) {
		i2c->CR1 |= I2C_CR1_STOP;
		/* Las banderas de SR1 se limpian escribiendo 0 y un 1 no las altera: asi se
		 * limpia solo AF, sin leer-modificar-escribir. */
		i2c->SR1 = ~I2C_SR1_AF;
		i2c->CR1 &= ~(I2C_CR1_POS | I2C_CR1_ACK);
		if (esperar_stop(i2c) == I2C_OK) {
			return motivo;
		}
	}
	recuperar_bus(b);
	return motivo;
}

/** START (o START repetido) + direccion. Deja ADDR en 1 sin limpiar. */
static i2c_res_t iniciar(I2C_TypeDef *i2c, uint8_t dir7, bool lectura)
{
	i2c_res_t r;

	i2c->CR1 |= I2C_CR1_START;
	r = esperar(i2c, I2C_SR1_SB);
	if (r != I2C_OK) {
		return r;
	}
	i2c->DR = (uint32_t)(dir7 << 1) | (lectura ? 1U : 0U);
	return esperar(i2c, I2C_SR1_ADDR);
}

/** Recepcion de n bytes una vez enviado el START repetido + direccion de lectura. */
static i2c_res_t recibir(I2C_TypeDef *i2c, uint8_t *datos, size_t n)
{
	i2c_res_t r;

	if (n == 1U) {
		/* NACK programado antes de limpiar ADDR; STOP apenas se limpia. Las interrupciones
		 * se cortan esos pocos ciclos para que el STOP salga antes del fin del byte. */
		i2c->CR1 &= ~I2C_CR1_ACK;
		__disable_irq();
		limpiar_addr(i2c);
		i2c->CR1 |= I2C_CR1_STOP;
		__enable_irq();
		r = esperar(i2c, I2C_SR1_RXNE);
		if (r != I2C_OK) {
			return r;
		}
		datos[0] = (uint8_t)i2c->DR;
	} else if (n == 2U) {
		/* POS hace que el NACK se aplique al segundo byte. */
		__disable_irq();
		limpiar_addr(i2c);
		i2c->CR1 &= ~I2C_CR1_ACK;
		__enable_irq();
		r = esperar(i2c, I2C_SR1_BTF);
		if (r != I2C_OK) {
			return r;
		}
		__disable_irq();
		i2c->CR1 |= I2C_CR1_STOP;
		datos[0] = (uint8_t)i2c->DR;
		__enable_irq();
		datos[1] = (uint8_t)i2c->DR;
	} else {
		size_t i = 0U;

		limpiar_addr(i2c);
		while (n - i > 3U) {
			r = esperar(i2c, I2C_SR1_RXNE);
			if (r != I2C_OK) {
				return r;
			}
			datos[i++] = (uint8_t)i2c->DR;
		}
		/* Ultimos tres: byte N-2 en DR y N-1 en el registro de desplazamiento. */
		r = esperar(i2c, I2C_SR1_BTF);
		if (r != I2C_OK) {
			return r;
		}
		i2c->CR1 &= ~I2C_CR1_ACK;
		datos[i++] = (uint8_t)i2c->DR;
		r = esperar(i2c, I2C_SR1_BTF);
		if (r != I2C_OK) {
			return r;
		}
		__disable_irq();
		i2c->CR1 |= I2C_CR1_STOP;
		datos[i++] = (uint8_t)i2c->DR;
		__enable_irq();
		datos[i] = (uint8_t)i2c->DR;
	}
	return I2C_OK;
}

/** Verifica que el bus este libre antes de empezar; si no, intenta destrabarlo. */
static i2c_res_t asegurar_libre(const bus_cfg_t *b)
{
	if ((b->i2c->SR2 & I2C_SR2_BUSY) != 0U) {
		recuperar_bus(b);
		if ((b->i2c->SR2 & I2C_SR2_BUSY) != 0U) {
			return I2C_ERR_BUS;
		}
	}
	return I2C_OK;
}

i2c_res_t I2C_Init(i2c_bus_t bus)
{
	if (bus >= I2C_CANT_BUSES) {
		return I2C_ERR_PARAM;
	}
	const bus_cfg_t *b = &buses[bus];

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->APB1ENR |= b->rcc_apb1;
	(void)RCC->APB1ENR;		/* el periferico tarda un ciclo en quedar habilitado */

	configurar_pin(b->pin_scl, b->af_scl);
	configurar_pin(b->pin_sda, b->af_sda);
	configurar_periferico(b->i2c);

	return asegurar_libre(b);
}

i2c_res_t I2C_LeerRegistros(i2c_bus_t bus, uint8_t dir7, uint8_t reg, uint8_t *datos, size_t n)
{
	if (bus >= I2C_CANT_BUSES || datos == NULL || n == 0U) {
		return I2C_ERR_PARAM;
	}
	const bus_cfg_t *b = &buses[bus];
	I2C_TypeDef *i2c = b->i2c;

	i2c_res_t r = asegurar_libre(b);
	if (r != I2C_OK) {
		return r;
	}

	/* Fase de escritura: direccion + numero de registro. */
	i2c->CR1 &= ~I2C_CR1_POS;
	r = iniciar(i2c, dir7, false);
	if (r != I2C_OK) {
		return abortar(b, r);
	}
	limpiar_addr(i2c);
	r = esperar(i2c, I2C_SR1_TXE);
	if (r != I2C_OK) {
		return abortar(b, r);
	}
	i2c->DR = reg;
	r = esperar(i2c, I2C_SR1_BTF);
	if (r != I2C_OK) {
		return abortar(b, r);
	}

	/* Fase de lectura: START repetido. ACK (y POS si son dos bytes) se programan antes
	 * de que llegue ADDR. */
	i2c->CR1 |= I2C_CR1_ACK;
	if (n == 2U) {
		i2c->CR1 |= I2C_CR1_POS;
	}
	r = iniciar(i2c, dir7, true);
	if (r == I2C_OK) {
		r = recibir(i2c, datos, n);
	}
	if (r != I2C_OK) {
		return abortar(b, r);
	}

	i2c->CR1 &= ~(I2C_CR1_POS | I2C_CR1_ACK);
	r = esperar_stop(i2c);
	return (r == I2C_OK) ? I2C_OK : abortar(b, r);
}

i2c_res_t I2C_Sondear(i2c_bus_t bus, uint8_t dir7)
{
	if (bus >= I2C_CANT_BUSES) {
		return I2C_ERR_PARAM;
	}
	const bus_cfg_t *b = &buses[bus];
	I2C_TypeDef *i2c = b->i2c;

	i2c_res_t r = asegurar_libre(b);
	if (r != I2C_OK) {
		return r;
	}

	r = iniciar(i2c, dir7, false);
	if (r != I2C_OK) {
		return abortar(b, r);
	}
	limpiar_addr(i2c);
	i2c->CR1 |= I2C_CR1_STOP;
	r = esperar_stop(i2c);
	return (r == I2C_OK) ? I2C_OK : abortar(b, r);
}
