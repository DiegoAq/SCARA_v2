/**
 * @file vl53l0x.c
 * @brief Driver del VL53L0X sobre i2c.c (CMSIS, sin HAL).
 *
 * ST no documenta estos registros: solo publica la API en C. Los numeros de esta
 * inicializacion salen de esa API (VL53L0X_DataInit, VL53L0X_StaticInit y
 * VL53L0X_PerformRefCalibration) y de la tabla DefaultTuningSettings de vl53l0x_tuning.h.
 * No se recalcula el timing budget al sacar MSRC y TCC: la medicion queda en unos 30 ms,
 * que para verificar un objeto antes de tomarlo alcanza.
 */
#include "vl53l0x.h"

#include <stddef.h>

#include "config.h"
#include "i2c.h"
#include "stm32f4xx.h"
#include "systick.h"

#define BUS						I2C_BUS_1
#define DIRECCION_FABRICA		0x29U
#define DIRECCION_A				0x2AU
#define PIN_XSHUT_A				8U		/* PA8 */
#define PIN_XSHUT_B				7U		/* PB7 */

#define REG_SYSRANGE_START		0x00U
#define REG_SEQUENCE_CONFIG		0x01U
#define REG_GPIO_CONFIG			0x0AU
#define REG_INTERRUPT_CLEAR		0x0BU
#define REG_INTERRUPT_STATUS	0x13U
#define REG_RANGE_MM			0x1EU	/* RESULT_RANGE_STATUS + 10 */
#define REG_RATE_LIMIT			0x44U
#define REG_DYN_SPAD_START		0x4FU
#define REG_DYN_SPAD_REQUESTED	0x4EU
#define REG_MSRC_CONFIG			0x60U
#define REG_GPIO_ACTIVO_ALTO	0x84U
#define REG_VHV_PAD				0x89U
#define REG_DIRECCION_I2C		0x8AU
#define REG_SPAD_ENABLES		0xB0U
#define REG_SPAD_START_SELECT	0xB6U
#define REG_MODEL_ID			0xC0U
#define REG_PAGINA				0xFFU

#define MODEL_ID				0xEEU
#define STATUS_LISTO_MASK		0x07U
#define START_MEDIR				0x01U
#define SEQ_VHV					0x01U
#define SEQ_FASE				0x02U
#define SEQ_NORMAL				0xE8U	/* sin MSRC ni TCC, como queda en la API de ST */
#define VHV_INICIO				0x40U
#define TASA_MIN_MCPS_Q9_7		32U		/* 0,25 MCPS en punto fijo 9.7 */
#define SPADS_POR_MAPA			48U
#define PRIMER_SPAD_APERTURA	12U
#define BYTES_MAPA_SPAD			6U
/* Con estos valores el sensor avisa que no hubo retorno. */
#define RANGO_SIN_RETORNO_MM	8190U

/** Par registro-valor de las tablas de inicializacion. */
typedef struct {
	uint8_t reg;
	uint8_t valor;
} reg_valor_t;

/* Secuencia de ST para exponer o guardar la "stop variable". */
static const reg_valor_t abrir_pagina[] = {
	{ 0x80U, 0x01U }, { 0xFFU, 0x01U }, { 0x00U, 0x00U },
};
static const reg_valor_t cerrar_pagina[] = {
	{ 0x00U, 0x01U }, { 0xFFU, 0x00U }, { 0x80U, 0x00U },
};

/* DefaultTuningSettings de la API de ST. */
static const reg_valor_t ajuste[] = {
	{ 0xFFU, 0x01U }, { 0x00U, 0x00U }, { 0xFFU, 0x00U }, { 0x09U, 0x00U }, { 0x10U, 0x00U },
	{ 0x11U, 0x00U }, { 0x24U, 0x01U }, { 0x25U, 0xFFU }, { 0x75U, 0x00U }, { 0xFFU, 0x01U },
	{ 0x4EU, 0x2CU }, { 0x48U, 0x00U }, { 0x30U, 0x20U }, { 0xFFU, 0x00U }, { 0x30U, 0x09U },
	{ 0x54U, 0x00U }, { 0x31U, 0x04U }, { 0x32U, 0x03U }, { 0x40U, 0x83U }, { 0x46U, 0x25U },
	{ 0x60U, 0x00U }, { 0x27U, 0x00U }, { 0x50U, 0x06U }, { 0x51U, 0x00U }, { 0x52U, 0x96U },
	{ 0x56U, 0x08U }, { 0x57U, 0x30U }, { 0x61U, 0x00U }, { 0x62U, 0x00U }, { 0x64U, 0x00U },
	{ 0x65U, 0x00U }, { 0x66U, 0xA0U }, { 0xFFU, 0x01U }, { 0x22U, 0x32U }, { 0x47U, 0x14U },
	{ 0x49U, 0xFFU }, { 0x4AU, 0x00U }, { 0xFFU, 0x00U }, { 0x7AU, 0x0AU }, { 0x7BU, 0x00U },
	{ 0x78U, 0x21U }, { 0xFFU, 0x01U }, { 0x23U, 0x34U }, { 0x42U, 0x00U }, { 0x44U, 0xFFU },
	{ 0x45U, 0x26U }, { 0x46U, 0x05U }, { 0x40U, 0x40U }, { 0x0EU, 0x06U }, { 0x20U, 0x1AU },
	{ 0x43U, 0x40U }, { 0xFFU, 0x00U }, { 0x34U, 0x03U }, { 0x35U, 0x44U }, { 0xFFU, 0x01U },
	{ 0x31U, 0x04U }, { 0x4BU, 0x09U }, { 0x4CU, 0x05U }, { 0x4DU, 0x04U }, { 0xFFU, 0x00U },
	{ 0x44U, 0x00U }, { 0x45U, 0x20U }, { 0x47U, 0x08U }, { 0x48U, 0x28U }, { 0x67U, 0x00U },
	{ 0x70U, 0x04U }, { 0x71U, 0x01U }, { 0x72U, 0xFEU }, { 0x76U, 0x00U }, { 0x77U, 0x00U },
	{ 0xFFU, 0x01U }, { 0x0DU, 0x01U }, { 0xFFU, 0x00U }, { 0x80U, 0x01U }, { 0x01U, 0xF8U },
	{ 0xFFU, 0x01U }, { 0x8EU, 0x01U }, { 0x00U, 0x01U }, { 0xFFU, 0x00U }, { 0x80U, 0x00U },
};

#define CANT(tabla)		(sizeof(tabla) / sizeof((tabla)[0]))

/** Estado de un sensor. */
typedef struct {
	uint8_t direccion;
	bool listo;
	bool midiendo;
	uint8_t stop_variable;
	uint32_t t_inicio_ms;
} sensor_t;

static sensor_t sensores[VL53_CANT];

/* Las funciones de abajo hablan siempre con este sensor: asi la secuencia de inicializacion no
 * lleva el mismo argumento en cada una de sus muchas escrituras. */
static sensor_t *actual;

static bool escribir(uint8_t reg, uint8_t valor)
{
	return I2C_EscribirRegistros(BUS, actual->direccion, reg, &valor, 1U) == I2C_OK;
}

static bool leer(uint8_t reg, uint8_t *valor)
{
	return I2C_LeerRegistros(BUS, actual->direccion, reg, valor, 1U) == I2C_OK;
}

static bool escribir_tabla(const reg_valor_t *tabla, size_t n)
{
	for (size_t i = 0U; i < n; i++) {
		if (!escribir(tabla[i].reg, tabla[i].valor)) {
			return false;
		}
	}
	return true;
}

/** Espera solo para la inicializacion, unica etapa del firmware donde se permite bloquear. */
static void pausa_ms(uint32_t ms)
{
	uint32_t t0 = Millis();

	while ((uint32_t)(Millis() - t0) < ms) {
	}
}

/** Deja los dos sensores en reset. Sin esto, tras un reset del micro quedan en su estado anterior. */
static void apagar_sensores(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
	(void)RCC->AHB1ENR;		/* el periferico tarda un ciclo en quedar habilitado */

	GPIOA->BSRR = 1UL << (PIN_XSHUT_A + 16U);
	GPIOB->BSRR = 1UL << (PIN_XSHUT_B + 16U);
	GPIOA->MODER = (GPIOA->MODER & ~(3UL << (PIN_XSHUT_A * 2U))) | (1UL << (PIN_XSHUT_A * 2U));
	GPIOB->MODER = (GPIOB->MODER & ~(3UL << (PIN_XSHUT_B * 2U))) | (1UL << (PIN_XSHUT_B * 2U));
	pausa_ms(VL53_RESET_MS);
}

/** Saca un sensor de reset y le da tiempo a arrancar. */
static void despertar(vl53_id_t id)
{
	if (id == VL53_A) {
		GPIOA->BSRR = 1UL << PIN_XSHUT_A;
	} else {
		GPIOB->BSRR = 1UL << PIN_XSHUT_B;
	}
	pausa_ms(VL53_RESET_MS);
}

/** VL53L0X_DataInit: modo I2C, stop variable, limites de senal. */
static bool iniciar_datos(void)
{
	uint8_t v;
	const uint8_t limite[2] = { 0U, TASA_MIN_MCPS_Q9_7 };

	/* El sensor arranca con I/O a 1,8 V; el modulo trabaja a 2,8 V. */
	if (!leer(REG_VHV_PAD, &v) || !escribir(REG_VHV_PAD, (uint8_t)(v | 0x01U))) {
		return false;
	}
	if (!escribir(0x88U, 0x00U)
			|| !escribir_tabla(abrir_pagina, CANT(abrir_pagina))
			|| !leer(0x91U, &actual->stop_variable)
			|| !escribir_tabla(cerrar_pagina, CANT(cerrar_pagina))) {
		return false;
	}
	/* Deshabilita las verificaciones de senal de MSRC y de pre-rango. */
	if (!leer(REG_MSRC_CONFIG, &v) || !escribir(REG_MSRC_CONFIG, (uint8_t)(v | 0x12U))) {
		return false;
	}
	return I2C_EscribirRegistros(BUS, actual->direccion, REG_RATE_LIMIT, limite, sizeof(limite)) == I2C_OK
		&& escribir(REG_SEQUENCE_CONFIG, 0xFFU);
}

/** Lee del sensor cuantos SPAD de referencia usar y de que tipo. */
static bool leer_info_spad(uint8_t *cantidad, bool *es_apertura)
{
	uint8_t v;

	if (!escribir_tabla(abrir_pagina, CANT(abrir_pagina))
			|| !escribir(REG_PAGINA, 0x06U)
			|| !leer(0x83U, &v) || !escribir(0x83U, (uint8_t)(v | 0x04U))
			|| !escribir(REG_PAGINA, 0x07U) || !escribir(0x81U, 0x01U)
			|| !escribir(0x80U, 0x01U) || !escribir(0x94U, 0x6BU) || !escribir(0x83U, 0x00U)) {
		return false;
	}

	uint32_t t0 = Millis();
	do {
		if (!leer(0x83U, &v) || (uint32_t)(Millis() - t0) > VL53_TIMEOUT_MS) {
			return false;
		}
	} while (v == 0x00U);

	if (!escribir(0x83U, 0x01U) || !leer(0x92U, &v)) {
		return false;
	}
	*cantidad = (uint8_t)(v & 0x7FU);
	*es_apertura = (v & 0x80U) != 0U;

	return escribir(0x81U, 0x00U) && escribir(REG_PAGINA, 0x06U)
		&& leer(0x83U, &v) && escribir(0x83U, (uint8_t)(v & ~0x04U))
		&& escribir(REG_PAGINA, 0x01U) && escribir(0x00U, 0x01U)
		&& escribir(REG_PAGINA, 0x00U) && escribir(0x80U, 0x00U);
}

/** Deja habilitados en el sensor solo los SPAD de referencia que corresponden. */
static bool configurar_spad(void)
{
	uint8_t cantidad, mapa[BYTES_MAPA_SPAD];
	bool es_apertura;

	if (!leer_info_spad(&cantidad, &es_apertura)
			|| I2C_LeerRegistros(BUS, actual->direccion, REG_SPAD_ENABLES, mapa, sizeof(mapa)) != I2C_OK
			|| !escribir(REG_PAGINA, 0x01U)
			|| !escribir(REG_DYN_SPAD_START, 0x00U)
			|| !escribir(REG_DYN_SPAD_REQUESTED, 0x2CU)
			|| !escribir(REG_PAGINA, 0x00U)
			|| !escribir(REG_SPAD_START_SELECT, 0xB4U)) {
		return false;
	}

	uint8_t primero = es_apertura ? PRIMER_SPAD_APERTURA : 0U;
	uint8_t habilitados = 0U;

	for (uint8_t i = 0U; i < SPADS_POR_MAPA; i++) {
		uint8_t bit = (uint8_t)(1U << (i % 8U));

		if (i < primero || habilitados == cantidad) {
			mapa[i / 8U] = (uint8_t)(mapa[i / 8U] & ~bit);
		} else if ((mapa[i / 8U] & bit) != 0U) {
			habilitados++;
		}
	}
	return I2C_EscribirRegistros(BUS, actual->direccion, REG_SPAD_ENABLES, mapa, sizeof(mapa)) == I2C_OK;
}

/** Una etapa de VL53L0X_PerformRefCalibration. */
static bool calibrar_referencia(uint8_t secuencia, uint8_t inicio)
{
	uint8_t estado;

	if (!escribir(REG_SEQUENCE_CONFIG, secuencia)
			|| !escribir(REG_SYSRANGE_START, (uint8_t)(START_MEDIR | inicio))) {
		return false;
	}

	uint32_t t0 = Millis();
	do {
		if (!leer(REG_INTERRUPT_STATUS, &estado) || (uint32_t)(Millis() - t0) > VL53_TIMEOUT_MS) {
			return false;
		}
	} while ((estado & STATUS_LISTO_MASK) == 0U);

	return escribir(REG_INTERRUPT_CLEAR, 0x01U) && escribir(REG_SYSRANGE_START, 0x00U);
}

/** Inicializa el sensor actual, ya despierto y con su direccion definitiva. */
static bool iniciar_sensor(void)
{
	uint8_t v;

	if (!leer(REG_MODEL_ID, &v) || v != MODEL_ID) {
		return false;
	}
	if (!iniciar_datos() || !configurar_spad() || !escribir_tabla(ajuste, CANT(ajuste))) {
		return false;
	}

	/* Interrupcion interna al terminar cada medicion; es lo que consulta el firmware. */
	if (!escribir(REG_GPIO_CONFIG, 0x04U)
			|| !leer(REG_GPIO_ACTIVO_ALTO, &v)
			|| !escribir(REG_GPIO_ACTIVO_ALTO, (uint8_t)(v & ~0x10U))
			|| !escribir(REG_INTERRUPT_CLEAR, 0x01U)) {
		return false;
	}

	if (!calibrar_referencia(SEQ_VHV, VHV_INICIO) || !calibrar_referencia(SEQ_FASE, 0x00U)) {
		return false;
	}
	return escribir(REG_SEQUENCE_CONFIG, SEQ_NORMAL);
}

int VL53L0X_Init(void)
{
	int cantidad = 0;

	for (int i = 0; i < VL53_CANT; i++) {
		sensores[i] = (sensor_t){ .direccion = DIRECCION_FABRICA };
	}
	apagar_sensores();

	/* El A se mueve de direccion antes de despertar al B: los dos en 0x29 chocarian en el bus. */
	despertar(VL53_A);
	actual = &sensores[VL53_A];
	if (escribir(REG_DIRECCION_I2C, DIRECCION_A)) {
		actual->direccion = DIRECCION_A;
		actual->listo = iniciar_sensor();
		cantidad += actual->listo ? 1 : 0;
	}

	despertar(VL53_B);
	actual = &sensores[VL53_B];
	actual->listo = iniciar_sensor();
	cantidad += actual->listo ? 1 : 0;
	return cantidad;
}

bool VL53L0X_Listo(vl53_id_t id)
{
	return id < VL53_CANT && sensores[id].listo;
}

bool VL53L0X_IniciarMedicion(vl53_id_t id)
{
	if (!VL53L0X_Listo(id)) {
		return false;
	}
	actual = &sensores[id];
	if (!escribir_tabla(abrir_pagina, CANT(abrir_pagina))
			|| !escribir(0x91U, actual->stop_variable)
			|| !escribir_tabla(cerrar_pagina, CANT(cerrar_pagina))
			|| !escribir(REG_SYSRANGE_START, START_MEDIR)) {
		actual->midiendo = false;
		return false;
	}
	actual->midiendo = true;
	actual->t_inicio_ms = Millis();
	return true;
}

vl53_res_t VL53L0X_Consultar(vl53_id_t id, uint16_t *mm)
{
	uint8_t estado;
	uint8_t datos[2];

	if (id >= VL53_CANT || !sensores[id].midiendo || mm == NULL) {
		return VL53_ERROR;
	}
	actual = &sensores[id];
	if (!leer(REG_INTERRUPT_STATUS, &estado)) {
		actual->midiendo = false;
		return VL53_ERROR;
	}
	if ((estado & STATUS_LISTO_MASK) == 0U) {
		if ((uint32_t)(Millis() - actual->t_inicio_ms) > VL53_TIMEOUT_MS) {
			actual->midiendo = false;
			return VL53_ERROR;
		}
		return VL53_EN_CURSO;
	}

	actual->midiendo = false;
	if (I2C_LeerRegistros(BUS, actual->direccion, REG_RANGE_MM, datos, sizeof(datos)) != I2C_OK
			|| !escribir(REG_INTERRUPT_CLEAR, 0x01U)) {
		return VL53_ERROR;
	}

	uint16_t valor = (uint16_t)(((uint16_t)datos[0] << 8) | datos[1]);

	if (valor >= RANGO_SIN_RETORNO_MM) {
		return VL53_SIN_OBJETO;
	}
	*mm = valor;
	return VL53_LISTO;
}
