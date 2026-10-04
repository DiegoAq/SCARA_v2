/**
 * @file as5600.c
 * @brief Lectura del AS5600 por su salida analogica (ADC1).
 *
 * No incluye headers de registros: solo usa adc.h, asi que tambien se prueba en la PC
 * con el ADC simulado.
 */
#include "as5600.h"

#include <stddef.h>

#include "adc.h"
#include "config.h"

#define MEDIA_VUELTA		(AS5600_CUENTAS_VUELTA / 2U)
#define GRADOS_POR_CUENTA	(360.0f / (float)AS5600_CUENTAS_VUELTA)

/** Canal del ADC de cada sensor (cableado fijo). */
static const uint8_t canal[AS5600_CANT] = {
	[AS5600_HOMBRO] = ADC_CANAL_PB0,
	[AS5600_CODO]   = ADC_CANAL_PB1,
};

void AS5600_Init(void)
{
	ADC_Init();
}

bool AS5600_LeerCuentas(as5600_id_t sensor, uint16_t *cuentas, uint16_t *dispersion)
{
	uint16_t muestra[ENC_MUESTRAS];
	uint16_t min = AS5600_CUENTAS_VUELTA;
	uint16_t max = 0U;

	if (sensor >= AS5600_CANT || cuentas == NULL) {
		return false;
	}

	for (unsigned i = 0U; i < ENC_MUESTRAS; i++) {
		if (!ADC_LeerCanal(canal[sensor], &muestra[i])) {
			return false;
		}
		if (muestra[i] < min) min = muestra[i];
		if (muestra[i] > max) max = muestra[i];
	}

	/* Si las muestras estan repartidas a los dos lados del cruce 0/360 (unas cerca de 0
	 * y otras cerca de 4095), promediarlas tal cual daria ~2048, media vuelta de error.
	 * Se "desenrollan" sumando una vuelta a las bajas antes de promediar. */
	bool en_el_cruce = (uint16_t)(max - min) > MEDIA_VUELTA;
	uint32_t suma = 0U;
	uint32_t min_d = UINT32_MAX;
	uint32_t max_d = 0U;

	for (unsigned i = 0U; i < ENC_MUESTRAS; i++) {
		uint32_t v = muestra[i];

		if (en_el_cruce && v < MEDIA_VUELTA) {
			v += AS5600_CUENTAS_VUELTA;
		}
		suma += v;
		if (v < min_d) min_d = v;
		if (v > max_d) max_d = v;
	}

	*cuentas = (uint16_t)(((suma + ENC_MUESTRAS / 2U) / ENC_MUESTRAS) % AS5600_CUENTAS_VUELTA);
	if (dispersion != NULL) {
		*dispersion = (uint16_t)(max_d - min_d);
	}
	return true;
}

bool AS5600_LeerAngulo(as5600_id_t sensor, float *grados)
{
	uint16_t cuentas;

	if (grados == NULL || !AS5600_LeerCuentas(sensor, &cuentas, NULL)) {
		return false;
	}
	*grados = (float)cuentas * GRADOS_POR_CUENTA;
	return true;
}
