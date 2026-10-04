/**
 * @file as5600.c
 * @brief Lectura del AS5600 por I2C.
 */
#include "as5600.h"

#include <stddef.h>

#define REG_STATUS		0x0BU
#define REG_ANGLE		0x0EU	/* 0x0E: bits 11..8, 0x0F: bits 7..0 */

#define STATUS_MH		0x08U	/* iman demasiado fuerte */
#define STATUS_ML		0x10U	/* iman demasiado debil */
#define STATUS_MD		0x20U	/* iman detectado */

#define GRADOS_POR_CUENTA	(360.0f / (float)AS5600_CUENTAS_VUELTA)

bool AS5600_Init(i2c_bus_t bus)
{
	return I2C_Init(bus) == I2C_OK && I2C_Sondear(bus, AS5600_DIRECCION) == I2C_OK;
}

bool AS5600_LeerCuentas(i2c_bus_t bus, uint16_t *cuentas)
{
	uint8_t datos[2];

	if (cuentas == NULL
			|| I2C_LeerRegistros(bus, AS5600_DIRECCION, REG_ANGLE, datos, sizeof(datos)) != I2C_OK) {
		return false;
	}
	/* Los 4 bits altos del primer byte no se usan. */
	*cuentas = (uint16_t)(((uint16_t)(datos[0] & 0x0FU) << 8) | datos[1]);
	return true;
}

bool AS5600_LeerAngulo(i2c_bus_t bus, float *grados)
{
	uint16_t cuentas;

	if (grados == NULL || !AS5600_LeerCuentas(bus, &cuentas)) {
		return false;
	}
	*grados = (float)cuentas * GRADOS_POR_CUENTA;
	return true;
}

as5600_iman_t AS5600_EstadoIman(i2c_bus_t bus)
{
	uint8_t status;

	if (I2C_LeerRegistros(bus, AS5600_DIRECCION, REG_STATUS, &status, 1U) != I2C_OK) {
		return AS5600_SIN_RESPUESTA;
	}
	if ((status & STATUS_MD) == 0U) {
		return AS5600_IMAN_AUSENTE;
	}
	if ((status & STATUS_ML) != 0U) {
		return AS5600_IMAN_DEBIL;
	}
	if ((status & STATUS_MH) != 0U) {
		return AS5600_IMAN_FUERTE;
	}
	return AS5600_IMAN_OK;
}

bool AS5600_ImanOK(i2c_bus_t bus)
{
	return AS5600_EstadoIman(bus) == AS5600_IMAN_OK;
}
