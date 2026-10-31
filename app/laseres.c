/**
 * @file laseres.c
 * @brief Lecturas de los dos sensores de la garra, de a uno por vez.
 */
#include "laseres.h"

#include <stddef.h>
#include <stdint.h>

#include "config.h"

static bool pedido;
static bool midiendo;
static bool descartar;
static vl53_id_t turno;

void Laseres_Init(void)
{
	pedido = false;
	midiendo = false;
	descartar = false;
	turno = VL53_A;
}

int Laseres_Listos(void)
{
	int n = 0;

	for (int i = 0; i < VL53_CANT; i++) {
		n += VL53L0X_Listo((vl53_id_t)i) ? 1 : 0;
	}
	return n;
}

/** Pasa el turno al proximo sensor listo, que puede ser el mismo si hay uno solo. */
static void pasar_turno(void)
{
	for (int i = 0; i < VL53_CANT; i++) {
		turno = (vl53_id_t)(((int)turno + 1) % VL53_CANT);
		if (VL53L0X_Listo(turno)) {
			return;
		}
	}
}

static bool medir_turno(void)
{
	midiendo = VL53L0X_IniciarMedicion(turno);
	return midiendo;
}

bool Laseres_Arrancar(void)
{
	if (Laseres_Listos() == 0) {
		return false;
	}
	pedido = true;
	if (midiendo) {
		descartar = true;
		return true;
	}
	/* Siempre desde el primero: asi el orden de las lecturas no depende del pedido anterior. */
	turno = VL53_B;
	pasar_turno();
	if (!medir_turno()) {
		pedido = false;
		return false;
	}
	return true;
}

void Laseres_Detener(void)
{
	pedido = false;
}

laser_res_t Laseres_Tick(laser_lectura_t *l)
{
	static const float offset[VL53_CANT] = { SENSOR_A_OFFSET_MM, SENSOR_B_OFFSET_MM };
	uint16_t mm = 0U;

	if (l == NULL || !midiendo) {
		return LASER_NADA;
	}

	vl53_res_t r = VL53L0X_Consultar(turno, &mm);

	if (r == VL53_EN_CURSO) {
		return LASER_NADA;
	}
	midiendo = false;
	if (r == VL53_ERROR) {
		descartar = false;
		return pedido ? LASER_FALLA : LASER_NADA;
	}

	bool sirve = !descartar && pedido;

	descartar = false;
	if (sirve) {
		l->sensor = turno;
		l->con_retorno = (r == VL53_LISTO);
		l->mm = (float)mm - offset[turno];
	}
	if (pedido) {
		pasar_turno();
		if (!medir_turno()) {
			return LASER_FALLA;
		}
	}
	return sirve ? LASER_LECTURA : LASER_NADA;
}
