/**
 * @file metricas.c
 * @brief Estadisticas de precision y repetibilidad, y ajuste de una recta (nucleo portable).
 */
#include "metricas.h"

#include <math.h>
#include <stddef.h>

void Metricas_Iniciar(metricas_t *m)
{
	if (m != NULL) {
		*m = (metricas_t){ .n = 0U };
	}
}

/** Paso de Welford: actualiza media y suma de cuadrados con una muestra nueva. */
static void acumular(float x, uint32_t n, float *media, float *m2)
{
	float delta = x - *media;

	*media += delta / (float)n;
	*m2 += delta * (x - *media);
}

float Metricas_Agregar(metricas_t *m, const float cmd[METRICAS_EJES], const float med[METRICAS_EJES])
{
	if (m == NULL || cmd == NULL || med == NULL) {
		return 0.0f;
	}

	float suma = 0.0f;

	m->n++;
	for (int i = 0; i < METRICAS_EJES; i++) {
		float d = med[i] - cmd[i];

		suma += d * d;
		acumular(med[i], m->n, &m->med_media[i], &m->med_m2[i]);
		m->delta_media[i] += (d - m->delta_media[i]) / (float)m->n;
	}

	float e = sqrtf(suma);

	acumular(e, m->n, &m->e_media, &m->e_m2);
	if (e > m->e_max) {
		m->e_max = e;
	}
	return e;
}

void Metricas_Resultado(const metricas_t *m, metricas_res_t *r)
{
	if (m == NULL || r == NULL) {
		return;
	}

	float varianza = 0.0f;
	float exact2 = 0.0f;

	r->n = m->n;
	r->e_media = m->e_media;
	r->e_max = m->e_max;
	r->e_desvio = (m->n > 1U) ? sqrtf(m->e_m2 / (float)(m->n - 1U)) : 0.0f;
	for (int i = 0; i < METRICAS_EJES; i++) {
		exact2 += m->delta_media[i] * m->delta_media[i];
		if (m->n > 1U) {
			varianza += m->med_m2[i] / (float)(m->n - 1U);
		}
	}
	r->exactitud = sqrtf(exact2);
	r->repetibilidad = 3.0f * sqrtf(varianza);
}

void Recta_Iniciar(recta_t *r)
{
	if (r != NULL) {
		*r = (recta_t){ .n = 0U };
	}
}

void Recta_Agregar(recta_t *r, float x, float y)
{
	if (r != NULL) {
		r->n++;
		r->sx += x;
		r->sy += y;
		r->sxx += x * x;
		r->sxy += x * y;
	}
}

bool Recta_Ajustar(const recta_t *r, float *pendiente, float *ordenada)
{
	if (r == NULL || pendiente == NULL || ordenada == NULL || r->n < 2U) {
		return false;
	}

	float n = (float)r->n;
	float denom = n * r->sxx - r->sx * r->sx;

	if (!(fabsf(denom) > 0.0f)) {
		return false;
	}
	*pendiente = (n * r->sxy - r->sx * r->sy) / denom;
	*ordenada = (r->sy - *pendiente * r->sx) / n;
	return true;
}
