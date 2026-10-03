/**
 * @file test_cinematica.c
 * @brief Tests en PC de la cinematica directa/inversa (criterios de E1).
 *
 * Los puntos de prueba suponen la mecanica de config.h al 2026-10-03: L1 = L2 = 200 mm,
 * theta1 en [-90, 90], theta2 en [0, 152] (codo recto en 180 de servo, solo rama +),
 * anillo operativo 100..395 mm. Si cambia config.h hay que revisarlos.
 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "cinematica.h"
#include "config.h"

#define PUNTOS_IDA_VUELTA	10000
#define TOL_IDA_VUELTA_MM	1.0e-3f

/* Generador congruente lineal: los tests tienen que dar lo mismo en cada corrida. */
static uint32_t semilla = 12345U;

static float aleatorio(float min, float max)
{
	semilla = semilla * 1664525U + 1013904223U;
	return min + (max - min) * ((float)(semilla >> 8) / 16777216.0f);
}

static bool cerca(float a, float b, float tol)
{
	return fabsf(a - b) <= tol;
}

/** Busca en sol[] la solucion de una rama. */
static const ik_sol_t *de_rama(const ik_sol_t *sol, int n, int rama)
{
	for (int i = 0; i < n; i++) {
		if (sol[i].rama == rama) {
			return &sol[i];
		}
	}
	return NULL;
}

static void test_directa(void)
{
	float x, y;

	assert(Cinematica_Directa(0.0f, 0.0f, &x, &y));
	assert(cerca(x, 400.0f, 1e-3f) && cerca(y, 0.0f, 1e-3f));

	assert(Cinematica_Directa(90.0f, 0.0f, &x, &y));
	assert(cerca(x, 0.0f, 1e-3f) && cerca(y, 400.0f, 1e-3f));

	assert(Cinematica_Directa(0.0f, 90.0f, &x, &y));
	assert(cerca(x, 200.0f, 1e-3f) && cerca(y, 200.0f, 1e-3f));

	assert(Cinematica_Directa(-90.0f, 90.0f, &x, &y));
	assert(cerca(x, 200.0f, 1e-3f) && cerca(y, -200.0f, 1e-3f));

	assert(!Cinematica_Directa(0.0f, 0.0f, NULL, &y));
	printf("OK  directa\n");
}

/** Puntos de prueba del SPEC 3.2 (rama con codo "+"). */
static void test_puntos_spec(void)
{
	static const struct { float x, y, th1, th2; } pts[] = {
		{ 300.0f,    0.0f, -41.4f, 82.8f },
		{ 300.0f,  100.0f, -19.3f, 75.5f },
		{ 350.0f,  100.0f,  -8.5f, 49.0f },
		{ 380.0f,    0.0f, -18.2f, 36.4f },
		{ 300.0f, -100.0f, -56.2f, 75.5f },
	};

	for (unsigned i = 0; i < sizeof(pts) / sizeof(pts[0]); i++) {
		ik_sol_t sol[2];
		int n = Cinematica_Inversa(pts[i].x, pts[i].y, sol);
		const ik_sol_t *s = de_rama(sol, n, CIN_RAMA_POS);

		assert(s != NULL);
		assert(cerca(s->th1_deg, pts[i].th1, 0.06f));
		assert(cerca(s->th2_deg, pts[i].th2, 0.06f));
		assert(Cinematica_PuntoAlcanzable(pts[i].x, pts[i].y));
	}
	printf("OK  puntos del SPEC 3.2\n");
}

/** FK(IK(p)) = p con error < 1e-3 mm en 10 000 puntos del anillo operativo. */
static void test_ida_vuelta_cartesiana(void)
{
	int validos = 0;
	int soluciones = 0;
	float err_max = 0.0f;

	while (validos < PUNTOS_IDA_VUELTA) {
		float r = aleatorio(R_MIN_OP_MM, R_MAX_OP_MM);
		float ang = aleatorio(-3.14159265f, 3.14159265f);
		float x = r * cosf(ang);
		float y = r * sinf(ang);
		ik_sol_t sol[2];
		int n = Cinematica_Inversa(x, y, sol);

		if (n == 0) {
			assert(!Cinematica_PuntoAlcanzable(x, y));
			continue;
		}
		assert(Cinematica_PuntoAlcanzable(x, y));
		validos++;

		for (int i = 0; i < n; i++) {
			float xr, yr;

			assert(Cinematica_JuntasEnLimites(sol[i].th1_deg, sol[i].th2_deg));
			assert(sol[i].rama == CIN_RAMA_POS ? sol[i].th2_deg >= 0.0f : sol[i].th2_deg <= 0.0f);
			assert(Cinematica_Directa(sol[i].th1_deg, sol[i].th2_deg, &xr, &yr));

			float err = hypotf(xr - x, yr - y);
			if (err > err_max) {
				err_max = err;
			}
			soluciones++;
		}
	}

	printf("OK  ida y vuelta cartesiana: %d puntos, %d soluciones, error max = %.2e mm\n",
		   validos, soluciones, (double)err_max);
	assert(err_max < TOL_IDA_VUELTA_MM);
}

/** IK(FK(q)) = q recorriendo todo el rango articular (lejos de theta2 = 0). */
static void test_ida_vuelta_articular(void)
{
	int casos = 0;

	for (float th1 = JOINT1_MIN_DEG; th1 <= JOINT1_MAX_DEG; th1 += 2.5f) {
		for (float th2 = JOINT2_MIN_DEG; th2 <= JOINT2_MAX_DEG; th2 += 2.5f) {
			float x, y;
			ik_sol_t sol;

			if (fabsf(th2) < 2.0f || fabsf(th2) > 178.0f) {
				continue;	/* singularidad: las dos ramas coinciden y la IK esta mal condicionada */
			}
			assert(Cinematica_Directa(th1, th2, &x, &y));
			assert(Cinematica_InversaRama(x, y, th2 > 0.0f ? CIN_RAMA_POS : CIN_RAMA_NEG, &sol) == CIN_OK);
			assert(cerca(sol.th1_deg, th1, 0.01f));
			assert(cerca(sol.th2_deg, th2, 0.01f));
			casos++;
		}
	}
	assert(casos > 1000);
	printf("OK  ida y vuelta articular: %d posturas\n", casos);
}

static void test_fuera_de_alcance(void)
{
	ik_sol_t sol[2];
	ik_sol_t s;

	/* Mas lejos que L1 + L2. */
	assert(Cinematica_Inversa(500.0f, 0.0f, sol) == 0);
	assert(Cinematica_InversaRama(500.0f, 0.0f, CIN_RAMA_POS, &s) == CIN_ERR_RADIO);
	assert(Cinematica_InversaRama(0.0f, -401.0f, CIN_RAMA_NEG, &s) == CIN_ERR_RADIO);
	assert(!Cinematica_PuntoAlcanzable(500.0f, 0.0f));

	/* Dentro del anillo pero detras de la base: theta1 fuera de +-90. */
	assert(Cinematica_Inversa(-300.0f, 0.0f, sol) == 0);
	assert(Cinematica_InversaRama(-300.0f, 0.0f, CIN_RAMA_POS, &s) == CIN_ERR_LIMITE);
	assert(!Cinematica_PuntoAlcanzable(-300.0f, 0.0f));

	/* MOVE X200 Y150 del enunciado (r = 250 mm): con el codo rearmado ya es alcanzable. */
	assert(Cinematica_Inversa(200.0f, 150.0f, sol) == 1);
	assert(sol[0].rama == CIN_RAMA_POS);
	assert(cerca(sol[0].th1_deg, -14.45f, 0.06f) && cerca(sol[0].th2_deg, 102.64f, 0.06f));
	assert(Cinematica_PuntoAlcanzable(200.0f, 150.0f));

	/* La rama negativa no existe con esta mecanica (theta2 >= 0). */
	assert(Cinematica_InversaRama(300.0f, 0.0f, CIN_RAMA_NEG, &s) == CIN_ERR_LIMITE);

	/* Demasiado cerca de la base: el codo no pliega tanto (theta2 > 152). */
	assert(Cinematica_Inversa(50.0f, 50.0f, sol) == 0);
	assert(!Cinematica_PuntoAlcanzable(50.0f, 50.0f));

	/* Lado derecho: el punto existe, pero el hombro tendria que pasar de -90. */
	assert(Cinematica_Inversa(0.0f, -300.0f, sol) == 0);
	assert(!Cinematica_PuntoAlcanzable(0.0f, -300.0f));

	/* Geometricamente alcanzable pero fuera del anillo operativo. */
	assert(Cinematica_Inversa(398.0f, 0.0f, sol) == 1);
	assert(!Cinematica_PuntoAlcanzable(398.0f, 0.0f));
	assert(!Cinematica_PuntoAlcanzable(0.0f, 0.0f));

	/* Entradas no finitas: se rechazan, no se propagan. */
	assert(Cinematica_Inversa(NAN, 0.0f, sol) == 0);
	assert(!Cinematica_PuntoAlcanzable(NAN, 300.0f));
	assert(!Cinematica_PuntoAlcanzable(INFINITY, 0.0f));

	printf("OK  fuera de alcance\n");
}

/** Brazo estirado (HOME): c2 = 1 justo; no tiene que aparecer un NaN. */
static void test_singularidad(void)
{
	float x, y;
	ik_sol_t sol[2];

	assert(Cinematica_Directa(0.0f, 0.0f, &x, &y));
	int n = Cinematica_Inversa(x, y, sol);
	assert(n == 2);
	for (int i = 0; i < n; i++) {
		assert(!isnan(sol[i].th1_deg) && !isnan(sol[i].th2_deg));
		assert(cerca(sol[i].th1_deg, 0.0f, 0.05f));
		assert(cerca(sol[i].th2_deg, 0.0f, 0.05f));
	}

	/* Un pelo mas alla de L1 + L2 por redondeo se satura, no da NaN. */
	n = Cinematica_Inversa(400.001f, 0.0f, sol);
	assert(n == 2);
	assert(!isnan(sol[0].th1_deg) && !isnan(sol[0].th2_deg));
	printf("OK  singularidad de brazo estirado\n");
}

static void test_limites_y_normalizacion(void)
{
	assert(Cinematica_JuntasEnLimites(JOINT1_MIN_DEG, JOINT2_MAX_DEG));
	assert(Cinematica_JuntasEnLimites(JOINT1_MAX_DEG, JOINT2_MIN_DEG));
	assert(!Cinematica_JuntasEnLimites(JOINT1_MAX_DEG + 1.0f, 0.0f));
	assert(!Cinematica_JuntasEnLimites(0.0f, JOINT2_MIN_DEG - 1.0f));

	assert(cerca(Cinematica_NormalizarDeg(190.0f), -170.0f, 1e-4f));
	assert(cerca(Cinematica_NormalizarDeg(-190.0f), 170.0f, 1e-4f));
	assert(cerca(Cinematica_NormalizarDeg(180.0f), 180.0f, 1e-4f));
	assert(cerca(Cinematica_NormalizarDeg(-180.0f), 180.0f, 1e-4f));
	assert(cerca(Cinematica_NormalizarDeg(540.0f), 180.0f, 1e-4f));

	assert(Cinematica_ExcesoRadio(300.0f, 0.0f) == 0.0f);
	assert(cerca(Cinematica_ExcesoRadio(400.0f, 0.0f), 400.0f - R_MAX_OP_MM, 1e-3f));
	assert(cerca(Cinematica_ExcesoRadio(0.0f, R_MIN_OP_MM / 2.0f), R_MIN_OP_MM / 2.0f, 1e-3f));
	printf("OK  limites y normalizacion\n");
}

/** Lectura del AS5600 (0..360) -> angulo de junta, incluido el cruce 0/360. */
static void test_junta_desde_encoder(void)
{
	/* Cero en 100: 130 de encoder son +30 de junta; con signo -1, -30. */
	assert(cerca(Cinematica_JuntaDesdeEncoder(130.0f, 100.0f, 1.0f), 30.0f, 1e-4f));
	assert(cerca(Cinematica_JuntaDesdeEncoder(130.0f, 100.0f, -1.0f), -30.0f, 1e-4f));
	assert(cerca(Cinematica_JuntaDesdeEncoder(100.0f, 100.0f, 1.0f), 0.0f, 1e-4f));

	/* Cruce 0/360: cero en 350, el encoder pasa de 359,9 a 0,1 sin que la junta salte. */
	assert(cerca(Cinematica_JuntaDesdeEncoder(359.9f, 350.0f, 1.0f), 9.9f, 1e-3f));
	assert(cerca(Cinematica_JuntaDesdeEncoder(0.1f, 350.0f, 1.0f), 10.1f, 1e-3f));
	assert(cerca(Cinematica_JuntaDesdeEncoder(20.0f, 350.0f, 1.0f), 30.0f, 1e-3f));
	assert(cerca(Cinematica_JuntaDesdeEncoder(340.0f, 10.0f, 1.0f), -30.0f, 1e-3f));
	assert(cerca(Cinematica_JuntaDesdeEncoder(340.0f, 10.0f, -1.0f), 30.0f, 1e-3f));

	/* Siempre en (-180, 180]. */
	for (float enc = 0.0f; enc < 360.0f; enc += 7.3f) {
		float th = Cinematica_JuntaDesdeEncoder(enc, 123.0f, -1.0f);
		assert(th > -180.0f && th <= 180.0f);
	}
	printf("OK  junta desde encoder (cruce 0/360)\n");
}

int main(void)
{
	printf("== test_cinematica ==\n");
	test_directa();
	test_puntos_spec();
	test_ida_vuelta_cartesiana();
	test_ida_vuelta_articular();
	test_fuera_de_alcance();
	test_singularidad();
	test_limites_y_normalizacion();
	test_junta_desde_encoder();
	printf("test_cinematica: TODO OK\n");
	return 0;
}
