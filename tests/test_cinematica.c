/**
 * @file test_cinematica.c
 * @brief Tests en PC de la cinematica directa/inversa (criterios de E1).
 *
 * Los puntos de prueba estan escritos en un marco LOCAL: +X local es la direccion en la
 * que apunta el brazo en HOME (HOME_TH1_DEG) y +Y local queda a su izquierda. Asi los
 * tests no cambian si se redefine hacia donde apunta el brazo; frente() los pasa al
 * marco del robot.
 *
 * Suponen: L1 = L2 = 200 mm, hombro +-90 alrededor de HOME, codo +-90 (recto en HOME),
 * anillo operativo 285..395 mm. Si cambia eso en config.h hay que revisarlos.
 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "cinematica.h"
#include "config.h"

#define PUNTOS_IDA_VUELTA	10000
#define TOL_IDA_VUELTA_MM	1.0e-3f
#define FRENTE_DEG			HOME_TH1_DEG
#define GRADOS_A_RAD		0.017453292519943295f

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

/** Pasa un punto del marco local (frente = +X local) al marco del robot. */
static void frente(float xl, float yl, float *x, float *y)
{
	float c = cosf(FRENTE_DEG * GRADOS_A_RAD);
	float s = sinf(FRENTE_DEG * GRADOS_A_RAD);

	*x = xl * c - yl * s;
	*y = xl * s + yl * c;
}

/** IK de una rama para un punto dado en el marco local. */
static cin_res_t ik_local(float xl, float yl, int rama, ik_sol_t *sol)
{
	float x, y;

	frente(xl, yl, &x, &y);
	return Cinematica_InversaRama(x, y, rama, sol);
}

static int cant_soluciones_local(float xl, float yl)
{
	float x, y;
	ik_sol_t sol[2];

	frente(xl, yl, &x, &y);
	return Cinematica_Inversa(x, y, sol);
}

static bool alcanzable_local(float xl, float yl)
{
	float x, y;

	frente(xl, yl, &x, &y);
	return Cinematica_PuntoAlcanzable(x, y);
}

/** FK(th1 local, th2) debe caer en el punto local (xl, yl). */
static void verificar_directa(float th1_local, float th2, float xl, float yl)
{
	float x, y, xe, ye;

	assert(Cinematica_Directa(FRENTE_DEG + th1_local, th2, &x, &y));
	frente(xl, yl, &xe, &ye);
	assert(cerca(x, xe, 1e-3f) && cerca(y, ye, 1e-3f));
}

static void test_directa(void)
{
	float x, y;

	verificar_directa(0.0f, 0.0f, 400.0f, 0.0f);
	verificar_directa(90.0f, 0.0f, 0.0f, 400.0f);
	verificar_directa(0.0f, 90.0f, 200.0f, 200.0f);
	verificar_directa(-90.0f, 90.0f, 200.0f, -200.0f);

	/* Dato del equipo: en HOME (los dos servos en 90) la pinza esta en X = 0, Y = 400. */
	assert(Cinematica_Directa(HOME_TH1_DEG, HOME_TH2_DEG, &x, &y));
	assert(cerca(x, 0.0f, 1e-3f) && cerca(y, 400.0f, 1e-3f));

	assert(!Cinematica_Directa(0.0f, 0.0f, NULL, &y));
	printf("OK  directa (HOME = X0 Y400)\n");
}

/** Puntos de prueba del SPEC 3.2 (rama con codo "+"), en el marco local. */
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
		ik_sol_t s;

		assert(ik_local(pts[i].x, pts[i].y, CIN_RAMA_POS, &s) == CIN_OK);
		assert(cerca(s.th1_deg - FRENTE_DEG, pts[i].th1, 0.06f));
		assert(cerca(s.th2_deg, pts[i].th2, 0.06f));
		assert(alcanzable_local(pts[i].x, pts[i].y));
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
	ik_sol_t s;
	ik_sol_t sol[2];

	/* Mas lejos que L1 + L2. */
	assert(cant_soluciones_local(500.0f, 0.0f) == 0);
	assert(ik_local(500.0f, 0.0f, CIN_RAMA_POS, &s) == CIN_ERR_RADIO);
	assert(ik_local(0.0f, -401.0f, CIN_RAMA_NEG, &s) == CIN_ERR_RADIO);
	assert(!alcanzable_local(500.0f, 0.0f));

	/* Dentro del anillo pero detras de la base: el hombro tendria que pasar de +-90. */
	assert(cant_soluciones_local(-300.0f, 0.0f) == 0);
	assert(ik_local(-300.0f, 0.0f, CIN_RAMA_POS, &s) == CIN_ERR_LIMITE);
	assert(!alcanzable_local(-300.0f, 0.0f));

	/* A 250 mm de la base: exige |theta2| > 90, mas de lo que pliega el codo. */
	assert(cant_soluciones_local(200.0f, 150.0f) == 0);
	assert(!alcanzable_local(200.0f, 150.0f));

	/* Geometricamente alcanzable pero fuera del anillo operativo. */
	assert(cant_soluciones_local(398.0f, 0.0f) == 2);
	assert(!alcanzable_local(398.0f, 0.0f));
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

	assert(Cinematica_Directa(HOME_TH1_DEG, HOME_TH2_DEG, &x, &y));
	int n = Cinematica_Inversa(x, y, sol);
	assert(n == 2);
	for (int i = 0; i < n; i++) {
		assert(!isnan(sol[i].th1_deg) && !isnan(sol[i].th2_deg));
		assert(cerca(sol[i].th1_deg, HOME_TH1_DEG, 0.05f));
		assert(cerca(sol[i].th2_deg, HOME_TH2_DEG, 0.05f));
	}

	/* Un pelo mas alla de L1 + L2 por redondeo se satura, no da NaN. */
	frente(400.001f, 0.0f, &x, &y);
	n = Cinematica_Inversa(x, y, sol);
	assert(n == 2);
	assert(!isnan(sol[0].th1_deg) && !isnan(sol[0].th2_deg));
	printf("OK  singularidad de brazo estirado\n");
}

static void test_limites_y_normalizacion(void)
{
	float x, y;

	assert(Cinematica_JuntasEnLimites(JOINT1_MIN_DEG, JOINT2_MAX_DEG));
	assert(Cinematica_JuntasEnLimites(JOINT1_MAX_DEG, JOINT2_MIN_DEG));
	assert(!Cinematica_JuntasEnLimites(JOINT1_MAX_DEG + 1.0f, 0.0f));
	assert(!Cinematica_JuntasEnLimites(0.0f, JOINT2_MIN_DEG - 1.0f));
	assert(Cinematica_JuntasEnLimites(HOME_TH1_DEG, HOME_TH2_DEG));

	assert(cerca(Cinematica_NormalizarDeg(190.0f), -170.0f, 1e-4f));
	assert(cerca(Cinematica_NormalizarDeg(-190.0f), 170.0f, 1e-4f));
	assert(cerca(Cinematica_NormalizarDeg(180.0f), 180.0f, 1e-4f));
	assert(cerca(Cinematica_NormalizarDeg(-180.0f), 180.0f, 1e-4f));
	assert(cerca(Cinematica_NormalizarDeg(540.0f), 180.0f, 1e-4f));

	frente(300.0f, 0.0f, &x, &y);
	assert(Cinematica_ExcesoRadio(x, y) == 0.0f);
	frente(400.0f, 0.0f, &x, &y);
	assert(cerca(Cinematica_ExcesoRadio(x, y), 400.0f - R_MAX_OP_MM, 1e-3f));
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
