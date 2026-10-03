/**
 * @file cinematica.c
 * @brief Cinematica directa e inversa del SCARA (nucleo portable, sin hardware).
 *
 * Unico archivo del proyecto que trabaja en radianes. Todo en float: el Cortex-M4F
 * solo acelera simple precision.
 */
#include "cinematica.h"
#include "config.h"

#include <math.h>
#include <stddef.h>

#define DEG_A_RAD	0.017453292519943295f
#define RAD_A_DEG	57.29577951308232f

/**
 * @brief Verifica un limite articular con la holgura numerica JOINT_LIM_TOL_DEG.
 *
 * No satura el angulo: correrlo al tope moveria el efector (0,01 grados son 0,07 mm
 * en la punta). El exceso admitido es despreciable y el driver de servo satura el pulso.
 */
static bool dentro_de_limite(float ang_deg, float min_deg, float max_deg)
{
	return ang_deg >= min_deg - JOINT_LIM_TOL_DEG && ang_deg <= max_deg + JOINT_LIM_TOL_DEG;
}

float Cinematica_NormalizarDeg(float ang_deg)
{
	/* Los angulos que entran aca estan a lo sumo a una o dos vueltas: un par de
	 * restas es mas barato y mas exacto que fmodf. */
	while (ang_deg > 180.0f) {
		ang_deg -= 360.0f;
	}
	while (ang_deg <= -180.0f) {
		ang_deg += 360.0f;
	}
	return ang_deg;
}

float Cinematica_JuntaDesdeEncoder(float enc_deg, float cero_deg, float signo)
{
	/* Se normaliza otra vez despues del signo: con signo -1, un +180 pasaria a -180,
	 * que queda fuera de (-180, 180]. */
	return Cinematica_NormalizarDeg(signo * Cinematica_NormalizarDeg(enc_deg - cero_deg));
}

bool Cinematica_JuntasEnLimites(float th1_deg, float th2_deg)
{
	return dentro_de_limite(th1_deg, JOINT1_MIN_DEG, JOINT1_MAX_DEG)
		&& dentro_de_limite(th2_deg, JOINT2_MIN_DEG, JOINT2_MAX_DEG);
}

bool Cinematica_Directa(float th1_deg, float th2_deg, float *x_mm, float *y_mm)
{
	if (x_mm == NULL || y_mm == NULL) {
		return false;
	}

	float th1 = th1_deg * DEG_A_RAD;
	float th12 = (th1_deg + th2_deg) * DEG_A_RAD;

	*x_mm = L1_MM * cosf(th1) + L2_MM * cosf(th12);
	*y_mm = L1_MM * sinf(th1) + L2_MM * sinf(th12);
	return true;
}

cin_res_t Cinematica_InversaRama(float x_mm, float y_mm, int rama, ik_sol_t *sol)
{
	if (sol == NULL) {
		return CIN_ERR_RADIO;
	}

	float c2 = (x_mm * x_mm + y_mm * y_mm - L1_MM * L1_MM - L2_MM * L2_MM)
			 / (2.0f * L1_MM * L2_MM);

	/* Comparacion negada a proposito: tambien rechaza NaN. */
	if (!(fabsf(c2) <= 1.0f + IK_EPS_C2)) {
		return CIN_ERR_RADIO;
	}
	/* Sin esta saturacion, un c2 = 1.0000001 por redondeo da sqrtf(negativo) = NaN. */
	if (c2 > 1.0f) {
		c2 = 1.0f;
	} else if (c2 < -1.0f) {
		c2 = -1.0f;
	}

	float signo = (rama >= 0) ? 1.0f : -1.0f;
	float s2 = signo * sqrtf(1.0f - c2 * c2);
	float th2 = atan2f(s2, c2);
	float th1 = atan2f(y_mm, x_mm) - atan2f(L2_MM * s2, L1_MM + L2_MM * c2);

	float th1_deg = Cinematica_NormalizarDeg(th1 * RAD_A_DEG);
	float th2_deg = th2 * RAD_A_DEG;

	sol->rama = (rama >= 0) ? CIN_RAMA_POS : CIN_RAMA_NEG;

	sol->th1_deg = th1_deg;
	sol->th2_deg = th2_deg;

	return Cinematica_JuntasEnLimites(th1_deg, th2_deg) ? CIN_OK : CIN_ERR_LIMITE;
}

int Cinematica_Inversa(float x_mm, float y_mm, ik_sol_t sol[2])
{
	static const int ramas[2] = { CIN_RAMA_POS, CIN_RAMA_NEG };
	int n = 0;

	if (sol == NULL) {
		return 0;
	}

	for (int i = 0; i < 2; i++) {
		ik_sol_t s;
		if (Cinematica_InversaRama(x_mm, y_mm, ramas[i], &s) == CIN_OK) {
			sol[n] = s;
			n++;
		}
	}
	return n;
}

float Cinematica_ExcesoRadio(float x_mm, float y_mm)
{
	float r = sqrtf(x_mm * x_mm + y_mm * y_mm);

	if (r > R_MAX_OP_MM) {
		return r - R_MAX_OP_MM;
	}
	if (r < R_MIN_OP_MM) {
		return R_MIN_OP_MM - r;
	}
	/* Un NaN no cumple ninguna de las dos comparaciones: tratarlo como fuera. */
	if (r != r) {
		return R_MAX_OP_MM;
	}
	return 0.0f;
}

bool Cinematica_PuntoAlcanzable(float x_mm, float y_mm)
{
	ik_sol_t sol[2];

	if (Cinematica_ExcesoRadio(x_mm, y_mm) > 0.0f) {
		return false;
	}
	return Cinematica_Inversa(x_mm, y_mm, sol) > 0;
}
