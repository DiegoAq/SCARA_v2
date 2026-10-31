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

float Cinematica_VueltaCentrada(float ang_deg, float centro_deg)
{
	return centro_deg + Cinematica_NormalizarDeg(ang_deg - centro_deg);
}

float Cinematica_ErrorPorJuntas(float th1_deg, float th2_deg, float d1_deg, float d2_deg)
{
	float th1 = th1_deg * DEG_A_RAD;
	float th12 = (th1_deg + th2_deg) * DEG_A_RAD;
	float d1 = fabsf(d1_deg) * DEG_A_RAD;
	float d2 = fabsf(d2_deg) * DEG_A_RAD;

	/* Jacobiano del SCARA, con cada termino en valor absoluto: peor caso de signos. */
	float ex = fabsf(L1_MM * sinf(th1) + L2_MM * sinf(th12)) * d1 + fabsf(L2_MM * sinf(th12)) * d2;
	float ey = fabsf(L1_MM * cosf(th1) + L2_MM * cosf(th12)) * d1 + fabsf(L2_MM * cosf(th12)) * d2;

	return sqrtf(ex * ex + ey * ey);
}

void Cinematica_PuntoCircunferencia(float cx_mm, float cy_mm, float r_mm, float ang_deg,
									float *x_mm, float *y_mm)
{
	if (x_mm != NULL && y_mm != NULL) {
		*x_mm = cx_mm + r_mm * cosf(ang_deg * DEG_A_RAD);
		*y_mm = cy_mm + r_mm * sinf(ang_deg * DEG_A_RAD);
	}
}

float Cinematica_Atan2Deg(float y, float x)
{
	return atan2f(y, x) * RAD_A_DEG;
}

void Cinematica_Direccion(float ang_deg, float *ux, float *uy)
{
	Cinematica_PuntoCircunferencia(0.0f, 0.0f, 1.0f, ang_deg, ux, uy);
}

float Cinematica_YawGarra(float th1_deg, float th2_deg, float rot_deg)
{
	return Cinematica_NormalizarDeg(th1_deg + th2_deg + rot_deg + GARRA_YAW_CERO_DEG);
}

void Cinematica_PuntoSensor(float th1_deg, float th2_deg, float rot_deg, float lado,
							float *x_mm, float *y_mm)
{
	float x, y, ux, uy;

	if (x_mm == NULL || y_mm == NULL) {
		return;
	}
	(void)Cinematica_Directa(th1_deg, th2_deg, &x, &y);
	Cinematica_Direccion(Cinematica_YawGarra(th1_deg, th2_deg, rot_deg), &ux, &uy);

	float a_lo_largo = lado * 0.5f * SENSOR_SEPARACION_MM;

	*x_mm = x + a_lo_largo * ux - SENSOR_DESVIO_MM * uy;
	*y_mm = y + a_lo_largo * uy + SENSOR_DESVIO_MM * ux;
}

bool Cinematica_RotParaYaw(float th1_deg, float th2_deg, float yaw_deg, float rot_actual_deg,
						   float *rot_deg)
{
	float base = Cinematica_NormalizarDeg(yaw_deg - th1_deg - th2_deg - GARRA_YAW_CERO_DEG);
	const float candidatos[3] = { base, base - 180.0f, base + 180.0f };
	bool hay = false;
	float mejor = 0.0f;

	if (rot_deg == NULL) {
		return false;
	}
	for (int i = 0; i < 3; i++) {
		float c = candidatos[i];

		if (c >= ROT_MIN_DEG && c <= ROT_MAX_DEG
				&& (!hay || fabsf(c - rot_actual_deg) < fabsf(mejor - rot_actual_deg))) {
			mejor = c;
			hay = true;
		}
	}
	if (hay) {
		*rot_deg = mejor;
	}
	return hay;
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

	/* Cada angulo se lleva a la vuelta centrada en el rango de su junta. Normalizar a
	 * (-180, 180] a secas falla si el rango llega justo a 180 (hombro en [0, 180]): por
	 * redondeo, un 180 sale como -179,999 y queda fuera de limites. */
	const float centro1 = 0.5f * (JOINT1_MIN_DEG + JOINT1_MAX_DEG);
	const float centro2 = 0.5f * (JOINT2_MIN_DEG + JOINT2_MAX_DEG);
	float th1_deg = centro1 + Cinematica_NormalizarDeg(th1 * RAD_A_DEG - centro1);
	float th2_deg = centro2 + Cinematica_NormalizarDeg(th2 * RAD_A_DEG - centro2);

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
