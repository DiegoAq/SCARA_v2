/**
 * @file planificador.c
 * @brief Pre-vuelo y ejecucion de un movimiento lineal (nucleo portable, sin hardware).
 */
#include "planificador.h"
#include "cinematica.h"
#include "config.h"

#include <math.h>
#include <stddef.h>

/** Resultado de evaluar un avance candidato. */
typedef enum {
	EVAL_OK,		/**< Alcanzable y dentro de los limites de velocidad y aceleracion articular */
	EVAL_RAPIDO,	/**< Alcanzable, pero alguna junta superaria su tope de este tick */
	EVAL_ERROR		/**< La IK fallo: no deberia pasar despues del pre-vuelo */
} eval_t;

/**
 * @brief Velocidad maxima que puede tener una junta en este tick.
 *
 * Dos limites: el tope absoluto JOINT_VMAX_DPS y la aceleracion, o sea que no puede ir mas
 * de JOINT_ACCEL_DPS2 * dt por encima de su velocidad del tick anterior. Sin el segundo, al
 * salir de la singularidad (brazo estirado) el codo pasaba de 0 al tope en un solo tick.
 *
 * El piso JOINT_W_ARRANQUE_DPS existe porque justo en la singularidad el angulo crece como
 * la raiz del avance: el paso mas chico representable en float ya exige ~2 grados/s, y sin
 * piso el brazo no arrancaria nunca.
 *
 * @param w_prev_dps Velocidad (modulo) de la junta en el tick anterior.
 */
static float tope_velocidad(float w_prev_dps, float dt_s)
{
	float tope = w_prev_dps + JOINT_ACCEL_DPS2 * dt_s;

	if (tope < JOINT_W_ARRANQUE_DPS) {
		tope = JOINT_W_ARRANQUE_DPS;
	}
	return (tope < JOINT_VMAX_DPS) ? tope : JOINT_VMAX_DPS;
}

/**
 * @brief Calcula la postura para una distancia recorrida y la velocidad articular que exige.
 * @param s_mm Distancia sobre la recta.
 * @param es_destino true si s_mm es el final del segmento: se usa el destino exacto
 *                   en vez de P0 + u*s para no arrastrar el redondeo de la suma.
 */
static eval_t evaluar(const plan_t *p, float s_mm, bool es_destino, float dt_s, ik_sol_t *sol)
{
	float x = es_destino ? p->xf_mm : p->x0_mm + p->ux * s_mm;
	float y = es_destino ? p->yf_mm : p->y0_mm + p->uy * s_mm;

	if (Cinematica_InversaRama(x, y, p->rama, sol) != CIN_OK) {
		return EVAL_ERROR;
	}

	float w1 = fabsf(sol->th1_deg - p->th1_deg) / dt_s;
	float w2 = fabsf(sol->th2_deg - p->th2_deg) / dt_s;

	return (w1 > tope_velocidad(p->w1_dps, dt_s) || w2 > tope_velocidad(p->w2_dps, dt_s))
		 ? EVAL_RAPIDO : EVAL_OK;
}

/**
 * @brief Mayor velocidad desde la que todavia se puede frenar con 'a' en el resto del tramo.
 *
 * La forma continua sqrtf(2*a*resto) no cierra en ticks discretos: el ultimo tick
 * frena hasta un 50 % de mas. Aca se busca la rampa exacta: N ticks con velocidades
 * que bajan de a a*dt, la ultima <= a*dt, y que suman justo `resto`:
 *     resto/dt = N*v_N + a*dt*N*(N-1)/2      con N el menor que da v_N <= a*dt
 * La velocidad del primer tick de esa rampa es a*dt*(R/N + (N-1)/2), R = resto/(a*dt^2).
 * Es continua en N, asi que un N corrido en uno por redondeo no cambia el resultado.
 *
 * @param resto_mm Distancia que falta (>= 0).
 * @return Velocidad (mm/s) para el tick actual.
 */
static float velocidad_de_frenado(float resto_mm, float a_mm_s2, float dt_s)
{
	float adt = a_mm_s2 * dt_s;
	float r = resto_mm / (adt * dt_s);
	float n = ceilf(0.5f * (sqrtf(1.0f + 8.0f * r) - 1.0f));

	if (n < 1.0f) {
		n = 1.0f;
	}
	return adt * (r / n + 0.5f * (n - 1.0f));
}

plan_res_t Planificador_Preparar(plan_t *p, float th1_act_deg, float th2_act_deg,
								 float xf_mm, float yf_mm, float v_max_mm_s, float a_mm_s2)
{
	static const int ramas[2] = { CIN_RAMA_POS, CIN_RAMA_NEG };
	float x0, y0;

	if (p == NULL || !(v_max_mm_s > 0.0f) || !(a_mm_s2 > 0.0f)) {
		return PLAN_ERR_PARAM;
	}

	(void)Cinematica_Directa(th1_act_deg, th2_act_deg, &x0, &y0);

	float dx = xf_mm - x0;
	float dy = yf_mm - y0;
	float d = sqrtf(dx * dx + dy * dy);

	if (!(d == d)) {
		return PLAN_ERR_PARAM;	/* NaN en el destino */
	}
	if (d < MOVE_DIST_MIN_MM) {
		return PLAN_YA_EN_DESTINO;
	}
	if (Cinematica_ExcesoRadio(xf_mm, yf_mm) > 0.0f) {
		return PLAN_ERR_RADIO;
	}

	/* Una rama solo es candidata si su IK en el origen coincide con la postura actual;
	 * si no, el primer tick seria un salto. Lejos de theta2 = 0 esto deja una sola. */
	bool rama_ok[2];
	ik_sol_t sol_fin[2];
	for (int k = 0; k < 2; k++) {
		ik_sol_t sol;
		rama_ok[k] = Cinematica_InversaRama(x0, y0, ramas[k], &sol) == CIN_OK
				&& fabsf(sol.th1_deg - th1_act_deg) <= PLAN_TOL_RAMA_DEG
				&& fabsf(sol.th2_deg - th2_act_deg) <= PLAN_TOL_RAMA_DEG;
	}

	int n = (int)ceilf(d / PREFLIGHT_STEP_MM);
	float exceso_prev = Cinematica_ExcesoRadio(x0, y0);

	for (int i = 0; i <= n; i++) {
		float t = (float)i / (float)n;
		float x = (i == n) ? xf_mm : x0 + dx * t;
		float y = (i == n) ? yf_mm : y0 + dy * t;

		/* El exceso de radio no puede crecer: asi un tramo con ambos extremos validos
		 * que cruza la zona prohibida se rechaza, pero se puede salir de HOME (r > R_MAX_OP). */
		float exceso = Cinematica_ExcesoRadio(x, y);
		if (exceso > exceso_prev + PLAN_TOL_RADIO_MM) {
			return PLAN_ERR_RADIO;
		}
		exceso_prev = exceso;

		for (int k = 0; k < 2; k++) {
			if (rama_ok[k] && Cinematica_InversaRama(x, y, ramas[k], &sol_fin[k]) != CIN_OK) {
				rama_ok[k] = false;
			}
		}
		if (!rama_ok[0] && !rama_ok[1]) {
			return PLAN_ERR_LIMITE;
		}
	}

	/* Entre las ramas que sirven, la de menor recorrido articular total. */
	int elegida = -1;
	float costo_min = 0.0f;
	for (int k = 0; k < 2; k++) {
		if (!rama_ok[k]) {
			continue;
		}
		float costo = fabsf(sol_fin[k].th1_deg - th1_act_deg)
					+ fabsf(sol_fin[k].th2_deg - th2_act_deg);
		if (elegida < 0 || costo < costo_min) {
			elegida = k;
			costo_min = costo;
		}
	}

	p->x0_mm = x0;
	p->y0_mm = y0;
	p->xf_mm = xf_mm;
	p->yf_mm = yf_mm;
	p->ux = dx / d;
	p->uy = dy / d;
	p->d_mm = d;
	p->d_obj_mm = d;
	p->s_mm = 0.0f;
	p->v_mm_s = 0.0f;
	p->v_max_mm_s = v_max_mm_s;
	p->a_mm_s2 = a_mm_s2;
	p->th1_deg = th1_act_deg;
	p->th2_deg = th2_act_deg;
	p->w1_dps = 0.0f;
	p->w2_dps = 0.0f;
	p->rama = ramas[elegida];
	p->ticks_atasco = 0;
	p->activo = true;
	p->detenido = false;
	return PLAN_OK;
}

plan_paso_t Planificador_Paso(plan_t *p, float dt_s, float *th1_deg, float *th2_deg)
{
	if (p == NULL || th1_deg == NULL || th2_deg == NULL || !p->activo || !(dt_s > 0.0f)) {
		return PLAN_PASO_ERROR;
	}

	float resto = p->d_obj_mm - p->s_mm;
	if (resto < 0.0f) {
		resto = 0.0f;
	}

	float v_freno = velocidad_de_frenado(resto, p->a_mm_s2, dt_s);

	float v_cmd = p->v_mm_s + p->a_mm_s2 * dt_s;
	if (v_cmd > p->v_max_mm_s) {
		v_cmd = p->v_max_mm_s;
	}
	if (v_cmd > v_freno) {
		v_cmd = v_freno;
	}

	float ds = v_cmd * dt_s;
	bool llega = false;
	if (ds >= resto - PLAN_TOL_FIN_MM) {
		ds = resto;
		llega = true;
	}

	ik_sol_t sol;
	eval_t ev = evaluar(p, p->s_mm + ds, llega && !p->detenido, dt_s, &sol);

	if (ev == EVAL_RAPIDO) {
		/* Limite articular: se busca por biparticion el mayor avance que respeta el tope
		 * de velocidad y de aceleracion de las dos juntas. Una regla de tres no alcanza
		 * cerca de la singularidad, donde el angulo crece como la raiz del avance. */
		float lo = 0.0f;
		float hi = ds;
		bool hay_sol = false;
		llega = false;

		for (int i = 0; i < PLAN_LIM_ITER; i++) {
			float medio = 0.5f * (lo + hi);
			ik_sol_t cand;

			ev = evaluar(p, p->s_mm + medio, false, dt_s, &cand);
			if (ev == EVAL_ERROR) {
				break;
			}
			if (ev == EVAL_OK) {
				lo = medio;
				sol = cand;
				hay_sol = true;
			} else {
				hi = medio;
			}
		}
		if (ev != EVAL_ERROR) {
			ev = EVAL_OK;
			ds = lo;
			if (!hay_sol) {
				/* Ni el avance mas chico sirve: este tick no se mueve. */
				sol.th1_deg = p->th1_deg;
				sol.th2_deg = p->th2_deg;
				p->ticks_atasco++;
			}
		}
	}

	if (ev == EVAL_ERROR || p->ticks_atasco > PLAN_TICKS_ATASCO) {
		p->activo = false;
		p->v_mm_s = 0.0f;
		return PLAN_PASO_ERROR;
	}
	if (ds > 0.0f) {
		p->ticks_atasco = 0;
	}

	/* Velocidad articular real de este tick: es la base del tope de aceleracion del proximo. */
	p->w1_dps = fabsf(sol.th1_deg - p->th1_deg) / dt_s;
	p->w2_dps = fabsf(sol.th2_deg - p->th2_deg) / dt_s;

	p->th1_deg = sol.th1_deg;
	p->th2_deg = sol.th2_deg;
	*th1_deg = sol.th1_deg;
	*th2_deg = sol.th2_deg;

	if (llega) {
		p->s_mm = p->d_obj_mm;
		p->v_mm_s = 0.0f;
		p->activo = false;
		return PLAN_PASO_FIN;
	}

	p->s_mm += ds;
	/* La velocidad real del tick, no la pedida: si el limite articular freno el
	 * avance, la rampa siguiente tiene que partir de lo que realmente se movio. */
	p->v_mm_s = ds / dt_s;
	return PLAN_PASO_EN_CURSO;
}

void Perfil_Iniciar(perfil_t *pf, float distancia, float v_max, float a)
{
	if (pf == NULL) {
		return;
	}
	pf->d_total = (distancia > 0.0f) ? distancia : 0.0f;
	pf->d_obj = pf->d_total;
	pf->s = 0.0f;
	pf->v = 0.0f;
	pf->v_max = v_max;
	pf->a = a;
	pf->detenido = false;
}

bool Perfil_Paso(perfil_t *pf, float dt_s)
{
	if (pf == NULL || !(dt_s > 0.0f) || !(pf->a > 0.0f)) {
		return true;
	}

	float resto = pf->d_obj - pf->s;
	if (resto < 0.0f) {
		resto = 0.0f;
	}

	float v_cmd = pf->v + pf->a * dt_s;
	float v_freno = velocidad_de_frenado(resto, pf->a, dt_s);
	if (v_cmd > pf->v_max) {
		v_cmd = pf->v_max;
	}
	if (v_cmd > v_freno) {
		v_cmd = v_freno;
	}

	float ds = v_cmd * dt_s;
	/* Holgura relativa: las unidades del perfil las pone quien lo usa (grados, us...). */
	if (ds >= resto - PERFIL_TOL_FIN * pf->d_total) {
		pf->s = pf->d_obj;
		pf->v = 0.0f;
		return true;
	}
	pf->s += ds;
	pf->v = v_cmd;
	return false;
}

void Perfil_Detener(perfil_t *pf)
{
	if (pf == NULL) {
		return;
	}

	float d_parada = pf->s + (pf->v * pf->v) / (2.0f * pf->a);

	if (d_parada < pf->d_obj) {
		pf->d_obj = d_parada;
		pf->detenido = true;
	}
}

void Planificador_Detener(plan_t *p)
{
	if (p == NULL || !p->activo) {
		return;
	}

	float d_parada = p->s_mm + (p->v_mm_s * p->v_mm_s) / (2.0f * p->a_mm_s2);

	/* Si ya esta frenando para llegar al destino, no hay nada que acortar. */
	if (d_parada < p->d_obj_mm) {
		p->d_obj_mm = d_parada;
		p->detenido = true;
	}
}
