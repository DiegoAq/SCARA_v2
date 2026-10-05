/**
 * @file test_planificador.c
 * @brief Tests en PC del pre-vuelo y del perfil de movimiento (criterios de E1).
 *
 * Los puntos estan en un marco LOCAL: +X local es la direccion en la que apunta el brazo
 * en HOME (HOME_TH1_DEG) y +Y local queda a su izquierda; frente() los pasa al marco del
 * robot. Suponen: L1 = L2 = 200 mm, hombro +-90 alrededor de HOME, codo +-90, anillo
 * operativo 285..395 mm.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "cinematica.h"
#include "config.h"
#include "planificador.h"

#define DT_S			((float)TICK_MS / 1000.0f)
#define TICKS_MAX		100000
#define TOL_RECTA_MM	0.01f	/* criterio de E1: desviacion maxima de la recta */
#define TOL_DESTINO_MM	1.0e-3f
#define TOL_REL			0.01f	/* holgura de redondeo al comparar contra un tope */

/* Velocidad y aceleracion altas para forzar el limite articular. Son del test: no
 * dependen de los topes de los comandos SPEED/ACCEL, que el equipo ajusta en config.h. */
#define V_RAPIDA_MM_S	150.0f
#define A_RAPIDA_MM_S2	500.0f

/** Lo que se mide al simular un movimiento completo. */
typedef struct {
	int   ticks;
	float desv_max_mm;		/**< Distancia maxima a la recta (FK de lo comandado) */
	float w_max_dps;		/**< Velocidad articular maxima */
	float v_pico_mm_s;		/**< Velocidad de avance maxima */
	float acel_max_mm_s2;	/**< Mayor aumento de velocidad entre ticks */
	float frenado_max_mm_s2;/**< Mayor caida de velocidad entre ticks */
	float x_fin, y_fin;		/**< FK de la ultima postura comandada */
	bool  rama_constante;	/**< theta2 nunca cambio de signo */
	bool  s_monotono;		/**< la distancia recorrida nunca retrocedio */
	plan_paso_t resultado;
} medicion_t;

/** Pasa un punto del marco local (frente = +X local) al marco del robot. */
static void frente(float xl, float yl, float *x, float *y)
{
	const float rad = HOME_TH1_DEG * 0.017453292519943295f;

	*x = xl * cosf(rad) - yl * sinf(rad);
	*y = xl * sinf(rad) + yl * cosf(rad);
}

/** Postura de partida: IK de un punto local en una rama. */
static void postura(float xl, float yl, int rama, float *th1, float *th2)
{
	float x, y;
	ik_sol_t sol;

	frente(xl, yl, &x, &y);
	assert(Cinematica_InversaRama(x, y, rama, &sol) == CIN_OK);
	*th1 = sol.th1_deg;
	*th2 = sol.th2_deg;
}

/** Planificador_Preparar con el destino dado en el marco local. */
static plan_res_t preparar(plan_t *p, float th1, float th2, float xl, float yl, float v, float a)
{
	float x, y;

	frente(xl, yl, &x, &y);
	return Planificador_Preparar(p, th1, th2, x, y, v, a);
}

/** Distancia (mm) entre un punto del robot y un punto dado en el marco local. */
static float distancia_a(float x, float y, float xl, float yl)
{
	float xe, ye;

	frente(xl, yl, &xe, &ye);
	return hypotf(x - xe, y - ye);
}

/**
 * @brief Ejecuta el movimiento tick a tick midiendo todo.
 * @param tick_stop Tick en el que se llama a Planificador_Detener (< 0: nunca).
 */
static medicion_t correr(plan_t *p, int tick_stop)
{
	medicion_t m = { .rama_constante = true, .s_monotono = true, .resultado = PLAN_PASO_EN_CURSO };
	float th1 = p->th1_deg;
	float th2 = p->th2_deg;
	float v_prev = 0.0f;
	float s_prev = 0.0f;

	while (m.resultado == PLAN_PASO_EN_CURSO) {
		float th1_prev = th1;
		float th2_prev = th2;

		if (m.ticks == tick_stop) {
			Planificador_Detener(p);
		}
		m.resultado = Planificador_Paso(p, DT_S, &th1, &th2);
		m.ticks++;
		assert(m.ticks < TICKS_MAX);
		if (m.resultado == PLAN_PASO_ERROR) {
			break;
		}

		float w1 = fabsf(th1 - th1_prev) / DT_S;
		float w2 = fabsf(th2 - th2_prev) / DT_S;
		if (w1 > m.w_max_dps) m.w_max_dps = w1;
		if (w2 > m.w_max_dps) m.w_max_dps = w2;

		/* Distancia del punto comandado a la recta: producto cruz con el versor. */
		assert(Cinematica_Directa(th1, th2, &m.x_fin, &m.y_fin));
		float desv = fabsf((m.x_fin - p->x0_mm) * p->uy - (m.y_fin - p->y0_mm) * p->ux);
		if (desv > m.desv_max_mm) m.desv_max_mm = desv;

		/* La velocidad del tick final se calcula con la distancia: el plan ya la dejo en 0. */
		float v = (p->s_mm - s_prev) / DT_S;
		float dv = (v - v_prev) / DT_S;
		if (dv > m.acel_max_mm_s2) m.acel_max_mm_s2 = dv;
		if (-dv > m.frenado_max_mm_s2) m.frenado_max_mm_s2 = -dv;
		if (v > m.v_pico_mm_s) m.v_pico_mm_s = v;

		if (p->s_mm < s_prev) m.s_monotono = false;
		if (p->rama == CIN_RAMA_POS ? th2 < 0.0f : th2 > 0.0f) m.rama_constante = false;

		v_prev = v;
		s_prev = p->s_mm;
	}
	return m;
}

/** Perfil trapezoidal lejos de la singularidad: el limite articular no interviene. */
static void test_perfil_trapezoidal(void)
{
	plan_t p;
	float th1, th2;
	const float v = SPEED_DEFAULT_MM_S;
	const float a = ACCEL_DEFAULT_MM_S2;

	postura(300.0f, -100.0f, CIN_RAMA_POS, &th1, &th2);
	assert(preparar(&p, th1, th2, 300.0f, 100.0f, v, a) == PLAN_OK);
	assert(fabsf(p.d_mm - 200.0f) < 1e-2f);

	medicion_t m = correr(&p, -1);

	assert(m.resultado == PLAN_PASO_FIN);
	assert(!p.activo && !p.detenido);
	assert(m.s_monotono && m.rama_constante);
	assert(m.desv_max_mm < TOL_RECTA_MM);
	assert(distancia_a(m.x_fin, m.y_fin, 300.0f, 100.0f) < TOL_DESTINO_MM);

	/* Crucero a v, rampas de 'a' en la subida y en la bajada. */
	assert(fabsf(m.v_pico_mm_s - v) < v * TOL_REL);
	assert(m.acel_max_mm_s2 <= a * (1.0f + TOL_REL));
	assert(m.frenado_max_mm_s2 <= a * (1.0f + TOL_REL));
	assert(m.frenado_max_mm_s2 >= a * (1.0f - TOL_REL));
	assert(m.w_max_dps < JOINT_VMAX_DPS);

	/* Duracion teorica de un trapecio: d/v + v/a. */
	float t_teorico = p.d_mm / v + v / a;
	float t_real = (float)m.ticks * DT_S;
	assert(fabsf(t_real - t_teorico) < 3.0f * DT_S);

	printf("OK  perfil trapezoidal: %d ticks (%.2f s, teorico %.2f s), desv max %.1e mm, "
		   "w max %.1f gr/s\n", m.ticks, (double)t_real, (double)t_teorico,
		   (double)m.desv_max_mm, (double)m.w_max_dps);
}

/** Tramo tan corto que no llega a la velocidad de crucero (perfil triangular). */
static void test_tramo_corto(void)
{
	plan_t p;
	float th1, th2;

	postura(300.0f, 0.0f, CIN_RAMA_POS, &th1, &th2);
	assert(preparar(&p, th1, th2, 302.0f, 0.0f, SPEED_DEFAULT_MM_S, ACCEL_DEFAULT_MM_S2) == PLAN_OK);

	medicion_t m = correr(&p, -1);

	assert(m.resultado == PLAN_PASO_FIN);
	assert(m.v_pico_mm_s < SPEED_DEFAULT_MM_S);
	assert(m.acel_max_mm_s2 <= ACCEL_DEFAULT_MM_S2 * (1.0f + TOL_REL));
	assert(m.frenado_max_mm_s2 <= ACCEL_DEFAULT_MM_S2 * (1.0f + TOL_REL));
	assert(distancia_a(m.x_fin, m.y_fin, 302.0f, 0.0f) < TOL_DESTINO_MM);

	/* Por debajo de MOVE_DIST_MIN_MM no hay movimiento. */
	assert(preparar(&p, th1, th2, 300.05f, 0.0f,
					SPEED_DEFAULT_MM_S, ACCEL_DEFAULT_MM_S2) == PLAN_YA_EN_DESTINO);
	printf("OK  tramo corto (triangular) y distancia nula\n");
}

/** Movimiento radial rapido cerca de la singularidad: tiene que actuar el limite articular. */
static void test_limite_articular(void)
{
	plan_t p;
	float th1, th2;

	postura(395.0f, 0.0f, CIN_RAMA_POS, &th1, &th2);
	assert(preparar(&p, th1, th2, 300.0f, 0.0f, V_RAPIDA_MM_S, A_RAPIDA_MM_S2) == PLAN_OK);

	medicion_t m = correr(&p, -1);

	assert(m.resultado == PLAN_PASO_FIN);
	assert(m.w_max_dps <= JOINT_VMAX_DPS * 1.0001f);
	assert(m.w_max_dps > JOINT_VMAX_DPS * 0.9f);	/* el limite realmente actuo */
	assert(m.v_pico_mm_s < V_RAPIDA_MM_S);			/* ...y freno el avance */
	assert(m.desv_max_mm < TOL_RECTA_MM);
	assert(m.s_monotono && m.rama_constante);
	assert(distancia_a(m.x_fin, m.y_fin, 300.0f, 0.0f) < TOL_DESTINO_MM);

	printf("OK  limite articular: w max %.2f gr/s (tope %.0f), v pico %.1f de %.0f mm/s, "
		   "desv max %.1e mm\n", (double)m.w_max_dps, (double)JOINT_VMAX_DPS,
		   (double)m.v_pico_mm_s, (double)V_RAPIDA_MM_S, (double)m.desv_max_mm);
}

/** Partir de HOME (brazo estirado, r = 400 > R_MAX_OP): es la postura de arranque. */
static void test_desde_home(void)
{
	plan_t p;

	assert(preparar(&p, HOME_TH1_DEG, HOME_TH2_DEG, 300.0f, 100.0f,
					SPEED_DEFAULT_MM_S, ACCEL_DEFAULT_MM_S2) == PLAN_OK);

	medicion_t m = correr(&p, -1);

	assert(m.resultado == PLAN_PASO_FIN);
	assert(m.w_max_dps <= JOINT_VMAX_DPS * 1.0001f);
	assert(m.desv_max_mm < TOL_RECTA_MM);
	assert(m.s_monotono && m.rama_constante);
	assert(distancia_a(m.x_fin, m.y_fin, 300.0f, 100.0f) < TOL_DESTINO_MM);

	/* Desde afuera del anillo solo se puede entrar: alejarse mas se rechaza. */
	assert(preparar(&p, HOME_TH1_DEG, HOME_TH2_DEG, 396.0f, 50.0f,
					SPEED_DEFAULT_MM_S, ACCEL_DEFAULT_MM_S2) == PLAN_ERR_RADIO);

	printf("OK  salida desde HOME: %d ticks, w max %.2f gr/s, desv max %.1e mm\n",
		   m.ticks, (double)m.w_max_dps, (double)m.desv_max_mm);
}

/** STOP: frena con 'a', se detiene sobre la recta a v^2/(2a) y sin sobreimpulso. */
static void test_detener(void)
{
	plan_t p;
	float th1, th2;
	const float v = SPEED_DEFAULT_MM_S;
	const float a = ACCEL_DEFAULT_MM_S2;
	const int tick_stop = 100;	/* en pleno crucero */

	postura(300.0f, -100.0f, CIN_RAMA_POS, &th1, &th2);
	assert(preparar(&p, th1, th2, 300.0f, 100.0f, v, a) == PLAN_OK);

	/* Mismo movimiento hasta el tick de parada, para saber donde estaba al pedir STOP. */
	plan_t ref = p;
	float r1, r2;
	for (int i = 0; i < tick_stop; i++) {
		assert(Planificador_Paso(&ref, DT_S, &r1, &r2) == PLAN_PASO_EN_CURSO);
	}
	float s_esperado = ref.s_mm + ref.v_mm_s * ref.v_mm_s / (2.0f * a);

	medicion_t m = correr(&p, tick_stop);

	assert(m.resultado == PLAN_PASO_FIN);
	assert(p.detenido && !p.activo);
	assert(fabsf(p.s_mm - s_esperado) < 1e-3f);
	assert(p.s_mm < p.d_mm);
	assert(m.s_monotono);
	assert(m.desv_max_mm < TOL_RECTA_MM);
	assert(m.frenado_max_mm_s2 <= a * (1.0f + TOL_REL));

	/* Queda donde freno, no en el destino. */
	float x_esp = p.x0_mm + p.ux * s_esperado;
	float y_esp = p.y0_mm + p.uy * s_esperado;
	assert(hypotf(m.x_fin - x_esp, m.y_fin - y_esp) < 2.0f * TOL_DESTINO_MM);

	/* Detener sobre un movimiento terminado no hace nada. */
	Planificador_Detener(&p);
	assert(Planificador_Paso(&p, DT_S, &r1, &r2) == PLAN_PASO_ERROR);

	printf("OK  STOP: frena en %.2f mm (v^2/2a = %.2f mm), %d ticks de frenado\n",
		   (double)(p.s_mm - ref.s_mm), (double)(v * v / (2.0f * a)), m.ticks - tick_stop);
}

/** Lo que el pre-vuelo tiene que rechazar sin mover nada. */
static void test_rechazos(void)
{
	plan_t p = { 0 };
	const plan_t intacto = p;
	float th1, th2;
	float x, y;
	ik_sol_t fin;
	const float v = SPEED_DEFAULT_MM_S;
	const float a = ACCEL_DEFAULT_MM_S2;

	postura(300.0f, 0.0f, CIN_RAMA_POS, &th1, &th2);

	/* Destino fuera del espacio de trabajo: lejos, muy cerca de la base, o mas alla de R_MAX_OP. */
	assert(preparar(&p, th1, th2, 500.0f, 0.0f, v, a) == PLAN_ERR_RADIO);
	assert(preparar(&p, th1, th2, 200.0f, 150.0f, v, a) == PLAN_ERR_RADIO);
	assert(preparar(&p, th1, th2, 398.0f, 0.0f, v, a) == PLAN_ERR_RADIO);

	/* Parametros invalidos. */
	assert(preparar(&p, th1, th2, 350.0f, 0.0f, 0.0f, a) == PLAN_ERR_PARAM);
	assert(preparar(&p, th1, th2, 350.0f, 0.0f, v, -1.0f) == PLAN_ERR_PARAM);
	assert(Planificador_Preparar(NULL, th1, th2, 0.0f, 350.0f, v, a) == PLAN_ERR_PARAM);
	assert(Planificador_Preparar(&p, th1, th2, NAN, 0.0f, v, a) == PLAN_ERR_PARAM);

	/* Ambos extremos validos EN LA MISMA RAMA, pero la recta pasa a 210 mm de la base. */
	postura(200.0f, 220.0f, CIN_RAMA_POS, &th1, &th2);
	frente(220.0f, -200.0f, &x, &y);
	assert(Cinematica_InversaRama(x, y, CIN_RAMA_POS, &fin) == CIN_OK);
	assert(preparar(&p, th1, th2, 220.0f, -200.0f, v, a) == PLAN_ERR_RADIO);

	/* El ejemplo del SPEC 7.1: de un lado al otro pasando por la base. */
	postura(0.0f, 300.0f, CIN_RAMA_POS, &th1, &th2);
	frente(0.0f, -300.0f, &x, &y);
	assert(Cinematica_PuntoAlcanzable(x, y));
	assert(preparar(&p, th1, th2, 0.0f, -300.0f, v, a) == PLAN_ERR_RADIO);

	/* Tramo dentro del anillo, pero el destino solo existe en la otra rama de codo:
	 * se rechaza; no se cambia de rama a mitad de camino. */
	postura(200.0f, 250.0f, CIN_RAMA_NEG, &th1, &th2);
	frente(150.0f, 290.0f, &x, &y);
	assert(Cinematica_InversaRama(x, y, CIN_RAMA_POS, &fin) == CIN_OK);
	assert(Cinematica_InversaRama(x, y, CIN_RAMA_NEG, &fin) == CIN_ERR_LIMITE);
	assert(preparar(&p, th1, th2, 150.0f, 290.0f, v, a) == PLAN_ERR_LIMITE);

	/* Ningun rechazo toco el estado. */
	assert(p.activo == intacto.activo && p.d_mm == intacto.d_mm && p.rama == intacto.rama);

	printf("OK  rechazos del pre-vuelo\n");
}

/** La rama de codo es la de la postura de partida y no cambia. */
static void test_rama(void)
{
	plan_t p;
	float th1, th2;

	postura(300.0f, 100.0f, CIN_RAMA_NEG, &th1, &th2);
	assert(th2 < 0.0f);
	assert(preparar(&p, th1, th2, 300.0f, -100.0f, SPEED_DEFAULT_MM_S, ACCEL_DEFAULT_MM_S2) == PLAN_OK);
	assert(p.rama == CIN_RAMA_NEG);

	medicion_t m = correr(&p, -1);
	assert(m.resultado == PLAN_PASO_FIN);
	assert(m.rama_constante);
	assert(p.th2_deg < 0.0f);
	assert(m.desv_max_mm < TOL_RECTA_MM);

	postura(300.0f, 100.0f, CIN_RAMA_POS, &th1, &th2);
	assert(preparar(&p, th1, th2, 300.0f, -100.0f, SPEED_DEFAULT_MM_S, ACCEL_DEFAULT_MM_S2) == PLAN_OK);
	assert(p.rama == CIN_RAMA_POS);
	printf("OK  rama de codo fija durante el movimiento\n");
}

int main(void)
{
	printf("== test_planificador ==\n");
	test_perfil_trapezoidal();
	test_tramo_corto();
	test_limite_articular();
	test_desde_home();
	test_detener();
	test_rechazos();
	test_rama();
	printf("test_planificador: TODO OK\n");
	return 0;
}
