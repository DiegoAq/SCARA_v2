/**
 * @file sim_move.c
 * @brief Simula un MOVE en la PC e imprime la trayectoria en CSV para graficarla.
 *
 * Uso:
 *   sim_move x0 y0 xf yf [speed] [accel] [t_stop_ms]     parte de IK(x0, y0), rama +
 *   sim_move J q1 q2 xf yf [speed] [accel] [t_stop_ms]   parte de una postura articular
 *
 * El CSV sale por stdout y el resumen por stderr:
 *   sim_move 300 -100 300 100 > recta.csv
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "cinematica.h"
#include "config.h"
#include "parser.h"
#include "planificador.h"

#define DT_S	((float)TICK_MS / 1000.0f)

static const char *nombre_error(plan_res_t r)
{
	switch (r) {
	case PLAN_YA_EN_DESTINO:	return "ya esta en el destino";
	case PLAN_ERR_RADIO:		return "ERR 2 fuera del espacio de trabajo";
	case PLAN_ERR_LIMITE:		return "ERR 3 limite articular";
	case PLAN_ERR_PARAM:		return "ERR 6 parametro fuera de rango";
	default:					return "OK";
	}
}

static int uso(void)
{
	fprintf(stderr,
			"uso: sim_move x0 y0 xf yf [speed] [accel] [t_stop_ms]\n"
			"     sim_move J q1 q2 xf yf [speed] [accel] [t_stop_ms]\n");
	return 2;
}

int main(int argc, char **argv)
{
	float num[7];
	int n = 0;
	bool por_juntas = (argc > 1 && (strcmp(argv[1], "J") == 0 || strcmp(argv[1], "j") == 0));
	int primero = por_juntas ? 2 : 1;

	for (int i = primero; i < argc && n < 7; i++) {
		if (!Parser_LeerFloat(argv[i], &num[n])) {
			fprintf(stderr, "argumento invalido: %s\n", argv[i]);
			return uso();
		}
		n++;
	}
	if (n < 4) {
		return uso();
	}

	float xf = num[2];
	float yf = num[3];
	float speed = (n > 4) ? num[4] : SPEED_DEFAULT_MM_S;
	float accel = (n > 5) ? num[5] : ACCEL_DEFAULT_MM_S2;
	float t_stop_ms = (n > 6) ? num[6] : -1.0f;

	float th1 = num[0];
	float th2 = num[1];
	if (!por_juntas) {
		ik_sol_t sol[2];
		if (Cinematica_Inversa(num[0], num[1], sol) == 0) {
			fprintf(stderr, "el punto de partida (%.1f, %.1f) no es alcanzable\n",
					(double)num[0], (double)num[1]);
			return 1;
		}
		th1 = sol[0].th1_deg;
		th2 = sol[0].th2_deg;
	}

	plan_t p;
	plan_res_t res = Planificador_Preparar(&p, th1, th2, xf, yf, speed, accel);
	if (res != PLAN_OK) {
		fprintf(stderr, "pre-vuelo: %s\n", nombre_error(res));
		return 1;
	}

	printf("t_ms,s_mm,v_mm_s,x_cmd,y_cmd,q1_deg,q2_deg,w1_dps,w2_dps,desv_mm\n");
	printf("0,0.0000,0.000,%.4f,%.4f,%.4f,%.4f,0.000,0.000,0.00000\n",
		   (double)p.x0_mm, (double)p.y0_mm, (double)th1, (double)th2);

	float desv_max = 0.0f;
	float w_max = 0.0f;
	float v_pico = 0.0f;
	float s_prev = 0.0f;
	bool stop_pedido = false;
	int tick = 0;
	plan_paso_t paso;

	do {
		float th1_prev = th1;
		float th2_prev = th2;

		if (!stop_pedido && t_stop_ms >= 0.0f && (float)(tick * TICK_MS) >= t_stop_ms) {
			Planificador_Detener(&p);
			stop_pedido = true;
		}

		paso = Planificador_Paso(&p, DT_S, &th1, &th2);
		tick++;
		if (paso == PLAN_PASO_ERROR) {
			fprintf(stderr, "el planificador aborto en t = %d ms\n", tick * TICK_MS);
			return 1;
		}

		float x, y;
		(void)Cinematica_Directa(th1, th2, &x, &y);

		float w1 = (th1 - th1_prev) / DT_S;
		float w2 = (th2 - th2_prev) / DT_S;
		float v = (p.s_mm - s_prev) / DT_S;
		float desv = fabsf((x - p.x0_mm) * p.uy - (y - p.y0_mm) * p.ux);

		if (desv > desv_max) desv_max = desv;
		if (fabsf(w1) > w_max) w_max = fabsf(w1);
		if (fabsf(w2) > w_max) w_max = fabsf(w2);
		if (v > v_pico) v_pico = v;
		s_prev = p.s_mm;

		printf("%d,%.4f,%.3f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.5f\n",
			   tick * TICK_MS, (double)p.s_mm, (double)v, (double)x, (double)y,
			   (double)th1, (double)th2, (double)w1, (double)w2, (double)desv);
	} while (paso == PLAN_PASO_EN_CURSO);

	fprintf(stderr,
			"rama %s | d = %.2f mm | recorrido = %.2f mm%s | t = %.2f s (%d ticks)\n"
			"v pico = %.2f mm/s | w max = %.2f gr/s (tope %.0f) | desviacion max = %.2e mm\n",
			p.rama == CIN_RAMA_POS ? "+" : "-", (double)p.d_mm, (double)p.s_mm,
			p.detenido ? " (STOP)" : "", (double)((float)tick * DT_S), tick,
			(double)v_pico, (double)w_max, (double)JOINT_VMAX_DPS, (double)desv_max);
	return 0;
}
