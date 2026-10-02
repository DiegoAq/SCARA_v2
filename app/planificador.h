/**
 * @file planificador.h
 * @brief Movimiento lineal con perfil trapezoidal y limite de velocidad articular
 *        (nucleo portable, sin hardware).
 *
 * Uso: Planificador_Preparar() valida TODO el segmento y elige la rama de codo
 * (pre-vuelo); despues se llama Planificador_Paso() una vez por tick hasta que
 * devuelva PLAN_PASO_FIN. No mueve nada: solo calcula los angulos a comandar.
 */
#ifndef PLANIFICADOR_H
#define PLANIFICADOR_H

#include <stdbool.h>

/** Resultado del pre-vuelo. */
typedef enum {
	PLAN_OK = 0,			/**< Segmento valido; llamar a Planificador_Paso() */
	PLAN_YA_EN_DESTINO,		/**< Distancia < MOVE_DIST_MIN_MM: no hay nada que mover */
	PLAN_ERR_RADIO,			/**< Destino o tramo fuera del espacio de trabajo */
	PLAN_ERR_LIMITE,		/**< Ninguna rama cumple los limites articulares en todo el tramo */
	PLAN_ERR_PARAM			/**< Velocidad/aceleracion no positivas o puntero NULL */
} plan_res_t;

/** Resultado de un tick de ejecucion. */
typedef enum {
	PLAN_PASO_EN_CURSO = 0,	/**< Hay nuevos angulos para comandar; seguir llamando */
	PLAN_PASO_FIN,			/**< Ultimo punto entregado (destino, o punto de parada tras Detener) */
	PLAN_PASO_ERROR			/**< No se pudo avanzar; los angulos de salida no se tocan */
} plan_paso_t;

/** Estado de un movimiento lineal. Se lee libremente; solo lo escribe el planificador. */
typedef struct {
	float x0_mm, y0_mm;		/**< Origen del segmento */
	float xf_mm, yf_mm;		/**< Destino pedido */
	float ux, uy;			/**< Versor del segmento */
	float d_mm;				/**< Longitud total del segmento */
	float d_obj_mm;			/**< Distancia a la que se detiene (d_mm, o menos tras Detener) */
	float s_mm;				/**< Distancia recorrida */
	float v_mm_s;			/**< Velocidad de avance del ultimo tick */
	float v_max_mm_s;		/**< Velocidad de crucero */
	float a_mm_s2;			/**< Aceleracion y frenado */
	float th1_deg, th2_deg;	/**< Ultima postura comandada */
	int   rama;				/**< Rama de codo, fija durante todo el movimiento */
	int   ticks_atasco;		/**< Ticks seguidos sin avanzar */
	bool  activo;			/**< true entre Preparar OK y el fin del movimiento */
	bool  detenido;			/**< true si se pidio Detener */
} plan_t;

/**
 * @brief Pre-vuelo: valida el segmento completo y deja el movimiento listo.
 *
 * Muestrea el segmento cada PREFLIGHT_STEP_MM. Reglas:
 *  - el destino debe estar dentro del anillo operativo;
 *  - el exceso de radio fuera del anillo nunca crece a lo largo del tramo (se puede
 *    partir desde afuera, p. ej. HOME con el brazo estirado, pero solo para entrar);
 *  - la rama de codo debe coincidir con la postura actual y cumplir los limites
 *    articulares en TODAS las muestras. No se cambia de rama a mitad de camino.
 * Si falla, @p p no se modifica y no hay que mover nada.
 *
 * @param p Estado del movimiento (salida).
 * @param th1_act_deg,th2_act_deg Postura comandada actual (grados).
 * @param xf_mm,yf_mm Destino (mm).
 * @param v_max_mm_s Velocidad de crucero (> 0).
 * @param a_mm_s2 Aceleracion (> 0).
 * @return PLAN_OK, PLAN_YA_EN_DESTINO o el motivo del rechazo.
 */
plan_res_t Planificador_Preparar(plan_t *p, float th1_act_deg, float th2_act_deg,
								 float xf_mm, float yf_mm, float v_max_mm_s, float a_mm_s2);

/**
 * @brief Avanza un tick: perfil trapezoidal + limite JOINT_VMAX_DPS por junta.
 *
 * Si alguna junta tuviera que superar JOINT_VMAX_DPS se reduce el *avance* sobre la
 * recta, no la recta: la trayectoria se conserva y solo se tarda mas.
 *
 * @param p Movimiento preparado.
 * @param dt_s Periodo del tick (s), > 0.
 * @param th1_deg,th2_deg Salida: angulos a comandar en este tick (grados).
 * @return PLAN_PASO_EN_CURSO, PLAN_PASO_FIN o PLAN_PASO_ERROR.
 */
plan_paso_t Planificador_Paso(plan_t *p, float dt_s, float *th1_deg, float *th2_deg);

/**
 * @brief Pide frenar con la aceleracion configurada y detenerse sobre la recta.
 *
 * Acorta la distancia objetivo a s + v^2/(2a). Hay que seguir llamando a
 * Planificador_Paso() hasta PLAN_PASO_FIN.
 */
void Planificador_Detener(plan_t *p);

#endif /* PLANIFICADOR_H */
