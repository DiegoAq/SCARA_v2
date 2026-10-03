/**
 * @file cinematica.h
 * @brief Cinematica directa e inversa del brazo SCARA de 2 juntas (nucleo portable).
 *
 * Marco: origen en el eje del hombro, brazo en HOME apuntando a +X, +Y a la izquierda.
 * theta1 se mide desde +X; theta2 es relativo al eslabon 1 (0 = brazo recto).
 * Positivo = antihorario. API en mm y grados; los radianes no salen de cinematica.c.
 */
#ifndef CINEMATICA_H
#define CINEMATICA_H

#include <stdbool.h>

/** Rama de codo con theta2 >= 0. */
#define CIN_RAMA_POS	(+1)
/** Rama de codo con theta2 <= 0. */
#define CIN_RAMA_NEG	(-1)

/** Solucion de la cinematica inversa. */
typedef struct {
	float th1_deg;	/**< Angulo del hombro, grados */
	float th2_deg;	/**< Angulo del codo, grados */
	int   rama;		/**< CIN_RAMA_POS o CIN_RAMA_NEG */
} ik_sol_t;

/** Resultado de la cinematica inversa para una rama. */
typedef enum {
	CIN_OK = 0,			/**< Solucion valida y dentro de los limites articulares */
	CIN_ERR_RADIO,		/**< El punto esta fuera del alcance geometrico del brazo */
	CIN_ERR_LIMITE		/**< Alcanzable, pero alguna junta sale de [JOINT_MIN, JOINT_MAX] */
} cin_res_t;

/**
 * @brief Cinematica directa.
 * @param th1_deg Angulo del hombro (grados).
 * @param th2_deg Angulo del codo (grados).
 * @param x_mm,y_mm Salida: posicion del efector (mm).
 * @return false si algun puntero es NULL. No verifica limites articulares.
 */
bool Cinematica_Directa(float th1_deg, float th2_deg, float *x_mm, float *y_mm);

/**
 * @brief Cinematica inversa de una rama de codo concreta.
 * @param x_mm,y_mm Punto objetivo (mm).
 * @param rama CIN_RAMA_POS o CIN_RAMA_NEG.
 * @param sol Salida. Con CIN_OK y con CIN_ERR_LIMITE queda la solucion geometrica
 *            (en el segundo caso, fuera de limites); con CIN_ERR_RADIO no se modifica.
 * @return CIN_OK, CIN_ERR_RADIO o CIN_ERR_LIMITE.
 */
cin_res_t Cinematica_InversaRama(float x_mm, float y_mm, int rama, ik_sol_t *sol);

/**
 * @brief Cinematica inversa: evalua las dos ramas de codo.
 * @param x_mm,y_mm Punto objetivo (mm).
 * @param sol Salida: las ramas validas, primero la positiva si lo es.
 * @return Cantidad de ramas validas (0, 1 o 2). No verifica el anillo operativo.
 */
int Cinematica_Inversa(float x_mm, float y_mm, ik_sol_t sol[2]);

/**
 * @brief Indica si un punto puede ser destino de un movimiento.
 * @return true si esta dentro del anillo operativo [R_MIN_OP_MM, R_MAX_OP_MM] y
 *         al menos una rama cumple los limites articulares.
 */
bool Cinematica_PuntoAlcanzable(float x_mm, float y_mm);

/**
 * @brief Distancia radial (mm) por la que un punto queda fuera del anillo operativo.
 * @return 0 si esta dentro de [R_MIN_OP_MM, R_MAX_OP_MM]; positivo si esta fuera.
 */
float Cinematica_ExcesoRadio(float x_mm, float y_mm);

/**
 * @brief Verifica los limites articulares (con la holgura JOINT_LIM_TOL_DEG).
 * @param th1_deg,th2_deg Angulos de junta (grados).
 */
bool Cinematica_JuntasEnLimites(float th1_deg, float th2_deg);

/**
 * @brief Lleva un angulo al intervalo (-180, 180].
 * @param ang_deg Angulo en grados, cualquier valor finito.
 */
float Cinematica_NormalizarDeg(float ang_deg);

/**
 * @brief Convierte la lectura de un encoder absoluto en angulo de junta.
 *
 * theta = signo * normalizar(enc_deg - cero_deg). La resta se normaliza antes de aplicar
 * el signo, asi el cruce 0/360 del encoder no produce un salto en la junta.
 *
 * @param enc_deg Lectura del encoder (grados, 0..360).
 * @param cero_deg Lectura del encoder con la junta en theta = 0.
 * @param signo +1 si la lectura crece en sentido antihorario; -1 si decrece.
 * @return Angulo de junta en (-180, 180].
 */
float Cinematica_JuntaDesdeEncoder(float enc_deg, float cero_deg, float signo);

#endif /* CINEMATICA_H */
