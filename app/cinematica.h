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
 * @brief Lleva un angulo a la vuelta centrada en otro: (centro - 180, centro + 180].
 *
 * Sirve para que un angulo de junta medido no de la vuelta si el rango de la junta llega a 180.
 * @param ang_deg Angulo en grados, cualquier valor finito.
 * @param centro_deg Centro de la vuelta buscada.
 */
float Cinematica_VueltaCentrada(float ang_deg, float centro_deg);

/**
 * @brief Error de posicion que provoca un error en las juntas, a primer orden (Jacobiano).
 *
 * Es el calculo analitico que piden las pautas: con la resolucion del encoder da el menor
 * error que se puede distinguir en esa postura. Toma el peor caso de signos.
 * @param th1_deg,th2_deg Postura (grados).
 * @param d1_deg,d2_deg Error de cada junta (grados).
 * @return Error de posicion en el plano (mm).
 */
float Cinematica_ErrorPorJuntas(float th1_deg, float th2_deg, float d1_deg, float d2_deg);

/**
 * @brief Punto de una circunferencia.
 * @param cx_mm,cy_mm Centro (mm).
 * @param r_mm Radio (mm).
 * @param ang_deg Angulo desde +X (grados).
 * @param x_mm,y_mm Salida (mm).
 */
void Cinematica_PuntoCircunferencia(float cx_mm, float cy_mm, float r_mm, float ang_deg,
									float *x_mm, float *y_mm);

/**
 * @brief Arco tangente de dos argumentos, en grados.
 * @return Angulo en (-180, 180]; 0 si los dos argumentos son 0.
 */
float Cinematica_Atan2Deg(float y, float x);

/**
 * @brief Vector unitario en una direccion.
 * @param ang_deg Angulo desde +X (grados).
 * @param ux,uy Salida.
 */
void Cinematica_Direccion(float ang_deg, float *ux, float *uy);

/**
 * @brief Direccion en la mesa de la linea que va del sensor B al sensor A.
 *
 * Es la suma de las dos juntas (la garra va montada sobre el antebrazo), de la orientacion y de
 * GARRA_YAW_CERO_DEG.
 * @param th1_deg,th2_deg Postura (grados).
 * @param rot_deg Orientacion de la garra (grados).
 * @return Angulo desde +X, en (-180, 180].
 */
float Cinematica_YawGarra(float th1_deg, float th2_deg, float rot_deg);

/**
 * @brief Punto de la mesa al que apunta un sensor de distancia de la garra.
 * @param th1_deg,th2_deg Postura (grados).
 * @param rot_deg Orientacion de la garra (grados).
 * @param lado +1 para el sensor A, -1 para el B.
 * @param x_mm,y_mm Salida (mm).
 */
void Cinematica_PuntoSensor(float th1_deg, float th2_deg, float rot_deg, float lado,
							float *x_mm, float *y_mm);

/**
 * @brief Orientacion que deja la garra en una direccion de la mesa.
 *
 * Girada 180 grados la garra agarra igual: de las soluciones dentro de [ROT_MIN_DEG, ROT_MAX_DEG]
 * elige la mas cercana a la orientacion actual, para girar lo menos posible.
 * @param th1_deg,th2_deg Postura en la que va a quedar (grados).
 * @param yaw_deg Direccion buscada para la linea de sensores (grados).
 * @param rot_actual_deg Orientacion actual (grados).
 * @param rot_deg Salida: orientacion a comandar (grados).
 * @return false si ninguna solucion entra en los limites.
 */
bool Cinematica_RotParaYaw(float th1_deg, float th2_deg, float yaw_deg, float rot_actual_deg,
						   float *rot_deg);

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
