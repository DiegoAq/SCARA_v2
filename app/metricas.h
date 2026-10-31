/**
 * @file metricas.h
 * @brief Estadisticas de precision y repetibilidad, y ajuste de una recta (nucleo portable).
 *
 * El error de cada llegada es el de las pautas: e = sqrt(dx^2 + dy^2 + dz^2). Se acumula en
 * linea (Welford), sin guardar las muestras, asi que no hace falta memoria por cada ciclo.
 */
#ifndef METRICAS_H
#define METRICAS_H

#include <stdbool.h>
#include <stdint.h>

#define METRICAS_EJES	3

/** Acumulador de las llegadas a un mismo punto. */
typedef struct {
	uint32_t n;
	float e_media;
	float e_m2;
	float e_max;
	float med_media[METRICAS_EJES];		/**< Centroide de lo medido */
	float med_m2[METRICAS_EJES];
	float delta_media[METRICAS_EJES];	/**< Promedio de (medido - comandado) */
} metricas_t;

/** Resumen de un punto (mm). */
typedef struct {
	uint32_t n;
	float e_media;			/**< Promedio del error de cada llegada */
	float e_desvio;			/**< Desvio estandar muestral de ese error */
	float e_max;			/**< Peor llegada */
	float exactitud;		/**< Distancia del centroide de lo medido al punto comandado */
	float repetibilidad;	/**< 3 sigma de lo medido alrededor de su centroide */
} metricas_res_t;

/** Acumulador de un ajuste lineal y = pendiente * x + ordenada. */
typedef struct {
	uint32_t n;
	float sx, sy, sxx, sxy;
} recta_t;

/** @brief Deja el acumulador vacio. */
void Metricas_Iniciar(metricas_t *m);

/**
 * @brief Agrega una llegada.
 * @param cmd Posicion comandada x, y, z (mm).
 * @param med Posicion medida x, y, z (mm).
 * @return Error e de esta llegada (mm).
 */
float Metricas_Agregar(metricas_t *m, const float cmd[METRICAS_EJES], const float med[METRICAS_EJES]);

/**
 * @brief Calcula el resumen. Con menos de dos llegadas, el desvio y la repetibilidad dan 0.
 */
void Metricas_Resultado(const metricas_t *m, metricas_res_t *r);

/** @brief Deja el acumulador del ajuste vacio. */
void Recta_Iniciar(recta_t *r);

/** @brief Agrega un punto al ajuste. */
void Recta_Agregar(recta_t *r, float x, float y);

/**
 * @brief Ajuste por minimos cuadrados.
 * @return false si no hay al menos dos valores distintos de x.
 */
bool Recta_Ajustar(const recta_t *r, float *pendiente, float *ordenada);

#endif /* METRICAS_H */
