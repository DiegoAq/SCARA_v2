/**
 * @file mapa.h
 * @brief Mapa de alturas de un area y deteccion de cajas (nucleo portable, sin hardware).
 *
 * El barrido carga lecturas sueltas de los sensores en una grilla. Despues se estima a que
 * distancia esta la mesa, se juntan las celdas mas altas que la mesa en manchas y de la mayor se
 * sacan el centro y la direccion del lado largo. Es una ubicacion aproximada: la exacta sale de
 * Mapa_Cruces(), con las pasadas lentas sobre la caja.
 *
 * Unidades: mm y grados.
 */
#ifndef MAPA_H
#define MAPA_H

#include <stdbool.h>
#include <stdint.h>

/** Una mancha de celdas mas altas que la mesa. */
typedef struct {
	float x_mm, y_mm;			/**< Centro */
	float yaw_deg;				/**< Direccion del lado largo desde +X, en (-90, 90] */
	float largo_mm, ancho_mm;	/**< Extension aparente: incluye lo que agranda el cono del sensor */
	float area_mm2;				/**< Superficie de las celdas de la mancha */
} mapa_mancha_t;

/** Una lectura de un sensor durante una pasada. */
typedef struct {
	float x_mm, y_mm;	/**< Donde apuntaba el sensor */
	float dist_mm;		/**< Distancia medida */
} mapa_muestra_t;

/** Donde un sensor entro a la caja y donde salio durante una pasada. */
typedef struct {
	bool hay_entrada;		/**< La pasada empezo afuera y entro */
	float xe_mm, ye_mm;		/**< Primer cruce hacia adentro */
	bool hay_salida;		/**< La pasada termino afuera y salio */
	float xs_mm, ys_mm;		/**< Ultimo cruce hacia afuera */
} mapa_cruces_t;

/**
 * @brief Prepara una grilla vacia para un area alineada con los ejes del robot.
 *
 * Las filas y columnas quedan centradas sobre los bordes del area, separadas MAPA_CELDA_MM.
 * @param x_mm,y_mm Esquina con los menores X e Y (mm).
 * @param ancho_mm,alto_mm Lados en X y en Y (mm), positivos.
 * @return false si los lados no son positivos o el area no entra en la grilla. Si falla, no
 *         queda ningun mapa.
 */
bool Mapa_Iniciar(float x_mm, float y_mm, float ancho_mm, float alto_mm);

/**
 * @brief Agrega una lectura a la celda que le toca. Si cae fuera del area, se ignora.
 * @param x_mm,y_mm Donde apuntaba el sensor (mm).
 * @param dist_mm Distancia medida (mm).
 */
void Mapa_Agregar(float x_mm, float y_mm, float dist_mm);

/**
 * @brief Busca cajas en lo cargado hasta ahora.
 *
 * La mesa es el percentil MAPA_PERCENTIL_MESA de las distancias: no le molestan ni unas pocas
 * lecturas perdidas ni una caja que ocupe buena parte del area.
 * @param mancha Salida: la mancha de mayor superficie, valida si devuelve true.
 * @param cantidad Salida: cuantas manchas alcanzan CAJA_MANCHA_MIN_MM2.
 * @param mesa_mm Salida: distancia estimada a la mesa (mm), 0 si no hay lecturas.
 * @return true si hay al menos una mancha.
 */
bool Mapa_Analizar(mapa_mancha_t *mancha, int *cantidad, float *mesa_mm);

/** @brief Filas (en Y) de la grilla actual; 0 si no hay mapa. */
int Mapa_Filas(void);

/** @brief Columnas (en X) de la grilla actual; 0 si no hay mapa. */
int Mapa_Columnas(void);

/**
 * @brief Distancia promedio guardada en una celda.
 * @param fila,columna Celda.
 * @param dist_mm Salida (mm).
 * @return false si la celda no existe o no recibio lecturas.
 */
bool Mapa_Celda(int fila, int columna, float *dist_mm);

/**
 * @brief Busca donde un sensor cruzo los bordes de la caja durante una pasada.
 *
 * Cada borde se ubica donde la lectura pasa por la mitad entre la mesa y lo mas alto de la
 * pasada. Asi el cono del sensor corre los dos bordes por igual y no mueve el centro.
 * @param m Lecturas, en orden de tiempo.
 * @param n Cantidad de lecturas.
 * @param mesa_mm Distancia a la mesa (mm).
 * @param c Salida: los cruces encontrados.
 * @return false si el sensor nunca vio algo CAJA_UMBRAL_MM mas alto que la mesa.
 */
bool Mapa_Cruces(const mapa_muestra_t *m, uint32_t n, float mesa_mm, mapa_cruces_t *c);

/**
 * @brief Tipo de caja segun su altura.
 * @param alto_mm Altura medida (mm).
 * @return Indice + 1 dentro de CAJA_ALTOS_MM, o 0 si no se parece a ninguna.
 */
int Mapa_TipoDeCaja(float alto_mm);

#endif /* MAPA_H */
