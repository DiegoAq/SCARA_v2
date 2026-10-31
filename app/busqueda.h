/**
 * @file busqueda.h
 * @brief Busqueda de la caja: junta las lecturas de los sensores con la postura del brazo y calcula
 *        de ellas el mapa, los bordes de la caja y su medida.
 *
 * Quien decide adonde ir es Secuencia_Buscar(). De las juntas se usa lo COMANDADO y no lo medido
 * por los encoders: despues el brazo vuelve a la caja con ese mismo comando, asi que un error de
 * calibracion que se repita igual se cancela en vez de sumarse.
 */
#ifndef BUSQUEDA_H
#define BUSQUEDA_H

#include <stdbool.h>

#include "mapa.h"

/** La caja encontrada. */
typedef struct {
	float x_mm, y_mm;			/**< Centro */
	float yaw_deg;				/**< Direccion del lado largo en la mesa (grados) */
	float largo_mm, ancho_mm;	/**< Medidas segun las pasadas */
	float alto_mm;				/**< Altura de la tapa sobre la mesa */
	int tipo;					/**< 1 en adelante, segun CAJA_ALTOS_MM; 0 si no se parece a ninguna */
} busqueda_caja_t;

/** Resultado del muestreo en un tick. */
typedef enum {
	BUSQUEDA_OK = 0,
	BUSQUEDA_ERR_SENSOR,	/**< Un sensor no contesta */
	BUSQUEDA_ERR_MUY_BAJO	/**< Un sensor esta mas cerca que BARRIDO_DIST_MIN_MM */
} busqueda_res_t;

/** @brief Olvida el mapa, la mesa y la caja. */
void Busqueda_Init(void);

/** @brief Empieza a cargar las lecturas en el mapa, ya preparado con Mapa_Iniciar(). */
bool Busqueda_ArrancarMapa(void);

/** @brief Empieza a guardar las lecturas de una pasada lenta, desde cero. */
bool Busqueda_ArrancarPasada(void);

/** @brief Deja de muestrear. */
void Busqueda_DetenerMuestreo(void);

/** @brief true si hay un muestreo en curso. */
bool Busqueda_Muestreando(void);

/** @brief Consulta los sensores y guarda lo que llego. Llamar en cada tick mientras se muestrea. */
busqueda_res_t Busqueda_Tick(void);

/**
 * @brief Analiza el mapa y recuerda a que altura esta la mesa.
 * @param z_mm Altura de la pinza durante el barrido (mm).
 * @param mancha Salida: la mayor mancha.
 * @param cantidad Salida: cuantas manchas hay.
 * @param mesa_mm Salida: distancia de los sensores a la mesa durante el barrido (mm).
 * @return true si hay una mancha.
 */
bool Busqueda_AnalizarMapa(float z_mm, mapa_mancha_t *mancha, int *cantidad, float *mesa_mm);

/** @brief Distancia a la mesa del ultimo barrido, para mostrar el mapa como alturas. */
bool Busqueda_MesaDelMapa(float *mesa_mm);

/**
 * @brief Altura sobre la mesa de lo que ve un sensor.
 * @param z_mm Altura de la pinza (mm).
 * @param dist_mm Distancia medida (mm).
 * @param alto_mm Salida (mm).
 * @return false si todavia no se barrio ninguna mesa con este cero del eje Z.
 */
bool Busqueda_AlturaSobreMesa(float z_mm, float dist_mm, float *alto_mm);

/** @brief Olvida la altura de la mesa. Llamar cuando cambia el cero del eje Z. */
void Busqueda_OlvidarMesa(void);

/**
 * @brief Cierra una pasada a lo ancho de la caja y la suma a las anteriores.
 * @return false si algun sensor no cruzo los dos bordes.
 */
bool Busqueda_CerrarPasadaAncho(void);

/**
 * @brief Con las pasadas a lo ancho, calcula el eje de la caja.
 *
 * Cada sensor da un punto del eje: el medio entre sus dos bordes. La recta entre esos dos puntos
 * es el eje, asi que la orientacion sale de la medida y no de suponer que la garra estaba bien.
 * @param x_mm,y_mm Salida: punto del eje entre los dos sensores (mm).
 * @param yaw_deg Salida: direccion del eje, de B hacia A (grados).
 * @param ancho_mm Salida: ancho medido (mm).
 * @return false si no hay pasadas cerradas.
 */
bool Busqueda_ResolverAncho(float *x_mm, float *y_mm, float *yaw_deg, float *ancho_mm);

/**
 * @brief Cierra una pasada a lo largo de la caja y la suma a las anteriores.
 *
 * El A adelante cruza el extremo lejano y el B el cercano. No importa cuanto valga exactamente
 * la separacion entre sensores: corre los dos extremos en sentidos opuestos y el medio no se mueve.
 * @param ida true si la pinza se movio hacia el lado del sensor A.
 * @return false si algun sensor no cruzo su extremo.
 */
bool Busqueda_CerrarPasadaLargo(bool ida);

/**
 * @brief Con las pasadas a lo largo, calcula el centro sobre el eje.
 * @param x_mm,y_mm Entrada: punto del eje. Salida: centro de la caja (mm).
 * @param yaw_deg Direccion del eje (grados).
 * @param largo_mm Salida: largo medido (mm).
 * @return false si no hay pasadas cerradas.
 */
bool Busqueda_ResolverLargo(float *x_mm, float *y_mm, float yaw_deg, float *largo_mm);

/** @brief Guarda la caja encontrada, para que la use PICK. */
void Busqueda_FijarCaja(const busqueda_caja_t *caja);

/**
 * @brief La ultima caja encontrada.
 * @param caja Salida.
 * @return false si no hay ninguna vigente.
 */
bool Busqueda_Caja(busqueda_caja_t *caja);

/** @brief Olvida la caja: ya no esta donde se la encontro. */
void Busqueda_OlvidarCaja(void);

#endif /* BUSQUEDA_H */
