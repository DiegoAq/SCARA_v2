/**
 * @file busqueda.c
 * @brief Busqueda de la caja: del tick de los sensores al mapa, los bordes y la medida.
 */
#include "busqueda.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "cinematica.h"
#include "config.h"
#include "eje_z.h"
#include "laseres.h"
#include "motion.h"

/** A donde van las lecturas. */
typedef enum {
	DESTINO_NINGUNO = 0,
	DESTINO_MAPA,
	DESTINO_PASADA
} destino_t;

/** Suma de puntos, para promediar varias pasadas. */
typedef struct {
	uint32_t n;
	float x, y;
} suma_punto_t;

static destino_t destino;
static float q1_previa;
static float q2_previa;

/* Una pasada */
static mapa_muestra_t muestras[VL53_CANT][CENTRADO_MUESTRAS_MAX];
static uint32_t cant_muestras[VL53_CANT];

/* Pasadas a lo ancho */
static suma_punto_t medio_ancho[VL53_CANT];
static float suma_ancho;
static uint32_t cant_anchos;

/* Pasadas a lo largo */
static suma_punto_t extremo_lejano;
static suma_punto_t extremo_cercano;

/* Mesa */
static bool hay_mesa;
static float z_mesa_mm;
static float mesa_mapa_mm;

/* Caja */
static bool hay_caja;
static busqueda_caja_t caja_actual;

static float distancia(float x0, float y0, float x1, float y1)
{
	return sqrtf((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
}

static void sumar_punto(suma_punto_t *s, float x, float y)
{
	s->n++;
	s->x += x;
	s->y += y;
}

static void reiniciar_pasadas(void)
{
	for (int i = 0; i < VL53_CANT; i++) {
		medio_ancho[i] = (suma_punto_t){ 0U, 0.0f, 0.0f };
	}
	extremo_lejano = (suma_punto_t){ 0U, 0.0f, 0.0f };
	extremo_cercano = (suma_punto_t){ 0U, 0.0f, 0.0f };
	suma_ancho = 0.0f;
	cant_anchos = 0U;
}

void Busqueda_Init(void)
{
	Laseres_Init();
	destino = DESTINO_NINGUNO;
	hay_mesa = false;
	hay_caja = false;
	reiniciar_pasadas();
}

/** Toma la postura de partida: la primera lectura empezo a medirse aca. */
static bool arrancar(destino_t d)
{
	Motion_PosturaComandada(&q1_previa, &q2_previa);
	cant_muestras[VL53_A] = 0U;
	cant_muestras[VL53_B] = 0U;
	destino = d;
	return Laseres_Arrancar();
}

bool Busqueda_ArrancarMapa(void)
{
	hay_mesa = false;
	hay_caja = false;
	reiniciar_pasadas();
	return arrancar(DESTINO_MAPA);
}

bool Busqueda_ArrancarPasada(void)
{
	return arrancar(DESTINO_PASADA);
}

void Busqueda_DetenerMuestreo(void)
{
	destino = DESTINO_NINGUNO;
	Laseres_Detener();
}

bool Busqueda_Muestreando(void)
{
	return destino != DESTINO_NINGUNO;
}

busqueda_res_t Busqueda_Tick(void)
{
	laser_lectura_t l;

	switch (Laseres_Tick(&l)) {
	case LASER_NADA:
		return BUSQUEDA_OK;
	case LASER_FALLA:
		return BUSQUEDA_ERR_SENSOR;
	default:
		break;
	}

	/* La medicion llevo desde la lectura anterior hasta ahora: el punto que le corresponde es el
	 * del medio. Sin esto, cada pasada veria los bordes corridos en su sentido de marcha. */
	float q1, q2, x, y;

	Motion_PosturaComandada(&q1, &q2);

	float q1_media = 0.5f * (q1 + q1_previa);
	float q2_media = 0.5f * (q2 + q2_previa);

	q1_previa = q1;
	q2_previa = q2;

	if (!l.con_retorno) {
		return BUSQUEDA_OK;
	}
	if (l.mm < BARRIDO_DIST_MIN_MM) {
		return BUSQUEDA_ERR_MUY_BAJO;
	}

	/* La orientacion va medida: el motor se queda a un pulso del comando, sin repetir de una vez a
	 * la otra. Un grado a media separacion de los sensores ya corre el borde mas de medio mm. */
	Cinematica_PuntoSensor(q1_media, q2_media, EjeZ_Orientacion(),
						   (l.sensor == VL53_A) ? 1.0f : -1.0f, &x, &y);

	if (destino == DESTINO_MAPA) {
		Mapa_Agregar(x, y, l.mm);
	} else if (cant_muestras[l.sensor] < CENTRADO_MUESTRAS_MAX) {
		mapa_muestra_t *m = &muestras[l.sensor][cant_muestras[l.sensor]];

		m->x_mm = x;
		m->y_mm = y;
		m->dist_mm = l.mm;
		cant_muestras[l.sensor]++;
	}
	return BUSQUEDA_OK;
}

bool Busqueda_AnalizarMapa(float z_mm, mapa_mancha_t *mancha, int *cantidad, float *mesa_mm)
{
	bool hay = Mapa_Analizar(mancha, cantidad, mesa_mm);

	if (*mesa_mm > 0.0f) {
		hay_mesa = true;
		mesa_mapa_mm = *mesa_mm;
		z_mesa_mm = z_mm - *mesa_mm;
	}
	return hay;
}

bool Busqueda_MesaDelMapa(float *mesa_mm)
{
	if (!hay_mesa || mesa_mm == NULL) {
		return false;
	}
	*mesa_mm = mesa_mapa_mm;
	return true;
}

bool Busqueda_AlturaSobreMesa(float z_mm, float dist_mm, float *alto_mm)
{
	if (!hay_mesa || alto_mm == NULL) {
		return false;
	}
	*alto_mm = z_mm - dist_mm - z_mesa_mm;
	return true;
}

void Busqueda_OlvidarMesa(void)
{
	hay_mesa = false;
}

/** Los cruces de una pasada para un sensor. */
static bool cruces_de(vl53_id_t id, mapa_cruces_t *c)
{
	return hay_mesa && Mapa_Cruces(muestras[id], cant_muestras[id], mesa_mapa_mm, c);
}

bool Busqueda_CerrarPasadaAncho(void)
{
	mapa_cruces_t c[VL53_CANT];

	for (int i = 0; i < VL53_CANT; i++) {
		if (!cruces_de((vl53_id_t)i, &c[i]) || !c[i].hay_entrada || !c[i].hay_salida) {
			return false;
		}
	}
	for (int i = 0; i < VL53_CANT; i++) {
		sumar_punto(&medio_ancho[i], 0.5f * (c[i].xe_mm + c[i].xs_mm), 0.5f * (c[i].ye_mm + c[i].ys_mm));
		suma_ancho += distancia(c[i].xe_mm, c[i].ye_mm, c[i].xs_mm, c[i].ys_mm);
		cant_anchos++;
	}
	return true;
}

bool Busqueda_ResolverAncho(float *x_mm, float *y_mm, float *yaw_deg, float *ancho_mm)
{
	if (cant_anchos == 0U || medio_ancho[VL53_A].n == 0U || medio_ancho[VL53_B].n == 0U) {
		return false;
	}

	float xa = medio_ancho[VL53_A].x / (float)medio_ancho[VL53_A].n;
	float ya = medio_ancho[VL53_A].y / (float)medio_ancho[VL53_A].n;
	float xb = medio_ancho[VL53_B].x / (float)medio_ancho[VL53_B].n;
	float yb = medio_ancho[VL53_B].y / (float)medio_ancho[VL53_B].n;

	*x_mm = 0.5f * (xa + xb);
	*y_mm = 0.5f * (ya + yb);
	*yaw_deg = Cinematica_Atan2Deg(ya - yb, xa - xb);
	*ancho_mm = suma_ancho / (float)cant_anchos;
	return true;
}

bool Busqueda_CerrarPasadaLargo(bool ida)
{
	mapa_cruces_t a, b;

	if (!cruces_de(VL53_A, &a) || !cruces_de(VL53_B, &b)) {
		return false;
	}
	if (ida) {
		if (!a.hay_salida || !b.hay_entrada) {
			return false;
		}
		sumar_punto(&extremo_lejano, a.xs_mm, a.ys_mm);
		sumar_punto(&extremo_cercano, b.xe_mm, b.ye_mm);
	} else {
		if (!a.hay_entrada || !b.hay_salida) {
			return false;
		}
		sumar_punto(&extremo_lejano, a.xe_mm, a.ye_mm);
		sumar_punto(&extremo_cercano, b.xs_mm, b.ys_mm);
	}
	return true;
}

bool Busqueda_ResolverLargo(float *x_mm, float *y_mm, float yaw_deg, float *largo_mm)
{
	if (extremo_lejano.n == 0U || extremo_cercano.n == 0U) {
		return false;
	}

	float xl = extremo_lejano.x / (float)extremo_lejano.n;
	float yl = extremo_lejano.y / (float)extremo_lejano.n;
	float xc = extremo_cercano.x / (float)extremo_cercano.n;
	float yc = extremo_cercano.y / (float)extremo_cercano.n;
	float ux, uy;

	Cinematica_Direccion(yaw_deg, &ux, &uy);

	/* Solo se corrige a lo largo: a lo ancho, la mejor medida es la de las pasadas a lo ancho. */
	float avance = (0.5f * (xl + xc) - *x_mm) * ux + (0.5f * (yl + yc) - *y_mm) * uy;

	*x_mm += avance * ux;
	*y_mm += avance * uy;
	*largo_mm = (xl - xc) * ux + (yl - yc) * uy;
	return true;
}

void Busqueda_FijarCaja(const busqueda_caja_t *caja)
{
	caja_actual = *caja;
	hay_caja = true;
}

bool Busqueda_Caja(busqueda_caja_t *caja)
{
	if (!hay_caja || caja == NULL) {
		return false;
	}
	*caja = caja_actual;
	return true;
}

void Busqueda_OlvidarCaja(void)
{
	hay_caja = false;
}
