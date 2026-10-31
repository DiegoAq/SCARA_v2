/**
 * @file mapa.c
 * @brief Mapa de alturas de un area y deteccion de cajas (nucleo portable, sin hardware).
 */
#include "mapa.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "cinematica.h"
#include "config.h"

_Static_assert(MAPA_CELDAS_MAX <= UINT16_MAX, "la pila de la busqueda guarda indices de 16 bits");

#define LECTURAS_SATURADAS	255U	/* a partir de aca el promedio sigue a las lecturas nuevas */
#define MARCA_LIBRE			0U
#define MARCA_OCUPADA		1U
#define MARCA_PRIMERA		2U		/* la primera mancha */
#define MARCA_ULTIMA		255U	/* las manchas de mas comparten esta marca */

/* La grilla entera y sus auxiliares son estaticos: el mapa no entra en la pila y el proyecto no
 * usa malloc. */
static uint16_t distancia[MAPA_CELDAS_MAX];
static uint8_t lecturas[MAPA_CELDAS_MAX];
static uint8_t marca[MAPA_CELDAS_MAX];
static uint16_t pendientes[MAPA_CELDAS_MAX];
static uint16_t histograma[MAPA_HIST_BINS];

static float x0_mm;
static float y0_mm;
static int filas;
static int columnas;

/** Acumula los momentos de una mancha, sobre coordenadas relativas al origen del area. */
typedef struct {
	uint32_t n;
	float sx, sy, sxx, syy, sxy;
} momentos_t;

static int indice(int fila, int columna)
{
	return fila * columnas + columna;
}

/** Cantidad de filas o columnas que cubren un lado, con una en cada borde. */
static int celdas_de_lado(float lado_mm)
{
	return (int)ceilf(lado_mm / MAPA_CELDA_MM) + 1;
}

bool Mapa_Iniciar(float x_mm, float y_mm, float ancho_mm, float alto_mm)
{
	filas = 0;
	columnas = 0;
	if (!(ancho_mm > 0.0f && alto_mm > 0.0f)
			|| ancho_mm > MAPA_CELDA_MM * (float)MAPA_COLUMNAS_MAX
			|| alto_mm > MAPA_CELDA_MM * (float)MAPA_CELDAS_MAX) {
		return false;
	}

	int c = celdas_de_lado(ancho_mm);
	int f = celdas_de_lado(alto_mm);

	if (c > MAPA_COLUMNAS_MAX || f * c > MAPA_CELDAS_MAX) {
		return false;
	}
	x0_mm = x_mm;
	y0_mm = y_mm;
	filas = f;
	columnas = c;
	memset(lecturas, 0, sizeof(lecturas));
	return true;
}

void Mapa_Agregar(float x_mm, float y_mm, float dist_mm)
{
	/* Sumar media celda deja cada celda centrada sobre su fila del barrido. */
	float fx = floorf((x_mm - x0_mm) / MAPA_CELDA_MM + 0.5f);
	float fy = floorf((y_mm - y0_mm) / MAPA_CELDA_MM + 0.5f);

	if (!(fx >= 0.0f && fy >= 0.0f && fx < (float)columnas && fy < (float)filas)) {
		return;
	}

	int i = indice((int)fy, (int)fx);
	float d = (dist_mm < 0.0f) ? 0.0f : dist_mm;

	if (lecturas[i] == 0U) {
		distancia[i] = (uint16_t)(d + 0.5f);
		lecturas[i] = 1U;
		return;
	}

	float previa = (float)distancia[i];

	distancia[i] = (uint16_t)(previa + (d - previa) / (float)(lecturas[i] + 1U) + 0.5f);
	if (lecturas[i] < LECTURAS_SATURADAS) {
		lecturas[i]++;
	}
}

/** Percentil MAPA_PERCENTIL_MESA de las celdas con lecturas. @return false si no hay ninguna. */
static bool estimar_mesa(float *mesa_mm)
{
	uint32_t total = 0U;
	int celdas = filas * columnas;

	memset(histograma, 0, sizeof(histograma));
	for (int i = 0; i < celdas; i++) {
		if (lecturas[i] != 0U) {
			uint32_t bin = (uint32_t)distancia[i] / MAPA_HIST_MM;

			histograma[(bin < MAPA_HIST_BINS) ? bin : (MAPA_HIST_BINS - 1U)]++;
			total++;
		}
	}
	if (total == 0U) {
		return false;
	}

	uint32_t objetivo = (total * MAPA_PERCENTIL_MESA + 99U) / 100U;
	uint32_t acumulado = 0U;

	for (uint32_t b = 0U; b < MAPA_HIST_BINS; b++) {
		acumulado += histograma[b];
		if (acumulado >= objetivo) {
			*mesa_mm = ((float)b + 0.5f) * (float)MAPA_HIST_MM;
			return true;
		}
	}
	return false;
}

static void agregar_a_momentos(momentos_t *m, int fila, int columna)
{
	float x = (float)columna * MAPA_CELDA_MM;
	float y = (float)fila * MAPA_CELDA_MM;

	m->n++;
	m->sx += x;
	m->sy += y;
	m->sxx += x * x;
	m->syy += y * y;
	m->sxy += x * y;
}

/**
 * Marca toda la mancha que contiene a una celda y acumula sus momentos. Cuenta las diagonales:
 * las pasadas no son paralelas al borde de la caja y una mancha oblicua se toca por las esquinas.
 */
static void recorrer_mancha(int inicio, uint8_t etiqueta, momentos_t *m)
{
	uint32_t n = 0U;

	marca[inicio] = etiqueta;
	pendientes[n++] = (uint16_t)inicio;
	*m = (momentos_t){ 0U, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

	while (n > 0U) {
		int i = pendientes[--n];
		int fila = i / columnas;
		int columna = i % columnas;

		agregar_a_momentos(m, fila, columna);

		for (int df = -1; df <= 1; df++) {
			for (int dc = -1; dc <= 1; dc++) {
				int f = fila + df;
				int c = columna + dc;

				if (f < 0 || c < 0 || f >= filas || c >= columnas) {
					continue;
				}

				int v = indice(f, c);

				if (marca[v] == MARCA_OCUPADA) {
					marca[v] = etiqueta;
					pendientes[n++] = (uint16_t)v;
				}
			}
		}
	}
}

/** Centro, direccion principal y extension de una mancha ya recorrida. */
static void describir_mancha(const momentos_t *m, uint8_t etiqueta, mapa_mancha_t *mancha)
{
	float n = (float)m->n;
	float mx = m->sx / n;
	float my = m->sy / n;
	float cxx = m->sxx / n - mx * mx;
	float cyy = m->syy / n - my * my;
	float cxy = m->sxy / n - mx * my;

	/* El eje mayor de la elipse de inercia. Una caja de 3:1 lo marca sin ambiguedad, incluso
	 * agrandada por el cono del sensor. */
	float yaw = 0.5f * Cinematica_Atan2Deg(2.0f * cxy, cxx - cyy);
	float ux, uy;

	Cinematica_Direccion(yaw, &ux, &uy);

	float min_u = 0.0f, max_u = 0.0f, min_v = 0.0f, max_v = 0.0f;
	int celdas = filas * columnas;

	for (int i = 0; i < celdas; i++) {
		if (marca[i] == etiqueta) {
			float dx = (float)(i % columnas) * MAPA_CELDA_MM - mx;
			float dy = (float)(i / columnas) * MAPA_CELDA_MM - my;
			float u = dx * ux + dy * uy;
			float v = -dx * uy + dy * ux;

			min_u = (u < min_u) ? u : min_u;
			max_u = (u > max_u) ? u : max_u;
			min_v = (v < min_v) ? v : min_v;
			max_v = (v > max_v) ? v : max_v;
		}
	}

	mancha->x_mm = x0_mm + mx;
	mancha->y_mm = y0_mm + my;
	mancha->yaw_deg = yaw;
	mancha->largo_mm = max_u - min_u + MAPA_CELDA_MM;
	mancha->ancho_mm = max_v - min_v + MAPA_CELDA_MM;
	mancha->area_mm2 = n * MAPA_CELDA_MM * MAPA_CELDA_MM;
}

bool Mapa_Analizar(mapa_mancha_t *mancha, int *cantidad, float *mesa_mm)
{
	int celdas = filas * columnas;

	*cantidad = 0;
	*mesa_mm = 0.0f;
	if (mancha == NULL || !estimar_mesa(mesa_mm)) {
		return false;
	}

	for (int i = 0; i < celdas; i++) {
		bool alta = lecturas[i] != 0U && *mesa_mm - (float)distancia[i] >= CAJA_UMBRAL_MM;

		marca[i] = alta ? MARCA_OCUPADA : MARCA_LIBRE;
	}

	uint32_t etiqueta = MARCA_PRIMERA;
	float mayor_area = 0.0f;

	for (int i = 0; i < celdas; i++) {
		if (marca[i] != MARCA_OCUPADA) {
			continue;
		}

		momentos_t m;
		uint8_t e = (uint8_t)((etiqueta < MARCA_ULTIMA) ? etiqueta : MARCA_ULTIMA);

		recorrer_mancha(i, e, &m);
		etiqueta++;

		float area = (float)m.n * MAPA_CELDA_MM * MAPA_CELDA_MM;

		if (area >= CAJA_MANCHA_MIN_MM2) {
			(*cantidad)++;
			if (area > mayor_area) {
				mayor_area = area;
				describir_mancha(&m, e, mancha);
			}
		}
	}
	return *cantidad > 0;
}

int Mapa_Filas(void)
{
	return filas;
}

int Mapa_Columnas(void)
{
	return columnas;
}

bool Mapa_Celda(int fila, int columna, float *dist_mm)
{
	if (dist_mm == NULL || fila < 0 || columna < 0 || fila >= filas || columna >= columnas) {
		return false;
	}

	int i = indice(fila, columna);

	if (lecturas[i] == 0U) {
		return false;
	}
	*dist_mm = (float)distancia[i];
	return true;
}

/** Mediana de tres: saca una lectura suelta sin correr los bordes, cosa que un promedio haria. */
static float filtrada(const mapa_muestra_t *m, uint32_t n, uint32_t i)
{
	if (i == 0U || i + 1U >= n) {
		return m[i].dist_mm;
	}

	float a = m[i - 1U].dist_mm;
	float b = m[i].dist_mm;
	float c = m[i + 1U].dist_mm;
	float mayor = (a > b) ? a : b;
	float menor = (a > b) ? b : a;

	if (c >= mayor) {
		return mayor;
	}
	return (c <= menor) ? menor : c;
}

/** Punto donde la lectura pasa por el umbral entre dos muestras seguidas. */
static void interpolar(const mapa_muestra_t *m, uint32_t i, float d0, float d1, float umbral,
					   float *x_mm, float *y_mm)
{
	float t = (d0 - umbral) / (d0 - d1);

	*x_mm = m[i - 1U].x_mm + t * (m[i].x_mm - m[i - 1U].x_mm);
	*y_mm = m[i - 1U].y_mm + t * (m[i].y_mm - m[i - 1U].y_mm);
}

bool Mapa_Cruces(const mapa_muestra_t *m, uint32_t n, float mesa_mm, mapa_cruces_t *c)
{
	if (m == NULL || c == NULL) {
		return false;
	}
	*c = (mapa_cruces_t){ .hay_entrada = false, .hay_salida = false };
	if (n < 3U) {
		return false;
	}

	float tope = filtrada(m, n, 0U);

	for (uint32_t i = 1U; i < n; i++) {
		float d = filtrada(m, n, i);

		tope = (d < tope) ? d : tope;
	}
	if (mesa_mm - tope < CAJA_UMBRAL_MM) {
		return false;
	}

	float umbral = 0.5f * (mesa_mm + tope);

	for (uint32_t i = 1U; i < n; i++) {
		float d0 = filtrada(m, n, i - 1U);
		float d1 = filtrada(m, n, i);

		if (d0 >= umbral && d1 < umbral) {
			interpolar(m, i, d0, d1, umbral, &c->xe_mm, &c->ye_mm);
			c->hay_entrada = true;
			break;
		}
	}
	for (uint32_t i = n - 1U; i >= 1U; i--) {
		float d0 = filtrada(m, n, i - 1U);
		float d1 = filtrada(m, n, i);

		if (d0 < umbral && d1 >= umbral) {
			interpolar(m, i, d0, d1, umbral, &c->xs_mm, &c->ys_mm);
			c->hay_salida = true;
			break;
		}
	}
	return true;
}

int Mapa_TipoDeCaja(float alto_mm)
{
	static const float altos[CAJA_CANT_TIPOS] = CAJA_ALTOS_MM;

	for (int t = 0; t < CAJA_CANT_TIPOS; t++) {
		if (fabsf(alto_mm - altos[t]) <= CAJA_TOL_ALTO_MM) {
			return t + 1;
		}
	}
	return 0;
}
