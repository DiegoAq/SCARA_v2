/**
 * @file test_mapa.c
 * @brief Test en PC del mapa de alturas, de la deteccion de cajas y de la geometria de la garra.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "cinematica.h"
#include "config.h"
#include "mapa.h"

#define MESA_MM		150.0f

static bool cerca(float a, float b, float tol)
{
	return fabsf(a - b) <= tol;
}

/** Diferencia entre dos direcciones sin sentido: una caja girada 180 grados es la misma caja. */
static float diferencia_de_eje(float a, float b)
{
	float d = fabsf(Cinematica_NormalizarDeg(a - b));

	return (d > 90.0f) ? 180.0f - d : d;
}

/** Carga el area con una lectura por mm: la mesa, salvo adentro de un rectangulo girado. */
static void cargar_rectangulo(float x0, float y0, float ancho, float alto,
							  float cx, float cy, float yaw, float largo, float ancho_caja, float dist)
{
	float ux, uy;

	Cinematica_Direccion(yaw, &ux, &uy);
	for (float y = y0; y <= y0 + alto; y += 1.0f) {
		for (float x = x0; x <= x0 + ancho; x += 1.0f) {
			float dx = x - cx;
			float dy = y - cy;
			bool adentro = fabsf(dx * ux + dy * uy) <= 0.5f * largo
						&& fabsf(-dx * uy + dy * ux) <= 0.5f * ancho_caja;

			Mapa_Agregar(x, y, adentro ? dist : MESA_MM);
		}
	}
}

static void test_grilla(void)
{
	float d;

	/* Una fila y una columna sobre cada borde. */
	assert(Mapa_Iniciar(100.0f, 200.0f, 150.0f, 60.0f));
	assert(Mapa_Columnas() == 11);
	assert(Mapa_Filas() == 5);

	/* Una celda vacia no tiene valor, ni la que no existe. */
	assert(!Mapa_Celda(0, 0, &d));
	assert(!Mapa_Celda(-1, 0, &d));
	assert(!Mapa_Celda(0, 11, &d));
	assert(!Mapa_Celda(5, 0, &d));

	/* Cada lectura cae en la celda mas cercana; las de afuera se ignoran. */
	Mapa_Agregar(100.0f, 200.0f, 80.0f);
	Mapa_Agregar(107.0f, 207.0f, 120.0f);
	assert(Mapa_Celda(0, 0, &d) && cerca(d, 100.0f, 1.0f));
	Mapa_Agregar(250.0f, 260.0f, 50.0f);
	assert(Mapa_Celda(4, 10, &d) && cerca(d, 50.0f, 0.5f));
	Mapa_Agregar(90.0f, 200.0f, 10.0f);
	Mapa_Agregar(100.0f, 280.0f, 10.0f);
	Mapa_Agregar(1000.0f, 200.0f, 10.0f);
	assert(!Mapa_Celda(1, 0, &d));

	/* Lo que no entra en la grilla, o no tiene superficie, se rechaza y no queda mapa. */
	assert(!Mapa_Iniciar(0.0f, 0.0f, MAPA_CELDA_MM * (float)MAPA_COLUMNAS_MAX, 10.0f));
	assert(Mapa_Filas() == 0 && Mapa_Columnas() == 0);
	assert(!Mapa_Iniciar(0.0f, 0.0f, 600.0f, 900.0f));
	assert(!Mapa_Iniciar(0.0f, 0.0f, 0.0f, 100.0f));
	assert(!Mapa_Iniciar(0.0f, 0.0f, 100.0f, -1.0f));
	assert(!Mapa_Celda(0, 0, &d));
	printf("OK  grilla: celdas, bordes y areas que no entran\n");
}

static void test_mesa_y_manchas(void)
{
	mapa_mancha_t m;
	int cant;
	float mesa;

	/* Vacio: no hay mesa ni mancha. */
	assert(Mapa_Iniciar(0.0f, 0.0f, 300.0f, 300.0f));
	assert(!Mapa_Analizar(&m, &cant, &mesa));
	assert(cant == 0 && mesa == 0.0f);

	/* Solo mesa. */
	cargar_rectangulo(0.0f, 0.0f, 300.0f, 300.0f, -500.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f);
	assert(!Mapa_Analizar(&m, &cant, &mesa));
	assert(cant == 0 && cerca(mesa, MESA_MM, (float)MAPA_HIST_MM));

	/* Una caja, en varias orientaciones. */
	static const float yaws[] = { 0.0f, 20.0f, 45.0f, 90.0f, -30.0f, -60.0f };

	for (unsigned i = 0; i < sizeof(yaws) / sizeof(yaws[0]); i++) {
		assert(Mapa_Iniciar(0.0f, 0.0f, 300.0f, 300.0f));
		cargar_rectangulo(0.0f, 0.0f, 300.0f, 300.0f, 140.0f, 160.0f, yaws[i],
						  CAJA_LARGO_MM, CAJA_ANCHO_MM, 50.0f);
		assert(Mapa_Analizar(&m, &cant, &mesa));
		assert(cant == 1);
		assert(cerca(mesa, MESA_MM, (float)MAPA_HIST_MM));
		assert(cerca(m.x_mm, 140.0f, 8.0f) && cerca(m.y_mm, 160.0f, 8.0f));
		assert(diferencia_de_eje(m.yaw_deg, yaws[i]) <= 12.0f);
		assert(cerca(m.largo_mm, CAJA_LARGO_MM, 2.0f * MAPA_CELDA_MM));
		assert(m.ancho_mm < m.largo_mm);
	}

	/* Una caja que ocupa buena parte del area no se confunde con la mesa. */
	assert(Mapa_Iniciar(0.0f, 0.0f, 120.0f, 90.0f));
	cargar_rectangulo(0.0f, 0.0f, 120.0f, 90.0f, 60.0f, 45.0f, 0.0f, CAJA_LARGO_MM, CAJA_ANCHO_MM, 50.0f);
	assert(Mapa_Analizar(&m, &cant, &mesa));
	assert(cerca(mesa, MESA_MM, (float)MAPA_HIST_MM));

	/* Lo que no alcanza el umbral de altura es mesa, y lo que no alcanza la superficie es ruido. */
	assert(Mapa_Iniciar(0.0f, 0.0f, 300.0f, 300.0f));
	cargar_rectangulo(0.0f, 0.0f, 300.0f, 300.0f, 140.0f, 160.0f, 0.0f,
					  CAJA_LARGO_MM, CAJA_ANCHO_MM, MESA_MM - 0.5f * CAJA_UMBRAL_MM);
	assert(!Mapa_Analizar(&m, &cant, &mesa));
	assert(Mapa_Iniciar(0.0f, 0.0f, 300.0f, 300.0f));
	cargar_rectangulo(0.0f, 0.0f, 300.0f, 300.0f, 140.0f, 160.0f, 0.0f, 20.0f, 20.0f, 50.0f);
	assert(!Mapa_Analizar(&m, &cant, &mesa));
	assert(cant == 0);

	/* Dos cajas: cuenta las dos y describe la mayor. */
	assert(Mapa_Iniciar(0.0f, 0.0f, 300.0f, 300.0f));
	for (float y = 0.0f; y <= 300.0f; y += 1.0f) {
		for (float x = 0.0f; x <= 300.0f; x += 1.0f) {
			bool chica = fabsf(x - 60.0f) <= 40.0f && fabsf(y - 60.0f) <= 25.0f;
			bool grande = fabsf(x - 200.0f) <= 60.0f && fabsf(y - 220.0f) <= 25.0f;

			Mapa_Agregar(x, y, (chica || grande) ? 50.0f : MESA_MM);
		}
	}
	assert(Mapa_Analizar(&m, &cant, &mesa));
	assert(cant == 2);
	assert(cerca(m.x_mm, 200.0f, 8.0f) && cerca(m.y_mm, 220.0f, 8.0f));
	printf("OK  mesa y manchas: orientacion, umbrales y varias cajas\n");
}

static void test_cruces(void)
{
	mapa_muestra_t m[60];
	mapa_cruces_t c;

	/* De x = 0 a x = 59: sube a la caja entre 20 y 40. */
	for (int i = 0; i < 60; i++) {
		m[i].x_mm = (float)i;
		m[i].y_mm = 5.0f;
		m[i].dist_mm = (i >= 20 && i < 40) ? 50.0f : MESA_MM;
	}
	assert(Mapa_Cruces(m, 60U, MESA_MM, &c));
	assert(c.hay_entrada && c.hay_salida);
	assert(cerca(c.xe_mm, 19.5f, 1.0f) && cerca(c.xs_mm, 39.5f, 1.0f));
	assert(cerca(c.ye_mm, 5.0f, 0.01f) && cerca(c.ys_mm, 5.0f, 0.01f));

	/* El punto medio no depende de cuanto el cono suavice el escalon, si lo hace parejo. */
	for (int i = 0; i < 60; i++) {
		float s = 50.0f + (MESA_MM - 50.0f) * fabsf((float)i - 29.5f) / 29.5f;

		m[i].dist_mm = s;
	}
	assert(Mapa_Cruces(m, 60U, MESA_MM, &c));
	assert(cerca(0.5f * (c.xe_mm + c.xs_mm), 29.5f, 1.0f));

	/* Una lectura suelta no cuenta como borde. */
	for (int i = 0; i < 60; i++) {
		m[i].dist_mm = (i >= 20 && i < 40) ? 50.0f : MESA_MM;
	}
	m[10].dist_mm = 50.0f;
	m[30].dist_mm = MESA_MM;
	assert(Mapa_Cruces(m, 60U, MESA_MM, &c));
	assert(cerca(c.xe_mm, 19.5f, 1.0f) && cerca(c.xs_mm, 39.5f, 1.0f));

	/* Empieza arriba de la caja: hay salida pero no entrada. */
	for (int i = 0; i < 60; i++) {
		m[i].dist_mm = (i < 40) ? 50.0f : MESA_MM;
	}
	assert(Mapa_Cruces(m, 60U, MESA_MM, &c));
	assert(!c.hay_entrada && c.hay_salida);

	/* Termina arriba de la caja: hay entrada pero no salida. */
	for (int i = 0; i < 60; i++) {
		m[i].dist_mm = (i >= 20) ? 50.0f : MESA_MM;
	}
	assert(Mapa_Cruces(m, 60U, MESA_MM, &c));
	assert(c.hay_entrada && !c.hay_salida);

	/* Solo mesa, o muy pocas lecturas. */
	for (int i = 0; i < 60; i++) {
		m[i].dist_mm = MESA_MM;
	}
	assert(!Mapa_Cruces(m, 60U, MESA_MM, &c));
	assert(!Mapa_Cruces(m, 2U, MESA_MM, &c));
	printf("OK  cruces de una pasada: bordes, lecturas sueltas y pasadas incompletas\n");
}

static void test_tipo(void)
{
	static const float altos[CAJA_CANT_TIPOS] = CAJA_ALTOS_MM;

	for (int t = 0; t < CAJA_CANT_TIPOS; t++) {
		assert(Mapa_TipoDeCaja(altos[t]) == t + 1);
		assert(Mapa_TipoDeCaja(altos[t] + 0.9f * CAJA_TOL_ALTO_MM) == t + 1);
		assert(Mapa_TipoDeCaja(altos[t] - 0.9f * CAJA_TOL_ALTO_MM) == t + 1);
	}
	assert(Mapa_TipoDeCaja(altos[0] - 2.0f * CAJA_TOL_ALTO_MM) == 0);
	assert(Mapa_TipoDeCaja(altos[CAJA_CANT_TIPOS - 1] + 2.0f * CAJA_TOL_ALTO_MM) == 0);
	printf("OK  tipo de caja segun su altura\n");
}

static void test_geometria_garra(void)
{
	float x, y, ex, ey, rot;

	/* Brazo recto sobre +Y: la linea de sensores queda a 90 grados, con el A del lado de +Y. */
	assert(cerca(Cinematica_YawGarra(90.0f, 0.0f, 0.0f), 90.0f + GARRA_YAW_CERO_DEG, 0.01f));
	(void)Cinematica_Directa(90.0f, 0.0f, &ex, &ey);
	Cinematica_PuntoSensor(90.0f, 0.0f, 0.0f, 1.0f, &x, &y);
	assert(cerca(x, ex, 0.01f) && cerca(y, ey + 0.5f * SENSOR_SEPARACION_MM, 0.01f));
	Cinematica_PuntoSensor(90.0f, 0.0f, 0.0f, -1.0f, &x, &y);
	assert(cerca(x, ex, 0.01f) && cerca(y, ey - 0.5f * SENSOR_SEPARACION_MM, 0.01f));

	/* Con rot = -90, el A pasa a +X. */
	Cinematica_PuntoSensor(90.0f, 0.0f, -90.0f, 1.0f, &x, &y);
	assert(cerca(x, ex + 0.5f * SENSOR_SEPARACION_MM, 0.01f) && cerca(y, ey, 0.01f));

	/* Las juntas suman, y la vuelta se normaliza. */
	assert(cerca(Cinematica_YawGarra(170.0f, 100.0f, 0.0f), -90.0f + GARRA_YAW_CERO_DEG, 0.01f));

	/* La orientacion deja la garra en la direccion pedida, con cualquier postura. */
	static const float posturas[][2] = { { 90.0f, 0.0f }, { 60.0f, 45.0f }, { 120.0f, -30.0f } };
	static const float yaws[] = { 0.0f, 45.0f, 90.0f, -30.0f, 179.0f };

	for (unsigned p = 0; p < sizeof(posturas) / sizeof(posturas[0]); p++) {
		for (unsigned i = 0; i < sizeof(yaws) / sizeof(yaws[0]); i++) {
			assert(Cinematica_RotParaYaw(posturas[p][0], posturas[p][1], yaws[i], 0.0f, &rot));
			assert(diferencia_de_eje(Cinematica_YawGarra(posturas[p][0], posturas[p][1], rot), yaws[i]) < 0.01f);
			assert(rot >= ROT_MIN_DEG && rot <= ROT_MAX_DEG);
		}
	}

	/* De las dos soluciones equivalentes, la mas cercana a donde esta. */
	assert(Cinematica_RotParaYaw(90.0f, 0.0f, 0.0f, 0.0f, &rot) && cerca(rot, -90.0f, 0.01f));
	assert(Cinematica_RotParaYaw(90.0f, 0.0f, 0.0f, 120.0f, &rot) && cerca(rot, 90.0f, 0.01f));

	/* Direccion y atan2, ida y vuelta. */
	float ux, uy;

	Cinematica_Direccion(30.0f, &ux, &uy);
	assert(cerca(Cinematica_Atan2Deg(uy, ux), 30.0f, 0.01f));
	assert(cerca(ux * ux + uy * uy, 1.0f, 1e-4f));
	printf("OK  geometria de la garra: sensores, direccion en la mesa y orientacion\n");
}

int main(void)
{
	printf("== test_mapa ==\n");
	test_grilla();
	test_mesa_y_manchas();
	test_cruces();
	test_tipo();
	test_geometria_garra();
	printf("test_mapa: TODO OK\n");
	return 0;
}
