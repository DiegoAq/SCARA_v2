/**
 * @file test_busqueda.c
 * @brief Test en PC de SCAN, MAP y PICK sobre la caja encontrada, con una mesa simulada: los sensores
 *        miden desde la pose real del brazo, con el cono del VL53L0X, y no saben donde esta la caja.
 *
 * Se compila con el codo remontado (ver Makefile), que es lo que hace falta para que las pasadas
 * sobre la caja entren en el espacio de trabajo.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "busqueda.h"
#include "cinematica.h"
#include "config.h"
#include "console.h"
#include "eje_z.h"
#include "gripper.h"
#include "hw_sim.h"
#include "mapa.h"
#include "motion.h"
#include "secuencia.h"

#define CORRER_MUCHO	400000

#define AREA_X			(-90.0f)
#define AREA_Y			180.0f
#define AREA_ANCHO		180.0f
#define AREA_ALTO		150.0f
#define AREA_CMD		"scan -90 180 180 150\r"

/** Lo que se le exige al SCAN contra la caja real. */
#define TOL_XY_MM		3.0f
#define TOL_YAW_DEG		3.0f
/** Hasta donde llega la garra: cada motor se queda a un pulso, y un pulso es un grado de orientacion. */
#define TOL_ORIENT_DEG	(2.5f / PULSOS_POR_GRADO)

static float diferencia_de_eje(float a, float b)
{
	float d = fabsf(Cinematica_NormalizarDeg(a - b));

	return (d > 90.0f) ? 180.0f - d : d;
}

/** Ticks sueltos: correr() vuelve apenas todo queda quieto, y MAP imprime con el brazo quieto. */
static void pasar_ticks(int n)
{
	for (int i = 0; i < n; i++) {
		(void)correr(1);
	}
}

/** Arranque comun, con una mesa vacia. */
static void preparar(void)
{
	hw_sim_reiniciar();
	assert(strstr(tipear("home\r"), "OK servos habilitados") != NULL);
	assert(strstr(tipear("setz0\r"), "OK") != NULL);
	mundo_activo = true;
	mundo_z_mesa = -100.0f;
	borrar_salida();
}

static void poner_caja(float x, float y, float yaw, float alto)
{
	mundo_caja[mundo_cajas++] = (mundo_caja_t){ x, y, yaw, CAJA_LARGO_MM, CAJA_ANCHO_MM, alto };
}

/** Corre un SCAN de la zona habitual y devuelve lo que encontro. */
static bool escanear(busqueda_caja_t *caja)
{
	assert(strstr(tipear(AREA_CMD), "OK\r\n") != NULL);
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(!Secuencia_Activa());
	return Busqueda_Caja(caja);
}

/** Muestra por que fallo, antes de que el assert corte. */
static void informar(const char *caso, bool hay, const busqueda_caja_t *c, const mundo_caja_t *real)
{
	if (!hay || fabsf(c->x_mm - real->x) > TOL_XY_MM || fabsf(c->y_mm - real->y) > TOL_XY_MM
			|| diferencia_de_eje(c->yaw_deg, real->yaw) > TOL_YAW_DEG) {
		const char *p = strstr(salida, "SEQ mapa");

		printf("FALLA %s: real x=%.1f y=%.1f yaw=%.1f\n", caso, (double)real->x, (double)real->y, (double)real->yaw);
		if (hay) {
			printf("  medido x=%.1f y=%.1f yaw=%.1f\n", (double)c->x_mm, (double)c->y_mm, (double)c->yaw_deg);
		}
		printf("%s\n", (p != NULL) ? p : salida);
	}
}

static void verificar(const char *caso, bool hay, const busqueda_caja_t *c, const mundo_caja_t *real)
{
	informar(caso, hay, c, real);
	assert(hay);
	assert(fabsf(c->x_mm - real->x) <= TOL_XY_MM);
	assert(fabsf(c->y_mm - real->y) <= TOL_XY_MM);
	assert(diferencia_de_eje(c->yaw_deg, real->yaw) <= TOL_YAW_DEG);
}

/** La caja se encuentra en distintas posiciones, orientaciones y alturas. */
static void test_poses(void)
{
	static const struct { float x, y, yaw, alto; int tipo; } casos[] = {
		{   0.0f, 250.0f,   0.0f,  60.0f, 1 },
		{   0.0f, 250.0f,   0.0f,  80.0f, 2 },
		{   0.0f, 250.0f,   0.0f, 100.0f, 3 },
		{  20.0f, 240.0f,  30.0f,  80.0f, 2 },
		{ -25.0f, 260.0f, -40.0f,  60.0f, 1 },
		{  10.0f, 265.0f,  90.0f, 100.0f, 3 },
		{ -15.0f, 235.0f,  60.0f,  80.0f, 2 },
		{  25.0f, 255.0f, -75.0f,  60.0f, 1 },
	};
	float peor_xy = 0.0f;
	float peor_yaw = 0.0f;

	for (unsigned i = 0; i < sizeof(casos) / sizeof(casos[0]); i++) {
		busqueda_caja_t c;

		preparar();
		poner_caja(casos[i].x, casos[i].y, casos[i].yaw, casos[i].alto);

		bool hay = escanear(&c);

		verificar("pose", hay, &c, &mundo_caja[0]);
		assert(c.tipo == casos[i].tipo);
		assert(fabsf(c.alto_mm - casos[i].alto) <= CAJA_TOL_ALTO_MM);
		assert(fabsf(c.largo_mm - CAJA_LARGO_MM) <= CAJA_TOL_MEDIDA_MM);
		assert(fabsf(c.ancho_mm - CAJA_ANCHO_MM) <= CAJA_TOL_MEDIDA_MM);

		/* Informa lo mismo que guarda, y una sola vez. */
		assert(contar(salida, "SEQ mapa cajas=1") == 1);
		assert(contar(salida, "SEQ eje") == 1);
		assert(contar(salida, "CAJA x=") == 1);
		assert(contar(salida, "FAULT") == 0);
		assert(contar(salida, "DONE") == 1);

		/* Se acerca para medir la altura, sin llegar a tocarla, y termina arriba con la garra abierta. */
		const char *medida = strstr(salida, "LEER_CAJA");

		assert(medida != NULL);
		assert(fabsf(EjeZ_Altura() - Z_BARRIDO_MM) <= 0.3f);
		assert(fabsf(Gripper_Apertura() - GRIPPER_MAX_MM) <= 0.1f);

		/* La garra termina sobre el centro y alineada con la caja. */
		float q1, q2, x, y;

		Motion_PosturaComandada(&q1, &q2);
		(void)Cinematica_Directa(q1, q2, &x, &y);
		assert(fabsf(x - c.x_mm) <= 0.5f && fabsf(y - c.y_mm) <= 0.5f);
		assert(diferencia_de_eje(Cinematica_YawGarra(q1, q2, EjeZ_Orientacion()), c.yaw_deg) <= TOL_ORIENT_DEG);

		float exy = hypotf(c.x_mm - casos[i].x, c.y_mm - casos[i].y);
		float eyaw = diferencia_de_eje(c.yaw_deg, casos[i].yaw);

		peor_xy = (exy > peor_xy) ? exy : peor_xy;
		peor_yaw = (eyaw > peor_yaw) ? eyaw : peor_yaw;
	}
	printf("OK  SCAN: posiciones, orientaciones y alturas (peor error %.2f mm, %.2f grados)\n",
		   (double)peor_xy, (double)peor_yaw);
}

/** Despues de SCAN, PICK sin argumentos toma la caja, y PLACE la deja a la misma altura. */
static void test_pick_de_la_caja(void)
{
	busqueda_caja_t c;

	preparar();
	assert(strstr(tipear("pick\r"), "ERR 9") != NULL);
	assert(!Secuencia_Activa() && !Motion_Ocupado());
	assert(strstr(tipear("map\r"), "ERR 9") != NULL);

	poner_caja(10.0f, 245.0f, 25.0f, 80.0f);
	assert(escanear(&c));
	assert(strstr(tipear("pick\r"), "OK\r\n") != NULL);
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(!Secuencia_Activa());
	assert(contar(salida, "FAULT") == 0);
	assert(contar(salida, "DONE") == 1);

	/* Bajo hasta dejar la tapa a PICK_DIST_TOMA_MM de los sensores. */
	float z_esperada = mundo_z_mesa + 80.0f + PICK_DIST_TOMA_MM;
	const char *ultimo = NULL;

	for (const char *p = strstr(salida, "z_toma="); p != NULL; p = strstr(p + 1, "z_toma=")) {
		ultimo = p;
	}

	float z_medida;

	assert(sscanf(ultimo, "z_toma=%f", &z_medida) == 1);
	assert(fabsf(z_medida - z_esperada) <= 3.0f);

	/* Sujeta con los dedos a lo ancho, en el centro, y sube. */
	float q1, q2, x, y;

	Motion_PosturaComandada(&q1, &q2);
	(void)Cinematica_Directa(q1, q2, &x, &y);
	assert(fabsf(x - 10.0f) <= TOL_XY_MM && fabsf(y - 245.0f) <= TOL_XY_MM);
	assert(diferencia_de_eje(Cinematica_YawGarra(q1, q2, EjeZ_Orientacion()), 25.0f) <= TOL_YAW_DEG);
	assert(fabsf(Gripper_Apertura() - GRIPPER_TOMA_MM) <= 0.1f);
	assert(fabsf(EjeZ_Altura() - Z_SEGURA_MM) <= 0.3f);

	/* La caja ya no esta donde estaba: un segundo PICK no va a buscarla ahi. */
	assert(strstr(tipear("pick\r"), "ERR 9") != NULL);

	/* PLACE baja a la altura de toma, sea la caja que sea. */
	mundo_cajas = 0;
	assert(strstr(tipear("place 0 260\r"), "OK\r\n") != NULL);
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(strstr(salida, "DONE") != NULL);

	const char *z_place = strstr(salida, " Z a=");
	float z_soltar;

	assert(z_place != NULL);
	z_place = strstr(z_place + 1, " Z a=");
	assert(z_place != NULL);
	assert(sscanf(z_place, " Z a=%f", &z_soltar) == 1);
	assert(fabsf(z_soltar - (z_medida + PLACE_HOLGURA_MM)) <= 0.2f);

	/* Un segundo PLACE ya no tiene toma previa: baja a Z_DEJAR_MM. */
	assert(strstr(tipear("place 0 260\r"), "OK\r\n") != NULL);
	borrar_salida();
	(void)correr(CORRER_MUCHO);

	char dejar[32];

	(void)snprintf(dejar, sizeof(dejar), "Z a=%.1f", (double)Z_DEJAR_MM);
	assert(strstr(salida, dejar) != NULL);
	printf("OK  PICK sobre la caja de SCAN y PLACE a la altura de toma\n");
}

/**
 * Desde arriba el cono toma parte de la mesa y la caja parece mas lejos de lo que esta: si PICK le
 * creyera a esa lectura, la garra terminaria apoyada contra la tapa. En ningun momento puede bajar
 * mas que lo que deja la tapa a PICK_DIST_TOMA_MM de los sensores.
 */
static void test_pick_no_baja_de_mas(void)
{
	static const float altos[] = CAJA_ALTOS_MM;

	for (int i = 0; i < CAJA_CANT_TIPOS; i++) {
		preparar();
		poner_caja(0.0f, 250.0f, 0.0f, altos[i]);
		assert(strstr(tipear("z 50\r"), "OK\r\n") != NULL);
		(void)correr(CORRER_MUCHO);

		float z_justa = mundo_z_mesa + altos[i] + PICK_DIST_TOMA_MM;
		float z_menor = EjeZ_Altura();

		assert(strstr(tipear("pick 0 250 0\r"), "OK\r\n") != NULL);
		borrar_salida();
		while (Secuencia_Activa()) {
			(void)correr(1);
			z_menor = (EjeZ_Altura() < z_menor) ? EjeZ_Altura() : z_menor;
		}
		assert(contar(salida, "FAULT") == 0);
		assert(fabsf(z_menor - z_justa) <= 3.0f);
	}
	printf("OK  PICK: no baja de mas aunque de lejos la caja parezca mas baja\n");
}

/** Rechazos antes de mover, y fallas durante la busqueda. */
static void test_fallas(void)
{
	busqueda_caja_t c;

	/* Area invalida o inalcanzable: nada se mueve. */
	preparar();
	assert(strstr(tipear("scan -90 180 0 150\r"), "ERR 6") != NULL);
	assert(strstr(tipear("scan -90 180 2000 150\r"), "ERR 6") != NULL);
	assert(strstr(tipear("scan -90 500 180 150\r"), "ERR 2") != NULL);
	assert(!Secuencia_Activa() && !Motion_Ocupado());

	/* El sentido de los lados no importa: es la misma area. */
	preparar();
	poner_caja(0.0f, 250.0f, 0.0f, 80.0f);
	assert(strstr(tipear("scan 90 330 -180 -150\r"), "OK\r\n") != NULL);
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(Busqueda_Caja(&c));
	verificar("lados negativos", true, &c, &mundo_caja[0]);

	/* Ocupado: no arranca dos veces. */
	preparar();
	assert(strstr(tipear(AREA_CMD), "OK\r\n") != NULL);
	assert(strstr(tipear(AREA_CMD), "ERR 4") != NULL);
	assert(strstr(tipear("map\r"), "ERR 4") != NULL);
	assert(strstr(tipear("stop\r"), "OK") != NULL);
	(void)correr(CORRER_MUCHO);

	/* Mesa vacia: lo dice, y no hay caja para tomar. */
	preparar();
	assert(!escanear(&c));
	assert(strstr(salida, "SEQ mapa cajas=0") != NULL);
	assert(strstr(salida, "FAULT rutina cortada: no se encontro ninguna caja") != NULL);
	assert(strstr(tipear("pick\r"), "ERR 9") != NULL);

	/* El mapa queda igual, para verlo. */
	assert(strstr(tipear("map\r"), "OK celda=") != NULL);
	pasar_ticks(MAPA_CELDAS_MAX / MAPA_COLUMNAS_MAX);

	/* Falta un sensor: la busqueda los necesita a los dos. */
	preparar();
	poner_caja(0.0f, 250.0f, 0.0f, 80.0f);
	vl53_presente[VL53_B] = false;
	assert(strstr(tipear(AREA_CMD), "ERR 8") != NULL);
	assert(!Secuencia_Activa() && !Motion_Ocupado());

	/* Un sensor deja de contestar en pleno barrido: corta. */
	preparar();
	poner_caja(0.0f, 250.0f, 0.0f, 80.0f);
	assert(strstr(tipear(AREA_CMD), "OK\r\n") != NULL);
	assert(correr(1500) == 1500);
	vl53_mudo[VL53_B] = true;
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(strstr(salida, "FAULT rutina cortada: sensor de distancia") != NULL);
	assert(!Secuencia_Activa() && !Motion_Ocupado());
	assert(!Busqueda_Caja(&c));

	/* Una lectura demasiado cerca corta antes de que los dedos choquen. */
	preparar();
	poner_caja(0.0f, 250.0f, 0.0f, 130.0f);
	assert(!escanear(&c));
	assert(strstr(salida, "FAULT rutina cortada: un sensor quedo muy cerca") != NULL);
	assert(!Motion_Ocupado());

	/* Algo que no tiene las medidas de la caja no se publica como caja. */
	preparar();
	mundo_caja[mundo_cajas++] = (mundo_caja_t){ 0.0f, 250.0f, 0.0f, 180.0f, 30.0f, 80.0f };
	assert(!escanear(&c));
	assert(strstr(salida, "FAULT") != NULL);

	/* STOP en cualquier momento: restaura la velocidad y no queda nada corriendo. */
	static const int ticks_stop[] = { 100, 1000, 3000, 6000, 9000, 12000 };

	for (unsigned i = 0; i < sizeof(ticks_stop) / sizeof(ticks_stop[0]); i++) {
		preparar();
		poner_caja(0.0f, 250.0f, 0.0f, 80.0f);
		assert(strstr(tipear("speed 40\r"), "OK") != NULL);
		assert(strstr(tipear(AREA_CMD), "OK\r\n") != NULL);
		(void)correr(ticks_stop[i]);
		if (Secuencia_Activa()) {
			assert(strstr(tipear("stop\r"), "OK") != NULL);
			borrar_salida();
			(void)correr(CORRER_MUCHO);
			assert(contar(salida, "STOPPED") == 1);
		}
		assert(!Secuencia_Activa() && !Motion_Ocupado() && !EjeZ_Ocupado());
		assert(!Busqueda_Muestreando());
		assert(fabsf(Motion_Velocidad() - 40.0f) < 0.01f);

		/* Y se puede volver a empezar. */
		assert(escanear(&c));
		verificar("despues de STOP", true, &c, &mundo_caja[0]);
		assert(fabsf(Motion_Velocidad() - 40.0f) < 0.01f);
	}

	/* Cambiar el cero del eje Z invalida la mesa: MAP deja de mostrar alturas. */
	preparar();
	poner_caja(0.0f, 250.0f, 0.0f, 80.0f);
	assert(escanear(&c));
	assert(strstr(tipear("map\r"), "OK celda=") != NULL);
	pasar_ticks(MAPA_CELDAS_MAX / MAPA_COLUMNAS_MAX);
	assert(strstr(tipear("setz0\r"), "OK") != NULL);
	assert(strstr(tipear("map\r"), "ERR 9") != NULL);
	printf("OK  SCAN: rechazos, mesa vacia, sensor perdido, muy cerca y STOP\n");
}

/** Lo que el SCAN tiene que aguantar del montaje real. */
static void test_montaje_imperfecto(void)
{
	busqueda_caja_t c;
	bool hay;

	/* La separacion real no es la de config.h: el centro no se mueve. */
	static const float separaciones[] = { 60.0f, 80.0f };

	for (unsigned i = 0; i < sizeof(separaciones) / sizeof(separaciones[0]); i++) {
		preparar();
		mundo_separacion = separaciones[i];
		poner_caja(15.0f, 250.0f, 20.0f, 80.0f);
		hay = escanear(&c);
		verificar("separacion", hay, &c, &mundo_caja[0]);
	}

	/* La garra no es exactamente paralela a lo cargado en config.h. */
	preparar();
	mundo_yaw_cero = GARRA_YAW_CERO_DEG + 4.0f;
	poner_caja(-10.0f, 245.0f, 15.0f, 80.0f);
	hay = escanear(&c);
	assert(hay);
	assert(fabsf(c.x_mm - mundo_caja[0].x) <= 2.0f * TOL_XY_MM);
	assert(fabsf(c.y_mm - mundo_caja[0].y) <= 2.0f * TOL_XY_MM);

	/* Ruido de las lecturas. */
	preparar();
	mundo_ruido_mm = 4.0f;
	poner_caja(5.0f, 255.0f, -20.0f, 60.0f);
	hay = escanear(&c);
	verificar("ruido", hay, &c, &mundo_caja[0]);
	assert(c.tipo == 1);

	/* El brazo real va atrasado respecto del comando: ida y vuelta lo cancelan. */
	preparar();
	mundo_retardo_ticks = 10;
	poner_caja(5.0f, 240.0f, 45.0f, 100.0f);
	hay = escanear(&c);
	verificar("retardo", hay, &c, &mundo_caja[0]);

	/* Un sensor mide de mas: con mas diferencia que la tolerancia, no da la caja por buena. */
	preparar();
	mundo_desvio_a = CAJA_TOL_PAREJA_MM + 10.0f;
	poner_caja(5.0f, 250.0f, 0.0f, 80.0f);
	(void)escanear(&c);
	assert(strstr(salida, "FAULT") != NULL);
	assert(!Busqueda_Caja(&c));
	printf("OK  SCAN: separacion y yaw mal cargados, ruido, retardo y sensor descalibrado\n");
}

/** Dos cajas: cuenta las dos y sigue con la mayor. */
static void test_dos_cajas(void)
{
	busqueda_caja_t c;

	preparar();
	poner_caja(-40.0f, 215.0f, 0.0f, 60.0f);
	mundo_caja[mundo_cajas++] = (mundo_caja_t){ 25.0f, 290.0f, 0.0f, CAJA_LARGO_MM + 15.0f, CAJA_ANCHO_MM, 80.0f };
	assert(strstr(tipear(AREA_CMD), "OK\r\n") != NULL);
	borrar_salida();
	(void)correr(CORRER_MUCHO);
	assert(strstr(salida, "SEQ mapa cajas=2") != NULL);
	assert(Busqueda_Caja(&c));
	assert(fabsf(c.x_mm - 25.0f) <= TOL_XY_MM && fabsf(c.y_mm - 290.0f) <= TOL_XY_MM);
	printf("OK  SCAN: con dos cajas cuenta las dos y sigue con la mayor\n");
}

/** MAP muestra las alturas sobre la mesa, fila por fila, con la caja donde esta. */
static void test_map(void)
{
	busqueda_caja_t c;

	preparar();
	poner_caja(0.0f, 250.0f, 0.0f, 80.0f);
	assert(escanear(&c));

	const char *r = tipear("map\r");

	assert(strstr(r, "OK celda=15.0") != NULL);

	int filas = Mapa_Filas();

	assert(filas == 11 && Mapa_Columnas() == 13);
	pasar_ticks(filas + 2);
	assert(contar(salida, "\nM,") + contar(salida, "KM,") == filas);
	assert(contar(salida, "DONE map") == 1);

	/* En la fila del centro, la columna del centro esta en lo alto de la caja. */
	const char *fila = strstr(salida, "M,5,");
	int alturas[13];
	int n = 0;

	assert(fila != NULL);
	fila += strlen("M,5,");
	while (n < 13) {
		assert(sscanf(fila, "%d", &alturas[n]) == 1);
		n++;
		fila = strchr(fila, ',');
		if (fila == NULL) {
			break;
		}
		fila++;
	}
	assert(n == 13);
	assert(alturas[6] >= 50 && alturas[6] <= 90);
	assert(alturas[0] >= -5 && alturas[0] <= 5);
	assert(alturas[12] >= -5 && alturas[12] <= 5);
	printf("OK  MAP: alturas por fila\n");
}

int main(void)
{
	printf("== test_busqueda ==\n");
	test_poses();
	test_pick_de_la_caja();
	test_pick_no_baja_de_mas();
	test_fallas();
	test_montaje_imperfecto();
	test_dos_cajas();
	test_map();
	printf("test_busqueda: TODO OK\n");
	return 0;
}
