/**
 * @file secuencia.c
 * @brief Ejecutor de rutinas de varios pasos.
 *
 * Cada rutina es una funcion generadora que, dado un indice, devuelve ese paso. Asi no hace falta
 * reservar memoria para las listas (un circulo o un REPEAT largos tienen muchos pasos), y el
 * pre-vuelo recorre exactamente lo mismo que despues se ejecuta.
 */
#include "secuencia.h"

#include <math.h>
#include <stddef.h>

#include "busqueda.h"
#include "cinematica.h"
#include "config.h"
#include "eje_z.h"
#include "gripper.h"
#include "laseres.h"
#include "mapa.h"
#include "motion.h"
#include "planificador.h"

_Static_assert(CALIB_MUESTRAS > 0 && CALIB_CANT_PASOS > 1, "CALIB necesita muestras y puntos");

/** Pasos por punto de CALIB: mover, esperar, medir. */
#define PASOS_CALIB_POR_PUNTO	3
/** Pasos de una junta de CALIB: todos sus puntos, el ajuste y volver a HOME. */
#define PASOS_CALIB_POR_JUNTA	(CALIB_CANT_PASOS * PASOS_CALIB_POR_PUNTO + 2)
/** Pasos de un ciclo de REPEAT: ir y medir a cada uno de los dos puntos. */
#define PASOS_REPEAT_POR_CICLO	4

#define GRADOS_VUELTA			360.0f
#define CUARTO_DE_VUELTA		90.0f
#define PI_F					3.14159265f

/* Cuanto se mueve la pinza a cada lado del centro en una pasada lenta. A lo largo, cada sensor
 * solo necesita cruzar el extremo de su lado: el A llega ahi con la pinza media separacion antes. */
#define PASADA_ANCHO_MM			(0.5f * CAJA_ANCHO_MM + CENTRADO_MARGEN_MM)
#define PASADA_LARGO_MM			(0.5f * (CAJA_LARGO_MM - SENSOR_SEPARACION_MM) + CENTRADO_MARGEN_MM)

/* Lo que no es pasada en la primera parte de SCAN: Z, velocidad, ir a la esquina, empezar a medir. */
#define PASOS_ANTES_DEL_BARRIDO	4U

/** Tipos de paso. */
typedef enum {
	PASO_NADA = 0,		/**< No hace nada: para los pasos opcionales de una rutina */
	PASO_XY,			/**< a, b = x, y */
	PASO_JUNTAS,		/**< a, b = th1, th2 */
	PASO_HOME,
	PASO_Z,				/**< a = altura */
	PASO_Z_APROX,		/**< se acerca al objeto, segun la ultima medicion */
	PASO_Z_TOMA,		/**< baja a la altura calculada con el sensor */
	PASO_ORIENT,		/**< a = direccion de la linea de sensores en la mesa */
	PASO_GRIPPER,		/**< a = apertura */
	PASO_DIST,			/**< mide el objeto y calcula la altura de toma; a = fraccion del proximo acercamiento */
	PASO_VELOCIDAD,		/**< a = velocidad de los tramos siguientes */
	PASO_MAPA_INICIO,	/**< empieza a cargar lecturas en el mapa */
	PASO_MAPA_FIN,		/**< deja de medir, busca la caja y valida el centrado */
	PASO_PASADA_INICIO,	/**< empieza a guardar las lecturas de una pasada lenta */
	PASO_ANCHO_FIN,		/**< cierra una pasada a lo ancho; n = 1 en la ultima */
	PASO_LARGO_FIN,		/**< cierra una pasada a lo largo; n = 1 en la ultima */
	PASO_LEER_CAJA,		/**< mide la tapa con la pinza en el centro y publica la caja */
	PASO_ESPERA,		/**< a = ms */
	PASO_MEDIR,			/**< a, b = x, y comandados; n = punto */
	PASO_CALIB,			/**< a = th comandado; n = indice de junta */
	PASO_CALIB_FIN		/**< n = indice de junta */
} paso_tipo_t;

typedef struct {
	paso_tipo_t tipo;
	float a;
	float b;
	int n;
} paso_t;

/** Genera el paso @p i de una rutina. @return false si la rutina no tiene ese paso. */
typedef bool (*generador_t)(uint32_t i, paso_t *p);

/** Resultado de arrancar un paso. */
typedef enum {
	ARRANQUE_LISTO,		/**< Ya termino */
	ARRANQUE_CORRE,		/**< Hay que esperar a que termine */
	ARRANQUE_FALLO		/**< No se pudo; ya se llamo a fallar() */
} arranque_t;

typedef enum {
	FASE_INACTIVA = 0,
	FASE_PROXIMO,		/**< Toca arrancar el proximo paso */
	FASE_EJECUTANDO,	/**< Esperando que termine el paso actual */
	FASE_FRENANDO		/**< Se corto: esperando que todo quede quieto */
} fase_t;

/** Parametros de la rutina activa. */
static struct {
	float x1, y1, x2, y2;
	float z;
	float z_inicial;
	float rot;
	bool con_rot;
	float ancho;
	float lado_x, lado_y;
	float radio;
	uint32_t cuerdas;
	uint32_t ciclos;
	int junta_ini;
	int juntas;
	bool lapiz_abajo;
	bool usa_caja;			/**< PICK sobre la caja del ultimo SCAN */
	float area_x, area_y;	/**< SCAN: esquina de menores X e Y */
	float area_ancho, area_alto;
	uint32_t filas;
	float yaw;				/**< SCAN: direccion de la linea de sensores, de B hacia A */
	float largo_caja, ancho_caja;
} prm;

static fase_t fase;
static generador_t generador;
static uint32_t indice;
static paso_t paso;
static seq_motivo_t motivo;
static seq_evento_t evento;
static uint32_t ms_espera;
static bool vel_cambiada;
static float vel_usuario;

/* PICK */
static float z_toma;
static float z_aprox;
static bool toma_hecha;		/**< Ya se bajo a z_toma: PLACE suelta a esa misma altura */
static float lect_suma[VL53_CANT];
static int lect_cant[VL53_CANT];

/* REPEAT */
static metricas_t metricas[SECUENCIA_PUNTOS_REPEAT];
static float postura_medida[SECUENCIA_PUNTOS_REPEAT][JUNTA_CANT];

/* CALIB */
static recta_t recta;
static float calib_cmd[CALIB_CANT_PASOS];
static float calib_enc[CALIB_CANT_PASOS];
static int calib_cant;
static float calib_primera;
static const float calib_pasos[CALIB_CANT_PASOS] = CALIB_PASOS_DEG;
static const float home_juntas[JUNTA_CANT] = { HOME_TH1_DEG, HOME_TH2_DEG };
static const float rango_servo[JUNTA_CANT] = { SHOULDER_RANGE_DEG, ELBOW_RANGE_DEG };
static const float zero_servo[JUNTA_CANT] = { SHOULDER_ZERO_SERVO_DEG, ELBOW_ZERO_SERVO_DEG };

/* ---- Generadores de pasos -------------------------------------------------- */

static void paso_z(paso_t *p, float z_mm)
{
	p->tipo = PASO_Z;
	p->a = z_mm;
}

static void paso_xy(paso_t *p, float x_mm, float y_mm)
{
	p->tipo = PASO_XY;
	p->a = x_mm;
	p->b = y_mm;
}

static void paso_gripper(paso_t *p, float mm)
{
	p->tipo = PASO_GRIPPER;
	p->a = mm;
}

static void paso_espera(paso_t *p, uint32_t ms)
{
	p->tipo = PASO_ESPERA;
	p->a = (float)ms;
}

/** Mide el objeto. @param fraccion Que parte de lo que falta recorre el acercamiento siguiente. */
static void paso_medir(paso_t *p, float fraccion)
{
	if (PICK_USA_SENSOR) {
		p->tipo = PASO_DIST;
		p->a = fraccion;
	}
}

static void paso_acercarse(paso_t *p)
{
	if (PICK_USA_SENSOR) {
		p->tipo = PASO_Z_APROX;
	}
}

static bool pasos_mover_xyz(uint32_t i, paso_t *p)
{
	bool sube = prm.z > prm.z_inicial;

	if (i > 1U) {
		return false;
	}
	if ((i == 0U) == sube) {
		paso_z(p, prm.z);
	} else {
		paso_xy(p, prm.x1, prm.y1);
	}
	return true;
}

/* Tomar y dejar llevan una cantidad fija de pasos: PICKPLACE los encadena por indice. */
#define PASOS_TOMAR		13
#define PASOS_DEJAR		6

static bool pasos_tomar(uint32_t i, paso_t *p)
{
	switch (i) {
	case 0U:
		paso_z(p, Z_SEGURA_MM);
		break;
	case 1U:
		paso_xy(p, prm.x1, prm.y1);
		break;
	case 2U:
		if (prm.con_rot) {
			p->tipo = PASO_ORIENT;
			p->a = prm.rot;
		}
		break;
	case 3U:
		paso_gripper(p, GRIPPER_MAX_MM);
		break;
	case 4U:
		paso_medir(p, PICK_APROX_FRACCION);
		break;
	case 5U:
	case 7U:
		paso_acercarse(p);
		break;
	case 6U:
	case 8U:
		paso_medir(p, 1.0f);
		break;
	case 9U:
		p->tipo = PASO_Z_TOMA;
		break;
	case 10U:
		paso_gripper(p, prm.ancho);
		break;
	case 11U:
		paso_espera(p, SEQ_ESPERA_GRIPPER_MS);
		break;
	case 12U:
		paso_z(p, Z_SEGURA_MM);
		break;
	default:
		return false;
	}
	return true;
}

static bool pasos_dejar(uint32_t i, paso_t *p)
{
	switch (i) {
	case 0U:
		paso_z(p, Z_SEGURA_MM);
		break;
	case 1U:
		paso_xy(p, prm.x2, prm.y2);
		break;
	case 2U:
		/* Lo tomado se suelta a la misma altura: asi no depende de cual de las cajas era. */
		paso_z(p, toma_hecha ? z_toma + PLACE_HOLGURA_MM : Z_DEJAR_MM);
		break;
	case 3U:
		paso_gripper(p, GRIPPER_MAX_MM);
		break;
	case 4U:
		paso_espera(p, SEQ_ESPERA_GRIPPER_MS);
		break;
	case 5U:
		paso_z(p, Z_SEGURA_MM);
		break;
	default:
		return false;
	}
	return true;
}

static bool pasos_ciclo(uint32_t i, paso_t *p)
{
	if (i < PASOS_TOMAR) {
		return pasos_tomar(i, p);
	}
	if (i < PASOS_TOMAR + PASOS_DEJAR) {
		return pasos_dejar(i - PASOS_TOMAR, p);
	}
	if (i == PASOS_TOMAR + PASOS_DEJAR) {
		p->tipo = PASO_HOME;
		return true;
	}
	return false;
}

static bool pasos_lapiz(uint32_t i, paso_t *p)
{
	if (i > 0U) {
		return false;
	}
	paso_z(p, prm.lapiz_abajo ? Z_LAPIZ_ABAJO_MM : Z_LAPIZ_ARRIBA_MM);
	return true;
}

static bool pasos_rect(uint32_t i, paso_t *p)
{
	switch (i) {
	case 0U:
		paso_z(p, Z_LAPIZ_ARRIBA_MM);
		break;
	case 1U:
	case 6U:
		paso_xy(p, prm.x1, prm.y1);
		break;
	case 2U:
		paso_z(p, Z_LAPIZ_ABAJO_MM);
		break;
	case 3U:
		paso_xy(p, prm.x1 + prm.lado_x, prm.y1);
		break;
	case 4U:
		paso_xy(p, prm.x1 + prm.lado_x, prm.y1 + prm.lado_y);
		break;
	case 5U:
		paso_xy(p, prm.x1, prm.y1 + prm.lado_y);
		break;
	case 7U:
		paso_z(p, Z_LAPIZ_ARRIBA_MM);
		break;
	default:
		return false;
	}
	return true;
}

static bool pasos_circulo(uint32_t i, paso_t *p)
{
	/* 0: levantar, 1: ir al inicio, 2: apoyar, 3..: cada cuerda, ultimo: levantar. */
	if (i == 0U) {
		paso_z(p, Z_LAPIZ_ARRIBA_MM);
	} else if (i == 2U) {
		paso_z(p, Z_LAPIZ_ABAJO_MM);
	} else if (i == 3U + prm.cuerdas) {
		paso_z(p, Z_LAPIZ_ARRIBA_MM);
	} else if (i < 3U + prm.cuerdas) {
		uint32_t k = (i == 1U) ? 0U : i - 2U;
		float ang = GRADOS_VUELTA * (float)k / (float)prm.cuerdas;
		float x, y;

		Cinematica_PuntoCircunferencia(prm.x1, prm.y1, prm.radio, ang, &x, &y);
		paso_xy(p, x, y);
	} else {
		return false;
	}
	return true;
}

static bool pasos_repetir(uint32_t i, paso_t *p)
{
	if (i >= prm.ciclos * PASOS_REPEAT_POR_CICLO) {
		return false;
	}

	uint32_t r = i % PASOS_REPEAT_POR_CICLO;
	int punto = (r < 2U) ? 0 : 1;
	float x = (punto == 0) ? prm.x1 : prm.x2;
	float y = (punto == 0) ? prm.y1 : prm.y2;

	/* El MOVE ya espera SETTLE_MS antes de terminar (hay encoders): la medicion es en reposo. */
	if ((r % 2U) == 0U) {
		paso_xy(p, x, y);
	} else {
		p->tipo = PASO_MEDIR;
		p->a = x;
		p->b = y;
		p->n = punto;
	}
	return true;
}

static bool pasos_calibrar(uint32_t i, paso_t *p)
{
	uint32_t j_rel = i / PASOS_CALIB_POR_JUNTA;

	if (j_rel >= (uint32_t)prm.juntas) {
		return false;
	}

	int j = prm.junta_ini + (int)j_rel;
	uint32_t r = i % PASOS_CALIB_POR_JUNTA;

	if (r == CALIB_CANT_PASOS * PASOS_CALIB_POR_PUNTO) {
		p->tipo = PASO_CALIB_FIN;
		p->n = j;
		return true;
	}
	if (r > CALIB_CANT_PASOS * PASOS_CALIB_POR_PUNTO) {
		p->tipo = PASO_HOME;
		return true;
	}

	float q[JUNTA_CANT] = { home_juntas[JUNTA_HOMBRO], home_juntas[JUNTA_CODO] };

	q[j] += calib_pasos[r / PASOS_CALIB_POR_PUNTO];
	if (!Cinematica_JuntasEnLimites(q[JUNTA_HOMBRO], q[JUNTA_CODO])) {
		return true;	/* punto fuera de limites: los tres pasos quedan en PASO_NADA */
	}

	switch (r % PASOS_CALIB_POR_PUNTO) {
	case 0U:
		p->tipo = PASO_JUNTAS;
		p->a = q[JUNTA_HOMBRO];
		p->b = q[JUNTA_CODO];
		break;
	case 1U:
		paso_espera(p, CALIB_SETTLE_MS);
		break;
	default:
		p->tipo = PASO_CALIB;
		p->a = q[j];
		p->n = j;
		break;
	}
	return true;
}

/** Punto a cierta distancia del centro de la caja, a lo largo de una direccion. */
static void paso_sobre_eje(paso_t *p, float distancia_mm, float ang_deg)
{
	float x, y;

	Cinematica_PuntoCircunferencia(prm.x1, prm.y1, distancia_mm, ang_deg, &x, &y);
	paso_xy(p, x, y);
}

/** Una pasada de ida y vuelta sobre la caja. */
enum {
	P_INICIO = 0, P_MEDIR_IDA, P_IDA, P_FIN_IDA, P_MEDIR_VUELTA, P_VUELTA, P_FIN_VUELTA,
	P_CANT
};

static void paso_de_pasada(uint32_t etapa, float mitad_mm, float dir_deg, paso_tipo_t fin, paso_t *p)
{
	switch (etapa) {
	case P_INICIO:
	case P_VUELTA:
		paso_sobre_eje(p, -mitad_mm, dir_deg);
		break;
	case P_IDA:
		paso_sobre_eje(p, mitad_mm, dir_deg);
		break;
	case P_MEDIR_IDA:
	case P_MEDIR_VUELTA:
		p->tipo = PASO_PASADA_INICIO;
		break;
	case P_FIN_IDA:
		p->tipo = fin;
		break;
	default:
		p->tipo = fin;
		p->n = 1;
		break;
	}
}

/** Pasos del centrado, una vez ubicada la caja: el centro y la direccion se van corrigiendo. */
enum {
	C_CENTRO_1 = 0, C_ORIENT_1, C_VEL,
	C_ANCHO,
	C_CENTRO_2 = C_ANCHO + P_CANT, C_ORIENT_2,
	C_LARGO,
	C_CENTRO_3 = C_LARGO + P_CANT, C_ORIENT_3,
	C_ABRIR, C_MEDIR_LEJOS, C_BAJAR_1, C_MEDIR_CERCA, C_BAJAR_2, C_LEER, C_SUBIR,
	C_CANT
};

static bool pasos_centrar(uint32_t i, paso_t *p)
{
	if (i >= C_ANCHO && i < C_ANCHO + P_CANT) {
		paso_de_pasada(i - C_ANCHO, PASADA_ANCHO_MM, prm.yaw + CUARTO_DE_VUELTA, PASO_ANCHO_FIN, p);
		return true;
	}
	if (i >= C_LARGO && i < C_LARGO + P_CANT) {
		paso_de_pasada(i - C_LARGO, PASADA_LARGO_MM, prm.yaw, PASO_LARGO_FIN, p);
		return true;
	}

	switch (i) {
	case C_CENTRO_1:
	case C_CENTRO_2:
	case C_CENTRO_3:
		paso_xy(p, prm.x1, prm.y1);
		break;
	case C_ORIENT_1:
	case C_ORIENT_2:
	case C_ORIENT_3:
		p->tipo = PASO_ORIENT;
		p->a = prm.yaw;
		break;
	case C_VEL:
		p->tipo = PASO_VELOCIDAD;
		p->a = CENTRADO_VEL_MM_S;
		break;
	case C_ABRIR:
		paso_gripper(p, GRIPPER_MAX_MM);
		break;
	case C_MEDIR_LEJOS:
		paso_medir(p, PICK_APROX_FRACCION);
		break;
	case C_MEDIR_CERCA:
		paso_medir(p, 1.0f);
		break;
	case C_BAJAR_1:
	case C_BAJAR_2:
		paso_acercarse(p);
		break;
	case C_LEER:
		p->tipo = PASO_LEER_CAJA;
		break;
	case C_SUBIR:
		paso_z(p, Z_BARRIDO_MM);
		break;
	default:
		return false;
	}
	return true;
}

/** Tramos en zigzag del barrido, sin contar el que lleva a la esquina. */
static uint32_t tramos_barrido(void)
{
	return 2U * prm.filas - 1U;
}

/** Punto de llegada de un tramo: pasada, salto a la fila siguiente, pasada al reves, salto... */
static void paso_barrido(paso_t *p, uint32_t tramo)
{
	uint32_t fila = (tramo + 1U) / 2U;
	bool al_final = ((tramo + 2U) / 2U) % 2U == 1U;
	float y = prm.area_y + prm.area_alto * (float)fila / (float)(prm.filas - 1U);

	paso_xy(p, al_final ? prm.area_x + prm.area_ancho : prm.area_x, y);
}

static bool pasos_buscar(uint32_t i, paso_t *p)
{
	switch (i) {
	case 0U:
		paso_z(p, Z_BARRIDO_MM);
		return true;
	case 1U:
		p->tipo = PASO_VELOCIDAD;
		p->a = BARRIDO_VEL_MM_S;
		return true;
	case 2U:
		paso_xy(p, prm.area_x, prm.area_y);
		return true;
	case 3U:
		p->tipo = PASO_MAPA_INICIO;
		return true;
	default:
		break;
	}

	uint32_t tramo = i - PASOS_ANTES_DEL_BARRIDO;

	if (tramo < tramos_barrido()) {
		paso_barrido(p, tramo);
		return true;
	}
	if (tramo == tramos_barrido()) {
		p->tipo = PASO_MAPA_FIN;
		return true;
	}
	return pasos_centrar(tramo - tramos_barrido() - 1U, p);
}

/** Pide un paso, con los campos ya en cero: asi los generadores solo escriben lo que usan. */
static bool generar(generador_t gen, uint32_t i, paso_t *p)
{
	*p = (paso_t){ .tipo = PASO_NADA };
	return gen(i, p);
}

/* ---- Pre-vuelo -------------------------------------------------------------- */

static seq_res_t validar_tramo(const paso_t *p, float *th1, float *th2)
{
	plan_t plan;

	if (!Motion_ServosActivos()) {
		return SEQ_ERR_APAGADO;
	}
	switch (Planificador_Preparar(&plan, *th1, *th2, p->a, p->b,
								  Motion_Velocidad(), Motion_Aceleracion())) {
	case PLAN_OK: {
		ik_sol_t sol;

		if (Cinematica_InversaRama(p->a, p->b, plan.rama, &sol) != CIN_OK) {
			return SEQ_ERR_LIMITE;
		}
		*th1 = sol.th1_deg;
		*th2 = sol.th2_deg;
		return SEQ_OK;
	}
	case PLAN_YA_EN_DESTINO:
		return SEQ_OK;
	case PLAN_ERR_RADIO:
		return SEQ_ERR_RADIO;
	case PLAN_ERR_LIMITE:
		return SEQ_ERR_LIMITE;
	default:
		return SEQ_ERR_PARAM;
	}
}

/**
 * Recorre los pasos desde @p desde hasta antes de @p hasta y valida cada uno sin mover nada. Lleva
 * la postura de un tramo al siguiente: un tramo puede ser valido solo desde donde dejo el anterior.
 */
static seq_res_t prevuelo(generador_t gen, uint32_t desde, uint32_t hasta)
{
	float th1, th2;

	Motion_PosturaComandada(&th1, &th2);

	for (uint32_t i = desde; i < hasta; i++) {
		paso_t p;
		seq_res_t r = SEQ_OK;

		if (!generar(gen, i, &p)) {
			break;
		}

		switch (p.tipo) {
		case PASO_XY:
			r = validar_tramo(&p, &th1, &th2);
			break;
		case PASO_JUNTAS:
			if (!Motion_ServosActivos()) {
				r = SEQ_ERR_APAGADO;
			} else if (!Cinematica_JuntasEnLimites(p.a, p.b)) {
				r = SEQ_ERR_LIMITE;
			}
			th1 = p.a;
			th2 = p.b;
			break;
		case PASO_HOME:
			if (!Motion_ServosActivos()) {
				r = SEQ_ERR_APAGADO;
			}
			th1 = HOME_TH1_DEG;
			th2 = HOME_TH2_DEG;
			break;
		case PASO_Z:
			if (!EjeZ_Referenciado()) {
				r = SEQ_ERR_Z_SIN_CERO;
			} else if (!(p.a >= Z_MIN_MM && p.a <= Z_MAX_MM)) {
				r = SEQ_ERR_Z_RANGO;
			}
			break;
		case PASO_Z_APROX:
		case PASO_Z_TOMA:
			if (!EjeZ_Referenciado()) {
				r = SEQ_ERR_Z_SIN_CERO;
			}
			break;
		case PASO_ORIENT: {
			float rot;

			if (!EjeZ_Referenciado()) {
				r = SEQ_ERR_Z_SIN_CERO;
			} else if (!Cinematica_RotParaYaw(th1, th2, p.a, EjeZ_OrientacionComandada(), &rot)) {
				r = SEQ_ERR_Z_RANGO;
			}
			break;
		}
		case PASO_GRIPPER:
			if (!(p.a >= GRIPPER_MIN_MM && p.a <= GRIPPER_MAX_MM)) {
				r = SEQ_ERR_GRIPPER_RANGO;
			}
			break;
		case PASO_DIST:
			if (Laseres_Listos() == 0) {
				r = SEQ_ERR_SENSOR;
			}
			break;
		default:
			break;
		}
		if (r != SEQ_OK) {
			return r;
		}
	}
	return SEQ_OK;
}

static seq_res_t arrancar(generador_t gen, uint32_t pasos_prevuelo)
{
	if (fase != FASE_INACTIVA || Motion_Ocupado() || EjeZ_Ocupado() || Gripper_Ocupado()) {
		return SEQ_ERR_OCUPADO;
	}
	if (Motion_Estado() == MOTION_FAULT) {
		return SEQ_ERR_FALLA;
	}

	seq_res_t r = prevuelo(gen, 0U, pasos_prevuelo);

	if (r != SEQ_OK) {
		return r;
	}
	generador = gen;
	indice = 0U;
	motivo = SEQ_MOTIVO_NINGUNO;
	vel_cambiada = false;
	fase = FASE_PROXIMO;
	return SEQ_OK;
}

/* ---- Ejecucion ----------------------------------------------------------------- */

/** Corta la rutina: frena todo y al quedar quieto avisa el motivo. */
static void fallar(seq_motivo_t m)
{
	motivo = m;
	Busqueda_DetenerMuestreo();
	Motion_Detener();
	EjeZ_Detener();
	Gripper_Detener();
	fase = FASE_FRENANDO;
}

/** Deja un aviso con el numero de paso actual. */
static seq_evento_t *nuevo_evento(seq_ev_tipo_t tipo)
{
	evento = (seq_evento_t){ .tipo = tipo, .paso = indice };
	return &evento;
}

static const char *nombre_paso(paso_tipo_t tipo)
{
	switch (tipo) {
	case PASO_XY:		return "XY";
	case PASO_JUNTAS:	return "JUNTAS";
	case PASO_HOME:		return "HOME";
	case PASO_Z:		return "Z";
	case PASO_Z_APROX:	return "Z_APROX";
	case PASO_Z_TOMA:	return "Z_TOMA";
	case PASO_ORIENT:	return "ORIENT";
	case PASO_GRIPPER:	return "GRIPPER";
	case PASO_DIST:		return "DIST";
	case PASO_LEER_CAJA:	return "LEER_CAJA";
	default:			return NULL;
	}
}

static arranque_t por_motion(motion_res_t r)
{
	if (r == MOTION_OK) {
		return ARRANQUE_CORRE;
	}
	if (r == MOTION_YA_EN_DESTINO) {
		return ARRANQUE_LISTO;
	}
	fallar(SEQ_MOTIVO_BRAZO);
	return ARRANQUE_FALLO;
}

static arranque_t por_eje_z(ejez_res_t r)
{
	if (r == EJEZ_OK) {
		return ARRANQUE_CORRE;
	}
	if (r == EJEZ_YA_EN_DESTINO) {
		return ARRANQUE_LISTO;
	}
	fallar(SEQ_MOTIVO_EJE_Z);
	return ARRANQUE_FALLO;
}

static arranque_t por_gripper(gripper_res_t r)
{
	if (r == GRIPPER_OK) {
		return ARRANQUE_CORRE;
	}
	if (r == GRIPPER_YA_EN_DESTINO) {
		return ARRANQUE_LISTO;
	}
	fallar(SEQ_MOTIVO_GRIPPER);
	return ARRANQUE_FALLO;
}

/** REPEAT: compara lo medido con lo comandado y lo acumula. */
static void medir_llegada(const paso_t *p)
{
	float q1, q2, x, y;

	(void)Motion_PosturaReal(&q1, &q2);
	(void)Cinematica_Directa(q1, q2, &x, &y);

	float z_ok = EjeZ_Referenciado() ? 1.0f : 0.0f;
	const float cmd[METRICAS_EJES] = { p->a, p->b, z_ok * EjeZ_AlturaComandada() };
	const float med[METRICAS_EJES] = { x, y, z_ok * EjeZ_Altura() };
	float e = Metricas_Agregar(&metricas[p->n], cmd, med);

	postura_medida[p->n][JUNTA_HOMBRO] = q1;
	postura_medida[p->n][JUNTA_CODO] = q2;

	seq_evento_t *ev = nuevo_evento(SEQ_EV_MEDIDA);

	ev->ciclo = indice / PASOS_REPEAT_POR_CICLO;
	ev->punto = p->n;
	ev->x_cmd = p->a;
	ev->y_cmd = p->b;
	ev->x_med = x;
	ev->y_med = y;
	ev->e = e;
}

/** Promedia CALIB_MUESTRAS lecturas de un encoder, cuidando el cruce 0/360. */
static bool promediar_encoder(int j, float *enc_deg)
{
	float base = 0.0f;
	float suma = 0.0f;

	for (int k = 0; k < CALIB_MUESTRAS; k++) {
		float lectura;

		if (!Motion_LeerJuntaMedida((junta_id_t)j, &lectura, NULL)) {
			return false;
		}
		if (k == 0) {
			base = lectura;
		}
		suma += Cinematica_NormalizarDeg(lectura - base);
	}
	*enc_deg = base + suma / (float)CALIB_MUESTRAS;
	return true;
}

/** CALIB: lee un punto y lo agrega al ajuste. */
static arranque_t medir_calib(const paso_t *p)
{
	float enc;

	if (!promediar_encoder(p->n, &enc)) {
		fallar(SEQ_MOTIVO_ENCODER);
		return ARRANQUE_FALLO;
	}
	/* El ajuste se hace sin la vuelta: los puntos siguen a la primera lectura de la junta. */
	if (calib_cant == 0) {
		calib_primera = enc;
	}
	enc = calib_primera + Cinematica_NormalizarDeg(enc - calib_primera);

	Recta_Agregar(&recta, p->a, enc);
	calib_cmd[calib_cant] = p->a;
	calib_enc[calib_cant] = enc;
	calib_cant++;

	seq_evento_t *ev = nuevo_evento(SEQ_EV_CALIB_PUNTO);

	ev->junta = p->n + 1;
	ev->cmd_deg = p->a;
	ev->enc_deg = enc;
	return ARRANQUE_LISTO;
}

/** CALIB: con los puntos de una junta calcula los valores sugeridos para config.h. */
static arranque_t terminar_calib(const paso_t *p)
{
	float pendiente, ordenada;

	if (!Recta_Ajustar(&recta, &pendiente, &ordenada)) {
		fallar(SEQ_MOTIVO_ENCODER);
		return ARRANQUE_FALLO;
	}

	float residuo = 0.0f;

	for (int k = 0; k < calib_cant; k++) {
		float d = fabsf(calib_enc[k] - (pendiente * calib_cmd[k] + ordenada));

		if (d > residuo) {
			residuo = d;
		}
	}

	float cero = Cinematica_NormalizarDeg(ordenada);
	float ganancia = fabsf(pendiente);

	seq_evento_t *ev = nuevo_evento(SEQ_EV_CALIB_AJUSTE);

	ev->junta = p->n + 1;
	ev->signo = (pendiente >= 0.0f) ? 1.0f : -1.0f;
	ev->cero_deg = (cero < 0.0f) ? cero + GRADOS_VUELTA : cero;
	ev->ganancia = ganancia;
	ev->rango_deg = rango_servo[p->n] * ganancia;
	ev->zero_servo_deg = zero_servo[p->n] * ganancia;
	ev->residuo_deg = residuo;

	Recta_Iniciar(&recta);
	calib_cant = 0;
	return ARRANQUE_LISTO;
}

/** Deja a cero las lecturas y empieza a medir la distancia a lo que hay abajo. */
static arranque_t arrancar_lecturas(void)
{
	for (int i = 0; i < VL53_CANT; i++) {
		lect_suma[i] = 0.0f;
		lect_cant[i] = 0;
	}
	if (!Laseres_Arrancar()) {
		fallar(SEQ_MOTIVO_SENSOR);
		return ARRANQUE_FALLO;
	}
	return ARRANQUE_CORRE;
}

/**
 * Gira la garra hasta la direccion pedida en la mesa. Se queda con la direccion que resulto: entre
 * las dos soluciones equivalentes, dice de que lado quedo cada sensor.
 */
static arranque_t arrancar_orientacion(float yaw_deg)
{
	float q1, q2, rot;

	Motion_PosturaComandada(&q1, &q2);
	if (!Cinematica_RotParaYaw(q1, q2, yaw_deg, EjeZ_OrientacionComandada(), &rot)) {
		fallar(SEQ_MOTIVO_EJE_Z);
		return ARRANQUE_FALLO;
	}
	prm.yaw = Cinematica_YawGarra(q1, q2, rot);
	return por_eje_z(EjeZ_MoverOrientacion(rot));
}

static arranque_t cambiar_velocidad(float v_mm_s)
{
	float previa = Motion_Velocidad();

	if (!Motion_ConfigurarVelocidad(v_mm_s)) {
		fallar(SEQ_MOTIVO_BRAZO);
		return ARRANQUE_FALLO;
	}
	if (!vel_cambiada) {
		vel_usuario = previa;
		vel_cambiada = true;
	}
	return ARRANQUE_LISTO;
}

static void restaurar_velocidad(void)
{
	if (vel_cambiada) {
		(void)Motion_ConfigurarVelocidad(vel_usuario);
		vel_cambiada = false;
	}
}

/** Valida lo que falta de la rutina con los valores recien medidos, antes de moverse otra vez. */
static arranque_t validar_lo_que_sigue(void)
{
	if (prevuelo(generador, indice + 1U, UINT32_MAX) != SEQ_OK) {
		fallar(SEQ_MOTIVO_CENTRADO_LEJOS);
		return ARRANQUE_FALLO;
	}
	return ARRANQUE_LISTO;
}

/** Termino el barrido: busca la caja en el mapa y prepara el centrado. */
static arranque_t terminar_mapa(void)
{
	mapa_mancha_t mancha = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	int cantidad;
	float mesa;

	Busqueda_DetenerMuestreo();

	bool hay = Busqueda_AnalizarMapa(EjeZ_AlturaComandada(), &mancha, &cantidad, &mesa);
	seq_evento_t *ev = nuevo_evento(SEQ_EV_MAPA);

	ev->mancha = mancha;
	ev->cantidad = cantidad;
	ev->a = mesa;
	if (!hay) {
		fallar(SEQ_MOTIVO_SIN_CAJA);
		return ARRANQUE_FALLO;
	}
	prm.x1 = mancha.x_mm;
	prm.y1 = mancha.y_mm;
	prm.yaw = mancha.yaw_deg;
	return validar_lo_que_sigue();
}

static arranque_t terminar_ancho(const paso_t *p)
{
	Busqueda_DetenerMuestreo();
	if (!Busqueda_CerrarPasadaAncho()) {
		fallar(SEQ_MOTIVO_CENTRADO);
		return ARRANQUE_FALLO;
	}
	if (p->n == 0) {
		return ARRANQUE_LISTO;
	}

	float yaw;

	if (!Busqueda_ResolverAncho(&prm.x1, &prm.y1, &yaw, &prm.ancho_caja)
			|| fabsf(Cinematica_NormalizarDeg(yaw - prm.yaw)) > CENTRADO_YAW_MAX_DEG) {
		fallar(SEQ_MOTIVO_CENTRADO);
		return ARRANQUE_FALLO;
	}
	prm.yaw = yaw;

	seq_evento_t *ev = nuevo_evento(SEQ_EV_CENTRADO);

	ev->x_cmd = prm.x1;
	ev->y_cmd = prm.y1;
	ev->a = prm.yaw;
	ev->b = prm.ancho_caja;
	return validar_lo_que_sigue();
}

static arranque_t terminar_largo(const paso_t *p)
{
	Busqueda_DetenerMuestreo();
	if (!Busqueda_CerrarPasadaLargo(p->n == 0)) {
		fallar(SEQ_MOTIVO_CENTRADO);
		return ARRANQUE_FALLO;
	}
	if (p->n == 0) {
		return ARRANQUE_LISTO;
	}
	if (!Busqueda_ResolverLargo(&prm.x1, &prm.y1, prm.yaw, &prm.largo_caja)) {
		fallar(SEQ_MOTIVO_CENTRADO);
		return ARRANQUE_FALLO;
	}
	return validar_lo_que_sigue();
}

static arranque_t arrancar_paso(const paso_t *p)
{
	switch (p->tipo) {
	case PASO_XY:
		return por_motion(Motion_MoverLineal(p->a, p->b));
	case PASO_JUNTAS:
		return por_motion(Motion_MoverJuntas(p->a, p->b));
	case PASO_HOME:
		return por_motion(Motion_Home());
	case PASO_Z:
		return por_eje_z(EjeZ_MoverAltura(p->a));
	case PASO_Z_APROX:
		return por_eje_z(EjeZ_MoverAltura(z_aprox));
	case PASO_Z_TOMA:
		return por_eje_z(EjeZ_MoverAltura(z_toma));
	case PASO_ORIENT:
		return arrancar_orientacion(p->a);
	case PASO_GRIPPER:
		return por_gripper(Gripper_MoverApertura(p->a));
	case PASO_DIST:
	case PASO_LEER_CAJA:
		return arrancar_lecturas();
	case PASO_VELOCIDAD:
		return cambiar_velocidad(p->a);
	case PASO_MAPA_INICIO:
	case PASO_PASADA_INICIO: {
		bool ok = (p->tipo == PASO_MAPA_INICIO) ? Busqueda_ArrancarMapa() : Busqueda_ArrancarPasada();

		if (!ok) {
			fallar(SEQ_MOTIVO_SENSOR);
			return ARRANQUE_FALLO;
		}
		return ARRANQUE_LISTO;
	}
	case PASO_MAPA_FIN:
		return terminar_mapa();
	case PASO_ANCHO_FIN:
		return terminar_ancho(p);
	case PASO_LARGO_FIN:
		return terminar_largo(p);
	case PASO_ESPERA:
		ms_espera = (uint32_t)p->a;
		return ARRANQUE_CORRE;
	case PASO_MEDIR:
		medir_llegada(p);
		return ARRANQUE_LISTO;
	case PASO_CALIB:
		return medir_calib(p);
	case PASO_CALIB_FIN:
		return terminar_calib(p);
	default:
		return ARRANQUE_LISTO;
	}
}

/**
 * Junta las lecturas de los sensores que estan listos, hasta DIST_MUESTRAS de cada uno.
 * @return true cuando estan todas. Si falla, ya llamo a fallar().
 */
static bool juntar_lecturas(void)
{
	laser_lectura_t l;

	switch (Laseres_Tick(&l)) {
	case LASER_NADA:
		return false;
	case LASER_LECTURA:
		break;
	default:
		fallar(SEQ_MOTIVO_SENSOR);
		return false;
	}
	if (!l.con_retorno) {
		fallar(SEQ_MOTIVO_SIN_OBJETO);
		return false;
	}
	lect_suma[l.sensor] += l.mm;
	lect_cant[l.sensor]++;

	for (int i = 0; i < VL53_CANT; i++) {
		if (VL53L0X_Listo((vl53_id_t)i) && lect_cant[i] < DIST_MUESTRAS) {
			return false;
		}
	}
	Laseres_Detener();
	return true;
}

/**
 * Promedio entre los sensores de lo que junto juntar_lecturas().
 * @param diferencia Salida: distancia entre lo que vio cada uno; 0 con un solo sensor.
 */
static float promediar_lecturas(float *diferencia)
{
	float suma = 0.0f;
	float menor = 0.0f;
	float mayor = 0.0f;
	int n = 0;

	for (int i = 0; i < VL53_CANT; i++) {
		if (lect_cant[i] > 0) {
			float d = lect_suma[i] / (float)lect_cant[i];

			menor = (n == 0 || d < menor) ? d : menor;
			mayor = (n == 0 || d > mayor) ? d : mayor;
			suma += d;
			n++;
		}
	}
	*diferencia = mayor - menor;
	return suma / (float)n;
}

/**
 * Vigila la medicion del objeto. Al completarla calcula hasta donde acercarse y cuanto bajar: la
 * distancia medida menos la que tiene que quedar con el objeto dentro de la garra.
 * @return true cuando termino bien. Si falla, ya llamo a fallar().
 */
static bool vigilar_distancia(void)
{
	float diferencia;
	float alto;

	if (!juntar_lecturas()) {
		return false;
	}

	float d = promediar_lecturas(&diferencia);
	float z_actual = EjeZ_AlturaComandada();

	/* Si los dos no ven lo mismo, uno quedo sobre la mesa: bajar ahi chocaria con la caja. */
	if (diferencia > CAJA_TOL_PAREJA_MM) {
		fallar(SEQ_MOTIVO_NO_ALINEADO);
		return false;
	}
	if (!(d >= DIST_OBJETO_MIN_MM && d <= DIST_OBJETO_MAX_MM)) {
		fallar(SEQ_MOTIVO_SIN_OBJETO);
		return false;
	}
	/* Si la mesa ya se conoce, una lectura a la altura de la mesa no es un objeto. */
	if (Busqueda_AlturaSobreMesa(z_actual, d, &alto) && alto < CAJA_UMBRAL_MM) {
		fallar(SEQ_MOTIVO_SIN_OBJETO);
		return false;
	}

	float z = z_actual - (d - PICK_DIST_TOMA_MM);

	if (!(z >= Z_MIN_MM && z <= Z_MAX_MM)) {
		fallar(SEQ_MOTIVO_OBJETO_LEJOS);
		return false;
	}
	z_toma = z;
	z_aprox = (d > PICK_DIST_APROX_MM) ? z_actual - paso.a * (d - PICK_DIST_APROX_MM) : z_actual;

	seq_evento_t *ev = nuevo_evento(SEQ_EV_DISTANCIA);

	ev->a = d;
	ev->b = z;
	return true;
}

/**
 * Mide la tapa de cerca, con la pinza en el centro: de lejos la altura sale de menos. Verifica que
 * los dos sensores la vean y publica la caja.
 * @return true cuando termino bien. Si falla, ya llamo a fallar().
 */
static bool vigilar_lectura_caja(void)
{
	float diferencia;
	float alto;

	if (!juntar_lecturas()) {
		return false;
	}

	float d = promediar_lecturas(&diferencia);

	if (!Busqueda_AlturaSobreMesa(EjeZ_AlturaComandada(), d, &alto) || alto < CAJA_UMBRAL_MM) {
		fallar(SEQ_MOTIVO_SIN_CAJA);
		return false;
	}

	busqueda_caja_t caja = {
		.x_mm = prm.x1, .y_mm = prm.y1, .yaw_deg = prm.yaw,
		.largo_mm = prm.largo_caja, .ancho_mm = prm.ancho_caja,
		.alto_mm = alto, .tipo = Mapa_TipoDeCaja(alto),
	};
	seq_evento_t *ev = nuevo_evento(SEQ_EV_CAJA);

	ev->caja = caja;
	if (diferencia > CAJA_TOL_PAREJA_MM) {
		fallar(SEQ_MOTIVO_NO_ALINEADO);
		return false;
	}
	if (fabsf(caja.largo_mm - CAJA_LARGO_MM) > CAJA_TOL_MEDIDA_MM
			|| fabsf(caja.ancho_mm - CAJA_ANCHO_MM) > CAJA_TOL_MEDIDA_MM) {
		fallar(SEQ_MOTIVO_CAJA_RARA);
		return false;
	}
	Busqueda_FijarCaja(&caja);
	return true;
}

/** Espera que termine el paso actual. */
static void vigilar_paso(void)
{
	switch (paso.tipo) {
	case PASO_XY:
	case PASO_JUNTAS:
	case PASO_HOME:
		if (Motion_Ocupado()) {
			return;
		}
		(void)Motion_TomarEvento();
		break;
	case PASO_Z:
	case PASO_Z_APROX:
	case PASO_Z_TOMA:
	case PASO_ORIENT:
		if (EjeZ_Ocupado()) {
			return;
		}
		if (EjeZ_TomarEvento() == EJEZ_EVENTO_ATASCO) {
			fallar(SEQ_MOTIVO_EJE_Z);
			return;
		}
		toma_hecha = toma_hecha || paso.tipo == PASO_Z_TOMA;
		break;
	case PASO_GRIPPER:
		if (Gripper_Ocupado()) {
			return;
		}
		(void)Gripper_TomarFin();
		break;
	case PASO_DIST:
		if (!vigilar_distancia()) {
			return;
		}
		break;
	case PASO_LEER_CAJA:
		if (!vigilar_lectura_caja()) {
			return;
		}
		break;
	case PASO_ESPERA:
		if (ms_espera > TICK_MS) {
			ms_espera -= TICK_MS;
			return;
		}
		break;
	default:
		break;
	}
	indice++;
	fase = FASE_PROXIMO;
}

/** La rutina completo todos sus pasos: lo que tomo o solto ya no esta donde estaba. */
static void terminar_rutina(void)
{
	if (generador == pasos_dejar || generador == pasos_ciclo) {
		toma_hecha = false;
	}
	if (generador == pasos_tomar && prm.usa_caja) {
		Busqueda_OlvidarCaja();
	}
	restaurar_velocidad();
	fase = FASE_INACTIVA;
	(void)nuevo_evento(SEQ_EV_FIN);
}

/** Arranca el proximo paso, o termina la rutina si no quedan. */
static void proximo_paso(void)
{
	/* Los pasos que no hacen nada no gastan un tick cada uno. */
	do {
		if (!generar(generador, indice, &paso)) {
			terminar_rutina();
			return;
		}
		if (paso.tipo == PASO_NADA) {
			indice++;
		}
	} while (paso.tipo == PASO_NADA);

	const char *nombre = nombre_paso(paso.tipo);

	if (nombre != NULL) {
		seq_evento_t *ev = nuevo_evento(SEQ_EV_PASO);

		ev->nombre = nombre;
		ev->a = paso.a;
		ev->b = paso.b;
	}

	switch (arrancar_paso(&paso)) {
	case ARRANQUE_LISTO:
		indice++;
		break;
	case ARRANQUE_CORRE:
		fase = FASE_EJECUTANDO;
		break;
	default:
		break;
	}
}

/** Espera que todo quede quieto despues de cortar, y da el aviso final. */
static void esperar_frenado(void)
{
	if (Motion_Ocupado() || EjeZ_Ocupado() || Gripper_Ocupado()) {
		return;
	}
	(void)Motion_TomarEvento();
	(void)EjeZ_TomarEvento();
	(void)Gripper_TomarFin();
	restaurar_velocidad();
	fase = FASE_INACTIVA;

	seq_evento_t *ev = nuevo_evento((motivo == SEQ_MOTIVO_NINGUNO) ? SEQ_EV_DETENIDA : SEQ_EV_FALLA);

	ev->motivo = motivo;
}

void Secuencia_Init(void)
{
	fase = FASE_INACTIVA;
	evento = (seq_evento_t){ .tipo = SEQ_EV_NINGUNO };
	toma_hecha = false;
	vel_cambiada = false;
	Busqueda_Init();
	for (int i = 0; i < SECUENCIA_PUNTOS_REPEAT; i++) {
		Metricas_Iniciar(&metricas[i]);
	}
}

void Secuencia_Tick(void)
{
	/* Una falla del brazo corta la rutina aunque el paso actual sea de otro eje. */
	if ((fase == FASE_PROXIMO || fase == FASE_EJECUTANDO) && Motion_Estado() == MOTION_FAULT) {
		fallar(SEQ_MOTIVO_BRAZO);
		return;
	}

	/* Mientras el brazo hace la pasada, los sensores siguen midiendo. */
	if (Busqueda_Muestreando()) {
		busqueda_res_t r = Busqueda_Tick();

		if (r != BUSQUEDA_OK) {
			fallar((r == BUSQUEDA_ERR_MUY_BAJO) ? SEQ_MOTIVO_MUY_BAJO : SEQ_MOTIVO_SENSOR);
			return;
		}
	}

	switch (fase) {
	case FASE_PROXIMO:
		proximo_paso();
		break;
	case FASE_EJECUTANDO:
		vigilar_paso();
		break;
	case FASE_FRENANDO:
		esperar_frenado();
		break;
	default:
		break;
	}
}

bool Secuencia_Activa(void)
{
	return fase != FASE_INACTIVA;
}

bool Secuencia_Abortar(void)
{
	if (fase == FASE_INACTIVA) {
		return false;
	}
	if (fase != FASE_FRENANDO) {
		fallar(SEQ_MOTIVO_NINGUNO);
	}
	return true;
}

bool Secuencia_TomarEvento(seq_evento_t *ev)
{
	if (ev == NULL || evento.tipo == SEQ_EV_NINGUNO) {
		return false;
	}
	*ev = evento;
	evento.tipo = SEQ_EV_NINGUNO;
	return true;
}

/* ---- Rutinas ------------------------------------------------------------------- */

seq_res_t Secuencia_MoverXYZ(float x_mm, float y_mm, float z_mm)
{
	prm.x1 = x_mm;
	prm.y1 = y_mm;
	prm.z = z_mm;
	prm.z_inicial = EjeZ_AlturaComandada();
	return arrancar(pasos_mover_xyz, UINT32_MAX);
}

/** Lo comun a las rutinas que empiezan con PICK. */
static void cargar_toma(float x_mm, float y_mm, float rot_deg, bool con_rot, float ancho_mm)
{
	prm.x1 = x_mm;
	prm.y1 = y_mm;
	prm.rot = rot_deg;
	prm.con_rot = con_rot;
	prm.ancho = ancho_mm;
	prm.usa_caja = false;
	z_toma = Z_TOMA_MM;
	z_aprox = Z_TOMA_MM;
	toma_hecha = false;
}

seq_res_t Secuencia_Tomar(float x_mm, float y_mm, float rot_deg, bool con_rot, float ancho_mm)
{
	cargar_toma(x_mm, y_mm, rot_deg, con_rot, ancho_mm);
	return arrancar(pasos_tomar, UINT32_MAX);
}

seq_res_t Secuencia_TomarCaja(void)
{
	busqueda_caja_t caja;

	if (!Busqueda_Caja(&caja)) {
		return SEQ_ERR_SIN_CAJA;
	}
	cargar_toma(caja.x_mm, caja.y_mm, caja.yaw_deg, true, GRIPPER_TOMA_MM);
	prm.usa_caja = true;
	return arrancar(pasos_tomar, UINT32_MAX);
}

seq_res_t Secuencia_Buscar(float x_mm, float y_mm, float ancho_mm, float alto_mm)
{
	if (fase != FASE_INACTIVA) {
		return SEQ_ERR_OCUPADO;
	}
	if (Laseres_Listos() < VL53_CANT) {
		return SEQ_ERR_SENSOR;
	}

	/* Con los lados negativos, el origen es otra esquina: se pasa a la de menores X e Y. */
	prm.area_x = (ancho_mm < 0.0f) ? x_mm + ancho_mm : x_mm;
	prm.area_y = (alto_mm < 0.0f) ? y_mm + alto_mm : y_mm;
	prm.area_ancho = fabsf(ancho_mm);
	prm.area_alto = fabsf(alto_mm);
	if (!Mapa_Iniciar(prm.area_x, prm.area_y, prm.area_ancho, prm.area_alto)) {
		return SEQ_ERR_PARAM;
	}
	prm.filas = (uint32_t)Mapa_Filas();

	/* El centrado depende de donde este la caja: se valida cuando se sepa, antes de ir hacia ella. */
	return arrancar(pasos_buscar, PASOS_ANTES_DEL_BARRIDO + tramos_barrido());
}

seq_res_t Secuencia_Dejar(float x_mm, float y_mm)
{
	prm.x2 = x_mm;
	prm.y2 = y_mm;
	return arrancar(pasos_dejar, UINT32_MAX);
}

seq_res_t Secuencia_Ciclo(float x1_mm, float y1_mm, float x2_mm, float y2_mm,
						  float rot_deg, bool con_rot, float ancho_mm)
{
	cargar_toma(x1_mm, y1_mm, rot_deg, con_rot, ancho_mm);
	prm.x2 = x2_mm;
	prm.y2 = y2_mm;
	return arrancar(pasos_ciclo, UINT32_MAX);
}

seq_res_t Secuencia_Lapiz(bool abajo)
{
	prm.lapiz_abajo = abajo;
	return arrancar(pasos_lapiz, UINT32_MAX);
}

seq_res_t Secuencia_DibujarRect(float x_mm, float y_mm, float ancho_mm, float alto_mm)
{
	prm.x1 = x_mm;
	prm.y1 = y_mm;
	prm.lado_x = ancho_mm;
	prm.lado_y = alto_mm;
	return arrancar(pasos_rect, UINT32_MAX);
}

seq_res_t Secuencia_DibujarCirculo(float x_mm, float y_mm, float r_mm)
{
	if (!(r_mm > 0.0f)) {
		return SEQ_ERR_PARAM;
	}

	float cuerdas = ceilf(2.0f * PI_F * r_mm / DRAW_CUERDA_MM);

	if (cuerdas > (float)DRAW_CUERDAS_MAX) {
		return SEQ_ERR_PARAM;
	}
	prm.x1 = x_mm;
	prm.y1 = y_mm;
	prm.radio = r_mm;
	prm.cuerdas = (cuerdas < 3.0f) ? 3U : (uint32_t)cuerdas;
	return arrancar(pasos_circulo, UINT32_MAX);
}

seq_res_t Secuencia_Repetir(float x1_mm, float y1_mm, float x2_mm, float y2_mm, int32_t ciclos)
{
	if (ciclos < 1 || ciclos > REPEAT_MAX_CICLOS) {
		return SEQ_ERR_PARAM;
	}
	if (Motion_CantEncoders() == 0) {
		return SEQ_ERR_ENCODER;
	}
	prm.x1 = x1_mm;
	prm.y1 = y1_mm;
	prm.x2 = x2_mm;
	prm.y2 = y2_mm;
	prm.ciclos = (uint32_t)ciclos;

	/* Basta validar dos ciclos: ir a P1, ir a P2, volver a P1 y volver a P2 ya son todos los tramos. */
	seq_res_t r = arrancar(pasos_repetir, 2U * PASOS_REPEAT_POR_CICLO);

	if (r == SEQ_OK) {
		for (int i = 0; i < SECUENCIA_PUNTOS_REPEAT; i++) {
			Metricas_Iniciar(&metricas[i]);
		}
	}
	return r;
}

bool Secuencia_ResultadoRepetir(int punto, metricas_res_t *r, float *e_resolucion)
{
	if (punto < 0 || punto >= SECUENCIA_PUNTOS_REPEAT || r == NULL || e_resolucion == NULL
			|| metricas[punto].n == 0U) {
		return false;
	}
	Metricas_Resultado(&metricas[punto], r);
	*e_resolucion = Cinematica_ErrorPorJuntas(postura_medida[punto][JUNTA_HOMBRO],
											  postura_medida[punto][JUNTA_CODO],
											  ENC_RESOLUCION_DEG, ENC_RESOLUCION_DEG);
	return true;
}

seq_res_t Secuencia_Calibrar(int32_t junta)
{
	if (junta < 0 || junta > JUNTA_CANT) {
		return SEQ_ERR_PARAM;
	}
	prm.junta_ini = (junta == 0) ? 0 : (int)junta - 1;
	prm.juntas = (junta == 0) ? JUNTA_CANT : 1;
	Recta_Iniciar(&recta);
	calib_cant = 0;
	return arrancar(pasos_calibrar, UINT32_MAX);
}
