/**
 * @file secuencia.h
 * @brief Rutinas de varios pasos: MOVE con Z, SCAN, PICK, PLACE, PICKPLACE, PEN, DRAW, REPEAT y CALIB.
 *
 * Una rutina es una lista de pasos (mover XY, mover Z, orientar, mover la garra, medir, esperar)
 * que se ejecutan de a uno, cada uno cuando termina el anterior. Antes de mover nada se recorre
 * la lista entera y se validan todos los tramos: si algo no se puede hacer, la rutina se rechaza
 * con el brazo quieto, en vez de descubrirlo a mitad de camino.
 *
 * Mientras hay una rutina activa, ella consume los eventos de motion, eje_z y gripper.
 */
#ifndef SECUENCIA_H
#define SECUENCIA_H

#include <stdbool.h>
#include <stdint.h>

#include "busqueda.h"
#include "mapa.h"
#include "metricas.h"

#define SECUENCIA_PUNTOS_REPEAT		2

/** Resultado de pedir una rutina. Salvo SEQ_OK, nada se movio. */
typedef enum {
	SEQ_OK = 0,
	SEQ_ERR_OCUPADO,		/**< Hay algo moviendose */
	SEQ_ERR_APAGADO,		/**< Servos sin habilitar: falta el primer HOME */
	SEQ_ERR_FALLA,			/**< Brazo en FAULT */
	SEQ_ERR_RADIO,			/**< Un tramo sale del espacio de trabajo */
	SEQ_ERR_LIMITE,			/**< Un tramo sale de los limites articulares */
	SEQ_ERR_Z_RANGO,		/**< Altura u orientacion fuera de limites */
	SEQ_ERR_Z_SIN_CERO,		/**< Falta SETZ0 */
	SEQ_ERR_GRIPPER_RANGO,	/**< Apertura fuera de limites */
	SEQ_ERR_ENCODER,		/**< La rutina necesita un encoder que no esta */
	SEQ_ERR_SENSOR,			/**< La rutina necesita un sensor de distancia y no contesta */
	SEQ_ERR_SIN_CAJA,		/**< PICK sin coordenadas, y no hay una caja encontrada por SCAN */
	SEQ_ERR_PARAM			/**< Argumento invalido */
} seq_res_t;

/** Por que se corto una rutina en ejecucion. */
typedef enum {
	SEQ_MOTIVO_NINGUNO = 0,
	SEQ_MOTIVO_BRAZO,			/**< El brazo fallo o rechazo un tramo */
	SEQ_MOTIVO_EJE_Z,			/**< El eje Z se atasco o rechazo un movimiento */
	SEQ_MOTIVO_GRIPPER,			/**< La garra rechazo el movimiento */
	SEQ_MOTIVO_SENSOR,			/**< El sensor de distancia fallo */
	SEQ_MOTIVO_SIN_OBJETO,		/**< El sensor no ve un objeto en el rango esperado */
	SEQ_MOTIVO_OBJETO_LEJOS,	/**< Para llegar al objeto el eje Z saldria de sus limites */
	SEQ_MOTIVO_ENCODER,			/**< No se pudo leer un encoder */
	SEQ_MOTIVO_NO_ALINEADO,		/**< Los dos sensores no ven lo mismo */
	SEQ_MOTIVO_SIN_CAJA,		/**< El barrido no encontro ninguna caja */
	SEQ_MOTIVO_MUY_BAJO,		/**< Un sensor quedo mas cerca que BARRIDO_DIST_MIN_MM */
	SEQ_MOTIVO_CENTRADO,		/**< Las pasadas lentas no encontraron los bordes de la caja */
	SEQ_MOTIVO_CENTRADO_LEJOS,	/**< Las pasadas lentas salen del espacio de trabajo */
	SEQ_MOTIVO_CAJA_RARA		/**< Lo medido no coincide con las medidas de la caja */
} seq_motivo_t;

/** Tipo de aviso. */
typedef enum {
	SEQ_EV_NINGUNO = 0,
	SEQ_EV_PASO,			/**< Empezo un paso: nombre, a, b */
	SEQ_EV_DISTANCIA,		/**< Objeto detectado: a = distancia, b = altura de toma */
	SEQ_EV_MEDIDA,			/**< REPEAT: una llegada medida */
	SEQ_EV_CALIB_PUNTO,		/**< CALIB: un punto medido */
	SEQ_EV_CALIB_AJUSTE,	/**< CALIB: resultado de una junta */
	SEQ_EV_MAPA,			/**< SCAN: termino el barrido. mancha, cantidad, a = distancia a la mesa */
	SEQ_EV_CENTRADO,		/**< SCAN: eje de la caja. x_cmd, y_cmd, a = direccion, b = ancho */
	SEQ_EV_CAJA,			/**< SCAN: la caja medida. caja */
	SEQ_EV_FIN,				/**< La rutina termino bien */
	SEQ_EV_DETENIDA,		/**< Termino de frenar tras un STOP */
	SEQ_EV_FALLA			/**< Se corto: motivo, paso */
} seq_ev_tipo_t;

/** Aviso de una rutina. Solo son validos los campos del tipo indicado. */
typedef struct {
	seq_ev_tipo_t tipo;
	uint32_t paso;			/**< Indice del paso actual */
	const char *nombre;		/**< SEQ_EV_PASO */
	float a, b;				/**< SEQ_EV_PASO, SEQ_EV_DISTANCIA */
	seq_motivo_t motivo;	/**< SEQ_EV_FALLA */

	uint32_t ciclo;			/**< SEQ_EV_MEDIDA */
	int punto;				/**< SEQ_EV_MEDIDA */
	float x_cmd, y_cmd;		/**< SEQ_EV_MEDIDA */
	float x_med, y_med;		/**< SEQ_EV_MEDIDA */
	float e;				/**< SEQ_EV_MEDIDA: error de esta llegada (mm) */

	int junta;				/**< SEQ_EV_CALIB_*: 1 hombro, 2 codo */
	float cmd_deg;			/**< SEQ_EV_CALIB_PUNTO: angulo comandado */
	float enc_deg;			/**< SEQ_EV_CALIB_PUNTO: lectura del encoder */
	float signo;			/**< SEQ_EV_CALIB_AJUSTE: valor sugerido para ENCx_SIGN */
	float cero_deg;			/**< SEQ_EV_CALIB_AJUSTE: valor sugerido para ENCx_ZERO_DEG */
	float ganancia;			/**< SEQ_EV_CALIB_AJUSTE: grados medidos por grado comandado */
	float rango_deg;		/**< SEQ_EV_CALIB_AJUSTE: valor sugerido para el RANGE_DEG del servo */
	float zero_servo_deg;	/**< SEQ_EV_CALIB_AJUSTE: ZERO_SERVO_DEG que conserva el pulso de theta = 0 */
	float residuo_deg;		/**< SEQ_EV_CALIB_AJUSTE: peor desvio respecto de la recta */

	mapa_mancha_t mancha;	/**< SEQ_EV_MAPA: la mayor mancha, si cantidad > 0 */
	int cantidad;			/**< SEQ_EV_MAPA: manchas encontradas */
	busqueda_caja_t caja;	/**< SEQ_EV_CAJA */
} seq_evento_t;

/** @brief Estado inicial, sin rutina activa. */
void Secuencia_Init(void);

/**
 * @brief Tick de la rutina. Llamar cada TICK_MS, despues de los ticks de motion, eje_z y gripper.
 */
void Secuencia_Tick(void);

/** @brief true mientras hay una rutina en curso, incluido su frenado final. */
bool Secuencia_Activa(void);

/**
 * @brief STOP: frena todo con rampa y corta la rutina.
 * @return false si no habia rutina activa (no hizo nada).
 */
bool Secuencia_Abortar(void);

/**
 * @brief Devuelve (una sola vez) el aviso pendiente.
 * @param ev Salida.
 * @return false si no hay ningun aviso.
 */
bool Secuencia_TomarEvento(seq_evento_t *ev);

/**
 * @brief MOVE X Y Z. Si Z sube, primero sube y despues se traslada; si baja, primero se traslada
 *        y despues baja. Asi el traslado siempre ocurre a la mayor de las dos alturas.
 * @param x_mm,y_mm Destino en el plano (mm).
 * @param z_mm Altura de destino (mm).
 */
seq_res_t Secuencia_MoverXYZ(float x_mm, float y_mm, float z_mm);

/**
 * @brief PICK: sube, se traslada, orienta, abre, se acerca de a poco midiendo el objeto, baja,
 *        cierra y sube.
 * @param x_mm,y_mm Posicion del objeto (mm).
 * @param rot_deg Direccion en la mesa de la linea de sensores, desde +X: la del lado largo de la
 *                caja (grados). Se ignora si @p con_rot es false.
 * @param con_rot false para dejar la orientacion como esta.
 * @param ancho_mm Apertura con la que sujetar (mm).
 */
seq_res_t Secuencia_Tomar(float x_mm, float y_mm, float rot_deg, bool con_rot, float ancho_mm);

/**
 * @brief PICK sobre la caja que encontro el ultimo SCAN. Si termina bien, esa caja se olvida.
 * @return SEQ_ERR_SIN_CAJA si no hay ninguna vigente.
 */
seq_res_t Secuencia_TomarCaja(void);

/**
 * @brief SCAN: barre un area con los dos sensores, ubica la caja, la cruza con pasadas lentas,
 *        baja con la garra abierta hasta PICK_DIST_APROX_MM de la tapa para medirle la altura y
 *        vuelve a subir. No la toca.
 *
 * Antes de mover valida el barrido entero. Antes de ir hacia la caja, valida las pasadas lentas.
 * Necesita los dos sensores. Barre a Z_BARRIDO_MM, con velocidades propias: al terminar, SPEED
 * vuelve a lo que estaba.
 * @param x_mm,y_mm Una esquina del area, respecto de la base (mm).
 * @param ancho_mm,alto_mm Lados en X y en Y desde esa esquina (mm), con signo y distintos de 0.
 */
seq_res_t Secuencia_Buscar(float x_mm, float y_mm, float ancho_mm, float alto_mm);

/**
 * @brief PLACE: sube, se traslada, baja, abre y sube.
 *
 * Si antes hubo un PICK, baja a la altura de esa toma: la caja queda apoyada igual, sea del alto
 * que sea. Si no, baja a Z_DEJAR_MM.
 * @param x_mm,y_mm Posicion donde soltar (mm).
 */
seq_res_t Secuencia_Dejar(float x_mm, float y_mm);

/**
 * @brief PICKPLACE: el diagrama completo del Anexo B de las pautas (PICK + PLACE + volver a HOME).
 * @param x1_mm,y1_mm Posicion del objeto (mm).
 * @param x2_mm,y2_mm Posicion donde soltarlo (mm).
 * @param rot_deg Direccion en la mesa de la linea de sensores (grados); se ignora si @p con_rot es false.
 * @param con_rot false para dejar la orientacion como esta.
 * @param ancho_mm Apertura con la que sujetar (mm).
 */
seq_res_t Secuencia_Ciclo(float x1_mm, float y1_mm, float x2_mm, float y2_mm,
						  float rot_deg, bool con_rot, float ancho_mm);

/**
 * @brief PEN: baja o levanta el marcador.
 * @param abajo true apoya el marcador; false lo levanta.
 */
seq_res_t Secuencia_Lapiz(bool abajo);

/**
 * @brief DRAW RECT: rectangulo con el marcador, desde una esquina.
 * @param x_mm,y_mm Esquina inicial (mm).
 * @param ancho_mm,alto_mm Lados en X y en Y (mm), con signo.
 */
seq_res_t Secuencia_DibujarRect(float x_mm, float y_mm, float ancho_mm, float alto_mm);

/**
 * @brief DRAW CIRCLE: circulo con el marcador, en tramos rectos de DRAW_CUERDA_MM.
 * @param x_mm,y_mm Centro (mm).
 * @param r_mm Radio (mm), positivo.
 */
seq_res_t Secuencia_DibujarCirculo(float x_mm, float y_mm, float r_mm);

/**
 * @brief REPEAT: va y vuelve entre dos puntos midiendo el error de cada llegada con los encoders.
 *
 * Necesita al menos un encoder habilitado. Con uno solo, el error mide solo esa junta.
 * @param x1_mm,y1_mm,x2_mm,y2_mm Los dos puntos (mm).
 * @param ciclos Idas y vueltas, de 1 a REPEAT_MAX_CICLOS.
 */
seq_res_t Secuencia_Repetir(float x1_mm, float y1_mm, float x2_mm, float y2_mm, int32_t ciclos);

/**
 * @brief Resultado de REPEAT para uno de los dos puntos.
 * @param punto 0 o 1.
 * @param r Salida: estadisticas de las llegadas.
 * @param e_resolucion Salida: menor error que distingue la resolucion del encoder en ese punto (mm).
 * @return false si el indice no es valido o ese punto no tiene llegadas.
 */
bool Secuencia_ResultadoRepetir(int punto, metricas_res_t *r, float *e_resolucion);

/**
 * @brief CALIB: recorre CALIB_PASOS_DEG alrededor de HOME en una junta o en las dos y ajusta una
 *        recta entre lo comandado y la lectura del encoder. No exige ENCx_HABILITADO.
 * @param junta 1 hombro, 2 codo, 0 las dos.
 */
seq_res_t Secuencia_Calibrar(int32_t junta);

#endif /* SECUENCIA_H */
