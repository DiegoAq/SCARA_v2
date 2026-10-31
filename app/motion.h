/**
 * @file motion.h
 * @brief Movimiento del brazo: estado, tick de control de 20 ms, supervision con encoders y
 *        comandos HOME/JOINT/PULSE/MOVE/STOP.
 *
 * LAZO ABIERTO SUPERVISADO: los servos se comandan sin realimentacion. Cada junta con encoder
 * habilitado (ENCx_HABILITADO en config.h) arranca desde lo medido y se supervisa en cada tick;
 * una junta sin encoder se supone en HOME al arrancar y el primer Motion_Home() la habilita en
 * esa postura. A partir de ahi todo movimiento sale de la ultima postura comandada y ninguna
 * junta se mueve a mas de JOINT_VMAX_DPS.
 */
#ifndef MOTION_H
#define MOTION_H

#include <stdbool.h>
#include <stdint.h>

#include "as5600.h"

/** Estados del brazo (SPEC 7.2). */
typedef enum {
	MOTION_IDLE = 0,
	MOTION_MOVING,
	MOTION_STOPPING,
	MOTION_FAULT
} motion_estado_t;

/** Juntas del brazo. */
typedef enum {
	JUNTA_HOMBRO = 0,	/**< theta1, encoder en I2C1 */
	JUNTA_CODO,			/**< theta2, encoder en I2C2 */
	JUNTA_CANT
} junta_id_t;

/** Resultado de un comando de movimiento. */
typedef enum {
	MOTION_OK = 0,				/**< Aceptado; el fin se avisa con un evento */
	MOTION_YA_EN_DESTINO,		/**< No hay nada que mover */
	MOTION_SERVOS_HABILITADOS,	/**< Primer HOME: se habilito el PWM ya en la postura de HOME */
	MOTION_SERVOS_SINCRONIZADOS,/**< Primer HOME: se habilito el PWM en lo medido y va hacia HOME */
	MOTION_ERR_RADIO,			/**< Destino o tramo fuera del espacio de trabajo */
	MOTION_ERR_LIMITE,			/**< Limite articular */
	MOTION_ERR_OCUPADO,			/**< Hay un movimiento en curso */
	MOTION_ERR_APAGADO,			/**< Servos sin habilitar: falta el primer HOME */
	MOTION_ERR_FALLA,			/**< Brazo en FAULT: hay que rearmarlo con HOME */
	MOTION_ERR_ENCODER,			/**< Un encoder habilitado no da una posicion confiable */
	MOTION_ERR_PARAM			/**< Argumento invalido */
} motion_res_t;

/** Avisos asincronos, para que la consola los informe. */
typedef enum {
	MOTION_EVENTO_NINGUNO = 0,
	MOTION_EVENTO_FIN,			/**< El movimiento llego a destino */
	MOTION_EVENTO_DETENIDO,		/**< Termino de frenar tras un STOP */
	MOTION_EVENTO_FALLA,		/**< El planificador aborto; el brazo quedo donde estaba */
	MOTION_EVENTO_FALLA_ENCODER	/**< Error o falta de lectura sostenidos; el brazo quedo donde estaba */
} motion_evento_t;

/**
 * @brief Parametros por defecto, estado IDLE, servos sin pulsos y encoders inicializados.
 *        Llamar despues de SysTick_Init() (el I2C usa timeouts por Millis).
 */
void Motion_Init(void);

/**
 * @brief Tick de control: avanza un paso el movimiento en curso y supervisa los encoders.
 *
 * Llamar cada TICK_MS desde el loop principal (nunca desde una ISR). Si el loop se atrasa
 * NO hay que llamarlo varias veces seguidas para "recuperar": cada llamada avanza el brazo
 * un paso completo.
 */
void Motion_Tick(void);

/**
 * @brief HOME. La primera vez, y para salir de FAULT, habilita el PWM: cada junta con encoder
 *        desde lo medido (sin salto), y cada junta sin encoder en HOME (salta hasta ahi si
 *        estaba en otro lado). Despues va a HOME a JOINT_MOVE_DPS.
 * @return MOTION_SERVOS_HABILITADOS, MOTION_SERVOS_SINCRONIZADOS, MOTION_OK,
 *         MOTION_YA_EN_DESTINO, MOTION_ERR_OCUPADO o MOTION_ERR_ENCODER.
 */
motion_res_t Motion_Home(void);

/**
 * @brief Mueve las dos juntas a los angulos pedidos, sincronizadas, a JOINT_MOVE_DPS.
 * @param th1_deg,th2_deg Angulos de junta (grados), dentro de los limites articulares.
 */
motion_res_t Motion_MoverJuntas(float th1_deg, float th2_deg);

/**
 * @brief Mueve la pinza en linea recta hasta (x, y) con el perfil SPEED/ACCEL vigente.
 *
 * Valida todo el segmento antes de mover (pre-vuelo). Si falla, el brazo no se mueve.
 * @param x_mm,y_mm Destino (mm).
 */
motion_res_t Motion_MoverLineal(float x_mm, float y_mm);

/**
 * @brief Lleva un servo a un pulso crudo, a JOINT_MOVE_DPS (diagnostico y calibracion).
 *
 * No mira los limites articulares: sirve justamente para encontrarlos. Usar con cuidado.
 * @param junta Servo a mover.
 * @param pulso_us Pulso destino; se satura a [PULSE_MIN_US, PULSE_MAX_US].
 */
motion_res_t Motion_MoverPulso(junta_id_t junta, uint32_t pulso_us);

/**
 * @brief STOP. Frena con rampa: un MOVE sobre la recta y un movimiento articular sobre su camino.
 */
void Motion_Detener(void);

/**
 * @brief Devuelve (una sola vez) el aviso pendiente, o MOTION_EVENTO_NINGUNO.
 */
motion_evento_t Motion_TomarEvento(void);

/** @brief Estado actual del brazo. */
motion_estado_t Motion_Estado(void);

/** @brief true mientras hay un movimiento en curso (incluye la espera final antes de medir). */
bool Motion_Ocupado(void);

/** @brief Nombre del estado para la consola ("IDLE", "MOVING", ...). */
const char *Motion_NombreEstado(motion_estado_t estado);

/** @brief true si ya se hizo el primer HOME y los servos reciben pulsos. */
bool Motion_ServosActivos(void);

/**
 * @brief Ultima postura comandada (grados). Antes del primer HOME es la postura supuesta.
 */
void Motion_PosturaComandada(float *th1_deg, float *th2_deg);

/**
 * @brief Postura que se quiso alcanzar (grados). Es la comandada, salvo mientras el trim corre
 *        lo comandado para compensar un error: ahi esta sigue siendo el destino pedido.
 */
void Motion_PosturaObjetivo(float *th1_deg, float *th2_deg);

/**
 * @brief Ultimo pulso cargado a cada servo (us). Es lo que sale por PA6/PA7 cuando los
 *        servos estan habilitados: 500 = 0 de servo, 1500 = 90, 2500 = 180.
 */
void Motion_PulsosComandados(uint32_t *hombro_us, uint32_t *codo_us);

/**
 * @brief Fija la velocidad de crucero de MOVE.
 * @param v_mm_s Velocidad en mm/s, dentro de [SPEED_MIN_MM_S, SPEED_MAX_MM_S].
 * @return false (y no cambia nada) si esta fuera de rango o no es un numero.
 */
bool Motion_ConfigurarVelocidad(float v_mm_s);

/** @brief Velocidad de crucero vigente (mm/s). */
float Motion_Velocidad(void);

/**
 * @brief Fija la aceleracion (y frenado) de MOVE.
 * @param a_mm_s2 Aceleracion en mm/s2, dentro de [ACCEL_MIN_MM_S2, ACCEL_MAX_MM_S2].
 * @return false (y no cambia nada) si esta fuera de rango o no es un numero.
 */
bool Motion_ConfigurarAceleracion(float a_mm_s2);

/** @brief Aceleracion vigente (mm/s2). */
float Motion_Aceleracion(void);

/**
 * @brief Mayor intervalo medido entre dos ticks (ms). Idealmente TICK_MS.
 *
 * Si algo bloquea el loop, el tick se atrasa y esto lo muestra.
 * @param reiniciar true para empezar una medicion nueva despues de leer.
 */
uint32_t Motion_PeriodoTickMaxMs(bool reiniciar);

/**
 * @brief Lee ahora la posicion real de una junta con su AS5600, este habilitado o no.
 *
 * Aplica ENCx_ZERO_DEG y ENCx_SIGN de config.h. No verifica el iman.
 * @param junta Junta a leer.
 * @param enc_deg Salida opcional (puede ser NULL): lectura cruda del encoder, 0..360 grados.
 * @param theta_deg Salida opcional (puede ser NULL): angulo de junta, en la vuelta centrada en
 *                  su rango.
 * @return false si el encoder no contesta.
 */
bool Motion_LeerJuntaMedida(junta_id_t junta, float *enc_deg, float *theta_deg);

/**
 * @brief Lectura cruda del encoder de una junta, para diagnostico (comando RAW).
 * @param cuentas Salida: 0..4095.
 * @return false si el encoder no contesta.
 */
bool Motion_LeerEncoderCrudo(junta_id_t junta, uint16_t *cuentas);

/**
 * @brief Estado del iman del encoder de una junta.
 */
as5600_iman_t Motion_EstadoIman(junta_id_t junta);

/** @brief Cantidad de juntas con encoder habilitado en config.h. */
int Motion_CantEncoders(void);

/**
 * @brief Medicion del ultimo tick de una junta con encoder habilitado.
 * @param th_deg Salida opcional: angulo medido (grados).
 * @param err_deg Salida opcional: objetivo - medido, en (-180, 180] (grados).
 * @return false si el encoder no esta habilitado o el ultimo tick no dejo una lectura valida.
 */
bool Motion_Medida(junta_id_t junta, float *th_deg, float *err_deg);

/**
 * @brief Mejor estimacion de la postura real: lo medido donde hay lectura valida, lo comandado
 *        en las demas juntas.
 * @return Cantidad de juntas que aportaron una medicion real (0, 1 o 2).
 */
int Motion_PosturaReal(float *th1_deg, float *th2_deg);

#endif /* MOTION_H */
