/**
 * @file config.h
 * @brief TODOS los parametros fisicos y de calibracion del robot SCARA.
 *
 * Unica fuente de numeros del proyecto: longitudes, offsets, pulsos, limites y
 * velocidades. Ningun otro archivo lleva numeros magicos. Este header es parte del
 * nucleo portable: no incluye nada de hardware y compila con gcc en la PC.
 *
 * Unidades: mm, grados, segundos (salvo que el nombre diga otra cosa).
 * Los valores marcados [CONFIRMAR] dependen de una medicion o decision del equipo.
 */
#ifndef CONFIG_H
#define CONFIG_H

/* ------------------------------------------------------------------------- */
/* Geometria                                                                 */
/* ------------------------------------------------------------------------- */

/* [CONFIRMADO 2026-10-03] L1 = L2 = 200mm, centro a centro de eje. */
#define L1_MM						200.0f	/**< Eslabon 1: eje hombro -> eje codo */
#define L2_MM						200.0f	/**< Eslabon 2: eje codo -> efector */

/* ------------------------------------------------------------------------- */
/* Mapeo servo <-> junta (SPEC 3.1)                                          */
/*   theta = SIGN * (servo - ZERO)        servo = ZERO + SIGN * theta        */
/* ------------------------------------------------------------------------- */

#define SERVO_RANGE_DEG				180.0f	/**< [CONFIRMAR] recorrido real del servo entre PULSE_MIN y PULSE_MAX */
#define PULSE_MIN_US				500		/**< [CONFIRMAR] calibrar por servo */
#define PULSE_MAX_US				2500	/**< [CONFIRMAR] calibrar por servo */

/* Montaje actual (dato del equipo, 2026-10-04): con LOS DOS servos en 90 el brazo queda
 * extendido y la pinza en X = 0, Y = 400. O sea: hombro a 90 de servo => theta1 = 90 (apunta
 * a +Y); codo a 90 de servo => theta2 = 0 (brazo recto).
 *
 * [CONFIRMAR con el brazo, prueba de sentidos del README] Los SIGNOS son un supuesto: +1 = al
 * subir el angulo de servo la junta gira en sentido antihorario visto desde arriba. Si un
 * servo gira al reves, cambiar su SIGN a (-1.0f) y su ZERO segun indica el README. */
#define SHOULDER_ZERO_SERVO_DEG		0.0f	/**< Angulo de servo con el brazo sobre +X (theta1 = 0) */
#define SHOULDER_SIGN				1.0f	/**< [CONFIRMAR] +1 o -1: sentido del servo respecto del antihorario */
#define SHOULDER_SERVO_MIN_DEG		0.0f	/**< [CONFIRMAR] tope mecanico inferior, en angulo de servo */
#define SHOULDER_SERVO_MAX_DEG		180.0f	/**< [CONFIRMAR] tope mecanico superior, en angulo de servo */

#define ELBOW_ZERO_SERVO_DEG		90.0f	/**< Angulo de servo con el brazo recto (theta2 = 0) */
#define ELBOW_SIGN					1.0f	/**< [CONFIRMAR] */
#define ELBOW_SERVO_MIN_DEG			0.0f	/**< [CONFIRMAR] tope mecanico, en angulo de servo */
#define ELBOW_SERVO_MAX_DEG			180.0f	/**< [CONFIRMAR] tope mecanico, en angulo de servo */

/* Postura de HOME y de arranque. SIN ENCODERS el firmware no sabe donde esta el brazo: supone
 * que esta en HOME. El primer HOME habilita el PWM directamente en esta postura; si el brazo
 * estaba en otro lado, el servo va de golpe hasta aca (ver README). */
#define HOME_TH1_DEG				90.0f	/**< Hombro a 90 de servo */
#define HOME_TH2_DEG				0.0f	/**< Codo a 90 de servo (brazo recto) */

/* Limites articulares: se DERIVAN de los limites de servo; no editar a mano. */
#define CFG_MIN(a, b)				(((a) < (b)) ? (a) : (b))
#define CFG_MAX(a, b)				(((a) > (b)) ? (a) : (b))
#define CFG_JUNTA(sign, zero, servo)	((sign) * ((servo) - (zero)))

#define JOINT1_MIN_DEG	CFG_MIN(CFG_JUNTA(SHOULDER_SIGN, SHOULDER_ZERO_SERVO_DEG, SHOULDER_SERVO_MIN_DEG), \
								CFG_JUNTA(SHOULDER_SIGN, SHOULDER_ZERO_SERVO_DEG, SHOULDER_SERVO_MAX_DEG))
#define JOINT1_MAX_DEG	CFG_MAX(CFG_JUNTA(SHOULDER_SIGN, SHOULDER_ZERO_SERVO_DEG, SHOULDER_SERVO_MIN_DEG), \
								CFG_JUNTA(SHOULDER_SIGN, SHOULDER_ZERO_SERVO_DEG, SHOULDER_SERVO_MAX_DEG))
#define JOINT2_MIN_DEG	CFG_MIN(CFG_JUNTA(ELBOW_SIGN, ELBOW_ZERO_SERVO_DEG, ELBOW_SERVO_MIN_DEG), \
								CFG_JUNTA(ELBOW_SIGN, ELBOW_ZERO_SERVO_DEG, ELBOW_SERVO_MAX_DEG))
#define JOINT2_MAX_DEG	CFG_MAX(CFG_JUNTA(ELBOW_SIGN, ELBOW_ZERO_SERVO_DEG, ELBOW_SERVO_MIN_DEG), \
								CFG_JUNTA(ELBOW_SIGN, ELBOW_ZERO_SERVO_DEG, ELBOW_SERVO_MAX_DEG))

/** Holgura numerica de los limites articulares: una postura exactamente sobre el
 *  tope no debe rechazarse por el redondeo de FK/IK en float. */
#define JOINT_LIM_TOL_DEG			0.01f

/* ------------------------------------------------------------------------- */
/* Espacio de trabajo operativo                                              */
/* ------------------------------------------------------------------------- */

/* Con el codo recto en 90 de servo y 180 de recorrido, theta2 llega a +-90: el radio minimo
 * geometrico es 282,8 mm. Se deja un margen. Ajustar si cambia el montaje del codo. */
#define R_MIN_OP_MM					285.0f	/**< Radio minimo operativo */
#define R_MAX_OP_MM					395.0f	/**< Margen contra la singularidad de brazo estirado */

/** |cos(theta2)| puede pasarse de 1 por redondeo; mas alla de esto es inalcanzable. */
#define IK_EPS_C2					1.0e-4f

/* ------------------------------------------------------------------------- */
/* Movimiento                                                                */
/* ------------------------------------------------------------------------- */

#define TICK_MS						20		/**< Periodo de control (50 Hz, alineado con el servo) */

#define SPEED_DEFAULT_MM_S			30.0f
#define SPEED_MIN_MM_S				1.0f	/**< Tope inferior del comando SPEED */
/* [CONFIRMADO 2026-10-03] bajado de 150 a 50 mm/s para las primeras pruebas fisicas;
 * subir una vez validado el comportamiento real del brazo armado. */
#define SPEED_MAX_MM_S				50.0f	/**< Tope superior del comando SPEED */
#define ACCEL_DEFAULT_MM_S2			80.0f
#define ACCEL_MIN_MM_S2				5.0f	/**< Tope inferior del comando ACCEL */
/* [CONFIRMADO 2026-10-03] bajado de 500 a 150 mm/s2 para las primeras pruebas fisicas;
 * subir una vez validado el comportamiento real del brazo armado. */
#define ACCEL_MAX_MM_S2				150.0f	/**< Tope superior del comando ACCEL */

/** Tope de velocidad por junta. Red de seguridad contra rotura de servos: nada
 *  (MOVE, JOINT, PULSE, HOME, arranque) mueve una junta mas rapido que esto. */
#define JOINT_VMAX_DPS				60.0f
/** Velocidad de los movimientos articulares (JOINT, HOME, PULSE). Debe ser <= JOINT_VMAX_DPS. */
#define JOINT_MOVE_DPS				30.0f

#define PREFLIGHT_STEP_MM			2.0f	/**< Paso de muestreo del segmento en el pre-vuelo */
#define MOVE_DIST_MIN_MM			0.1f	/**< Por debajo de esto el MOVE responde OK sin moverse */

/** Maxima diferencia por junta entre IK(P0) y la postura actual para considerar que
 *  el brazo ya esta en esa rama de codo (solo se cumple en ambas cerca de theta2 = 0). */
#define PLAN_TOL_RAMA_DEG			0.5f
/** Holgura al exigir que el exceso de radio no crezca a lo largo del segmento. */
#define PLAN_TOL_RADIO_MM			1.0e-3f
/** Si falta menos que esto, el tick llega al final (redondeo de la suma de avances). */
#define PLAN_TOL_FIN_MM				1.0e-4f
/** Biparticiones del avance al aplicar el limite de velocidad articular. */
#define PLAN_LIM_ITER				10
/** Ticks seguidos sin poder avanzar antes de abortar el movimiento. */
#define PLAN_TICKS_ATASCO			50

/* ------------------------------------------------------------------------- */
/* Encoders AS5600 (uno por bus I2C: hombro en I2C1, codo en I2C2)           */
/*   theta = ENC_SIGN * normalizar(lectura - ENC_ZERO), en (-180, 180]       */
/* ------------------------------------------------------------------------- */

/* [CONFIRMAR] se calibran a mano con el comando ANGLE (ver README, prueba de E4a):
 * ENCx_ZERO_DEG es lo que lee el encoder con el brazo recto apuntando a +X (theta = 0);
 * ENCx_SIGN es +1 si la lectura crece al girar la junta en sentido antihorario. */
#define ENC1_ZERO_DEG				0.0f	/**< [CONFIRMAR] lectura del AS5600 del hombro con theta1 = 0 */
#define ENC1_SIGN					1.0f	/**< [CONFIRMAR] */
#define ENC2_ZERO_DEG				0.0f	/**< [CONFIRMAR] lectura del AS5600 del codo con theta2 = 0 */
#define ENC2_SIGN					1.0f	/**< [CONFIRMAR] */

#define I2C_FRECUENCIA_HZ			400000U	/**< Modo rapido */
/** Tiempo maximo de espera de cada bandera del I2C. Una lectura completa del AS5600 tarda
 *  ~0,15 ms; si el sensor no contesta, la funcion vuelve con error en este tiempo. */
#define I2C_TIMEOUT_MS				2U
/** Vueltas de espera entre flancos al destrabar el bus a mano (unos pocos us a 96 MHz). */
#define I2C_RECUP_VUELTAS			200U

/* ------------------------------------------------------------------------- */
/* Consola (USART1)                                                          */
/* ------------------------------------------------------------------------- */

#define UART_BAUDIOS				115200U
/** Buffer de recepcion (potencia de 2). Sobra para pegar una linea de 100 caracteres. */
#define UART_RX_BUF_BYTES			128U
/** Buffer de transmision (potencia de 2). El eco de microrl manda ~20 bytes por tecla:
 *  pegar 100 caracteres genera ~2 KB de salida en unos pocos ms. */
#define UART_TX_BUF_BYTES			4096U
/** Prioridad NVIC de la USART1 (0 = maxima). El SysTick queda en la minima (15). */
#define UART_PRIORIDAD_IRQ			1U
/** Bytes de RX que la consola procesa por vuelta del loop, para no demorar el tick. */
#define CONSOLA_BYTES_POR_VUELTA	32U

/* ------------------------------------------------------------------------- */
/* Varios                                                                    */
/* ------------------------------------------------------------------------- */

#define LED_PERIODO_MS				1000	/**< LED de estado: 1 Hz en funcionamiento normal */
#define LED_PERIODO_FALLA_MS		200		/**< LED de estado: 5 Hz si el HSE no arranco */

#endif /* CONFIG_H */
