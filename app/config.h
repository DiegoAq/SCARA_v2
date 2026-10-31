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

#define SERVO_RANGE_DEG				180.0f	/**< [CONFIRMAR] recorrido nominal de un servo entre PULSE_MIN y PULSE_MAX */
#define PULSE_MIN_US				500		/**< Ningun pulso de ningun servo sale por debajo de esto */
#define PULSE_MAX_US				2500	/**< Ningun pulso de ningun servo sale por encima de esto */

/* Recorrido real de cada servo entre PULSE_MIN y PULSE_MAX: CALIB sugiere el valor. */
#define SHOULDER_RANGE_DEG			SERVO_RANGE_DEG	/**< [CONFIRMAR con CALIB] */
#define ELBOW_RANGE_DEG				SERVO_RANGE_DEG	/**< [CONFIRMAR con CALIB] */

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

#ifndef ELBOW_ZERO_SERVO_DEG
#define ELBOW_ZERO_SERVO_DEG		90.0f	/**< Angulo de servo con el brazo recto (theta2 = 0) */
#endif
#define ELBOW_SIGN					1.0f	/**< [CONFIRMAR] */
#define ELBOW_SERVO_MIN_DEG			0.0f	/**< [CONFIRMAR] tope mecanico, en angulo de servo */
#define ELBOW_SERVO_MAX_DEG			180.0f	/**< [CONFIRMAR] tope mecanico, en angulo de servo */

/* Postura de HOME y de arranque. Una junta SIN encoder habilitado no sabe donde esta: se supone
 * en HOME y el primer HOME habilita su PWM directamente en esta postura; si estaba en otro lado,
 * el servo va de golpe hasta aca. Una junta CON encoder arranca desde lo medido (ver README). */
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
 * geometrico es 282,8 mm. Se deja un margen. Las pautas piden 5 cm < R < 38 cm: el maximo ya
 * cumple, el minimo depende de remontar el codo (ELBOW_ZERO_SERVO_DEG) y de medir hasta donde
 * pliega sin chocar con la base. [CONFIRMAR] al definir el montaje final. */
#ifndef R_MIN_OP_MM
#define R_MIN_OP_MM					285.0f	/**< Radio minimo operativo */
#endif
#define R_MAX_OP_MM					380.0f	/**< Margen contra la singularidad de brazo estirado (pautas) */

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
/** Aceleracion maxima de cada junta, en todos los comandos (el brazo tiene masa: sin esto
 *  arrancaba y frenaba de golpe). [AJUSTAR con el brazo] 90 = llega a 30 grados/s en 1/3 s. */
#define JOINT_ACCEL_DPS2			90.0f
/** Velocidad articular que siempre se permite en un tick, aunque supere la rampa de
 *  aceleracion. Hace falta para poder salir de la postura de brazo estirado (ver planificador). */
#define JOINT_W_ARRANQUE_DPS		6.0f

#define PREFLIGHT_STEP_MM			2.0f	/**< Paso de muestreo del segmento en el pre-vuelo */
#define MOVE_DIST_MIN_MM			0.1f	/**< Por debajo de esto el MOVE responde OK sin moverse */

/** Maxima diferencia por junta entre IK(P0) y la postura actual para considerar que
 *  el brazo ya esta en esa rama de codo (solo se cumple en ambas cerca de theta2 = 0). */
#define PLAN_TOL_RAMA_DEG			0.5f
/** Holgura al exigir que el exceso de radio no crezca a lo largo del segmento. */
#define PLAN_TOL_RADIO_MM			1.0e-3f
/** Si falta menos que esto, el tick llega al final (redondeo de la suma de avances). */
#define PLAN_TOL_FIN_MM				1.0e-4f
/** Fraccion del recorrido por debajo de la cual un perfil articular da por terminado el tick final. */
#define PERFIL_TOL_FIN				1.0e-6f
/** Biparticiones del avance al aplicar el limite de velocidad articular. */
#define PLAN_LIM_ITER				10
/** Ticks seguidos sin poder avanzar antes de abortar el movimiento. */
#define PLAN_TICKS_ATASCO			50

/* ------------------------------------------------------------------------- */
/* Encoders AS5600 (uno por bus I2C: hombro en I2C1, codo en I2C2)           */
/*   theta = ENC_SIGN * normalizar(lectura - ENC_ZERO), en (-180, 180]       */
/* ------------------------------------------------------------------------- */

/* [CONFIRMAR] salen del comando CALIB y se afinan con el brazo alineado (ver README):
 * ENCx_ZERO_DEG es lo que lee el encoder con theta = 0;
 * ENCx_SIGN es +1 si la lectura crece al girar la junta en sentido antihorario. */
#define ENC1_ZERO_DEG				0.0f	/**< [CONFIRMAR] lectura del AS5600 del hombro con theta1 = 0 */
#define ENC1_SIGN					1.0f	/**< [CONFIRMAR] */
#define ENC2_ZERO_DEG				0.0f	/**< [CONFIRMAR] lectura del AS5600 del codo con theta2 = 0 */
#define ENC2_SIGN					1.0f	/**< [CONFIRMAR] */

/* Poner en 1 recien cuando el encoder ande y este calibrado: con un cero mal cargado el primer
 * pulso sale de una posicion falsa y el servo pega el tiron que esto quiere evitar. Se deciden
 * por separado porque hoy anda uno solo. ANGLE, RAW y CALIB leen el encoder igual. */
#ifndef ENC1_HABILITADO
#define ENC1_HABILITADO				0		/**< [CONFIRMAR] 1 = el hombro arranca de lo medido y se supervisa */
#endif
#ifndef ENC2_HABILITADO
#define ENC2_HABILITADO				0		/**< [CONFIRMAR] 1 = el codo arranca de lo medido y se supervisa */
#endif

#define ENC_RESOLUCION_DEG			(360.0f / 4096.0f)	/**< AS5600: 12 bits por vuelta */
#define ERR_FAULT_DEG				10.0f	/**< Error de junta que, sostenido, pasa a FAULT */
#define ERR_FAULT_MS				200U	/**< Tiempo sostenido de error o de falta de lectura antes de FAULT */
#define ENC_IMAN_PERIODO_MS			200U	/**< Cada cuanto se verifica el iman (lectura extra por I2C) */
#define SETTLE_MS					300U	/**< Espera tras llegar antes de medir el error final */

/* Correccion final (SPEC 8.5). Apagada hasta medir con REPEAT si hace falta. */
#ifndef TRIM_ENABLE
#define TRIM_ENABLE					0		/**< 1 = corrige el error final con los encoders */
#endif
#define ERR_TRIM_DEG				0.5f	/**< Por debajo de este error no se corrige */
#define K_TRIM						0.6f	/**< Fraccion del error corregida en cada iteracion */
#define TRIM_MAX_ITER				3		/**< Iteraciones de correccion por movimiento */
#define TRIM_MAX_DEG				3.0f	/**< Correccion total maxima por junta */

/* CALIB: la junta recorre estos desvios respecto de HOME, de a uno. La SPEC pedia +-60; se deja
 * en +-30 hasta medir los topes mecanicos reales. [AJUSTAR] ampliar para un ajuste mejor. */
#define CALIB_PASOS_DEG				{ -30.0f, -15.0f, 0.0f, 15.0f, 30.0f }
#define CALIB_CANT_PASOS			5
#define CALIB_SETTLE_MS				500U	/**< Espera en cada punto antes de medir */
#define CALIB_MUESTRAS				8		/**< Lecturas promediadas en cada punto */

#define I2C_FRECUENCIA_HZ			400000U	/**< Modo rapido */
/** Tiempo maximo de espera de cada bandera del I2C. Una lectura completa del AS5600 tarda
 *  ~0,15 ms; si el sensor no contesta, la funcion vuelve con error en este tiempo. */
#define I2C_TIMEOUT_MS				2U
/** Vueltas de espera entre flancos al destrabar el bus a mano (unos pocos us a 96 MHz). */
#define I2C_RECUP_VUELTAS			200U

/* ------------------------------------------------------------------------- */
/* Eje Z y orientacion: dos motores DC diferenciales (SPEC 9)                */
/*   a = Z * PULSOS_POR_MM + ROT_SIGN * rot * PULSOS_POR_GRADO               */
/*   b = Z * PULSOS_POR_MM - ROT_SIGN * rot * PULSOS_POR_GRADO               */
/* ------------------------------------------------------------------------- */

/* Mecanica todavia sin montar: TODO este bloque es un valor de partida. Se ajusta con ZTEST. */
#define Z_MIN_MM					(-100.0f)	/**< [CONFIRMAR] pautas: recorrido +-10 cm */
#define Z_MAX_MM					100.0f		/**< [CONFIRMAR] +Z = hacia arriba */
#define ROT_MIN_DEG					(-180.0f)	/**< [CONFIRMAR] limite de orientacion de la garra */
#define ROT_MAX_DEG					180.0f		/**< [CONFIRMAR] */

#define PULSOS_POR_MM				10.0f	/**< [CONFIRMAR] pulsos de un motor por mm de Z */
#define PULSOS_POR_GRADO			1.0f	/**< [CONFIRMAR] pulsos de un motor por grado de rotacion */
#define MOTOR_A_SIGN				1.0f	/**< [CONFIRMAR] +1 si duty positivo en A sube el eje; si no, -1 */
#define MOTOR_B_SIGN				1.0f	/**< [CONFIRMAR] idem para B */
#define ROT_SIGN					1.0f	/**< [CONFIRMAR] +1 si rot positivo gira en sentido antihorario */

#define Z_VEL_MM_S					10.0f	/**< [AJUSTAR] velocidad del eje Z */
#define Z_ACEL_MM_S2				20.0f	/**< [AJUSTAR] aceleracion del eje Z */
#define ROT_VEL_DPS					30.0f	/**< [AJUSTAR] velocidad de orientacion */
#define ROT_ACEL_DPS2				60.0f	/**< [AJUSTAR] aceleracion de orientacion */

#define MOTOR_PWM_HZ				10000U	/**< PWM de los enables del L298N */
#define MOTOR_PWM_MIN				0.35f	/**< [AJUSTAR] duty (0 a 1) por debajo del cual el motor no vence el rozamiento */
#define MOTOR_PWM_MAX				0.80f	/**< [AJUSTAR] duty maximo en control */
#define MOTOR_KP					0.02f	/**< [AJUSTAR] duty por pulso de error */
#define MOTOR_RAMPA_POR_TICK		0.05f	/**< Mayor suba de duty por tick */
#define MOTOR_TOL_PULSOS			1.0f	/**< Error aceptado al llegar */
#define MOTOR_TOL_ATASCO_PULSOS		5.0f	/**< Por debajo de este error, no moverse no cuenta como atasco */
#define MOTOR_STALL_MS				500U	/**< Tiempo empujando sin pulsos antes de cortar por atasco */
#define MOTOR_ASENTAR_MS			500U	/**< Tiempo maximo para entrar en tolerancia al final de la trayectoria */
#define MOTOR_PAUSA_INVERSION_MS	60		/**< Frenado antes de invertir: sin esto los pulsos de la inercia cuentan al reves */

#define ENCODER_FILTRO_US			100U	/**< Dos flancos mas juntos que esto se toman como rebote */
#define ENCODER_PRIORIDAD_IRQ		2U		/**< Prioridad NVIC de los encoders opticos */

#define ZTEST_PWM_MAX_PCT			80.0f	/**< Mayor duty aceptado por ZTEST (%) */
#define ZTEST_MS_MAX				3000	/**< Mayor duracion aceptada por ZTEST */

/* ------------------------------------------------------------------------- */
/* Gripper (MG90S en PB6)                                                    */
/* ------------------------------------------------------------------------- */

#define GRIPPER_MIN_MM				15.0f	/**< Apertura minima (pautas) */
#define GRIPPER_MAX_MM				60.0f	/**< Apertura maxima (pautas) */
#define GRIPPER_PULSO_MIN_MM_US		1000	/**< [CONFIRMAR con PULSE 3] pulso con la garra en GRIPPER_MIN_MM */
#define GRIPPER_PULSO_MAX_MM_US		2000	/**< [CONFIRMAR con PULSE 3] pulso con la garra en GRIPPER_MAX_MM */
#define GRIPPER_INICIO_MM			GRIPPER_MAX_MM	/**< Apertura supuesta al encender: sin encoder, el primer pulso es un salto */
#define GRIPPER_APRIETE_MM			3.0f	/**< [AJUSTAR] cuanto menos que el ancho de la caja se comanda al sujetar */
#define GRIPPER_TOMA_MM				(CAJA_ANCHO_MM - GRIPPER_APRIETE_MM)	/**< Apertura al sujetar si PICK no la indica */
#define GRIPPER_DPS					30.0f	/**< Velocidad del servo. Debe ser <= JOINT_VMAX_DPS */
#define GRIPPER_ACCEL_DPS2			90.0f	/**< Aceleracion del servo */

/* ------------------------------------------------------------------------- */
/* Sensores de distancia VL53L0X (I2C1, junto con el AS5600 del hombro)      */
/* ------------------------------------------------------------------------- */

#define VL53_RESET_MS				2U		/**< Tiempo de XSHUT en bajo y de arranque del sensor */
#define VL53_TIMEOUT_MS				100U	/**< Espera maxima de cada etapa o medicion */
#define DIST_MUESTRAS				3		/**< Mediciones de cada sensor promediadas antes de tomar un objeto */
#define DIST_OBJETO_MIN_MM			20.0f	/**< [AJUSTAR] mas cerca que esto no es un objeto valido */
#define DIST_OBJETO_MAX_MM			300.0f	/**< [AJUSTAR] mas lejos que esto se considera que no hay objeto */

/* Montaje: los dos miran hacia abajo, fijos al cuerpo de la garra y en lados opuestos. La linea
 * que los une es perpendicular al cierre de los dedos: con la garra bien puesta queda paralela al
 * lado largo de la caja, y los dos ven la tapa a la vez. El A es el del lado positivo de esa
 * linea, y su direccion en la mesa es theta1 + theta2 + rot + GARRA_YAW_CERO_DEG. */
#define SENSOR_SEPARACION_MM		70.0f	/**< [CONFIRMAR] distancia entre los dos haces. Tiene que ser menor que CAJA_LARGO_MM */
#define SENSOR_DESVIO_MM			0.0f	/**< [CONFIRMAR] cuanto se corren los dos respecto del centro de los dedos, a 90 grados antihorario de esa linea */
#define SENSOR_A_OFFSET_MM			0.0f	/**< [CONFIRMAR] lo que mide de mas el sensor A: se le resta a cada lectura */
#define SENSOR_B_OFFSET_MM			0.0f	/**< [CONFIRMAR] idem para el B */
#define GARRA_YAW_CERO_DEG			0.0f	/**< [CONFIRMAR] angulo de la linea de sensores (de B a A) respecto del antebrazo, con rot = 0 */

/* ------------------------------------------------------------------------- */
/* Cajas y busqueda (SCAN)                                                   */
/* ------------------------------------------------------------------------- */

/* [CONFIRMAR] Medidas que el equipo paso de memoria el 2026-10-30, a confirmar. */
#define CAJA_LARGO_MM				100.0f	/**< [CONFIRMAR] */
#define CAJA_ANCHO_MM				30.0f	/**< [CONFIRMAR] */
#define CAJA_ALTOS_MM				{ 60.0f, 80.0f, 100.0f }	/**< [CONFIRMAR] altura de cada tipo de caja */
#define CAJA_CANT_TIPOS				3
#define CAJA_TOL_ALTO_MM			9.0f	/**< [AJUSTAR] mas lejos que esto de todas las alturas, el tipo queda sin definir */
#define CAJA_TOL_MEDIDA_MM			30.0f	/**< [AJUSTAR] largo o ancho medido mas lejos que esto del esperado: no es la caja */
#define CAJA_TOL_PAREJA_MM			15.0f	/**< [AJUSTAR] diferencia maxima entre los dos sensores sobre la tapa */
/** Lo que una lectura tiene que subir respecto de la mesa para contar como caja. Bastante menos que
 *  la caja mas baja: con el cono del sensor apoyado en parte sobre la mesa, desde Z_BARRIDO_MM esa
 *  caja se ve solo unos 30 mm mas alta que la mesa. */
#define CAJA_UMBRAL_MM				20.0f
#define CAJA_MANCHA_MIN_MM2			1500.0f	/**< [AJUSTAR] manchas mas chicas que esto son ruido: es la mitad de la tapa */

/* Con la pinza en Z_BARRIDO_MM, la punta de los dedos tiene que pasar por arriba de la caja mas
 * alta. Cuanto mas abajo mejor: el cono del sensor abre unos 25 grados, asi que ve una mancha de
 * 22 mm a 50 mm de altura y de 44 mm a 100 mm, y la caja tiene 30 mm de ancho. */
#define Z_BARRIDO_MM				Z_SEGURA_MM	/**< [CONFIRMAR] altura de la pinza durante el barrido */
#define BARRIDO_PASO_MM				15.0f	/**< [AJUSTAR] separacion entre pasadas: con 15, al menos dos pasadas cruzan los 30 mm de la caja */
#define BARRIDO_VEL_MM_S			30.0f	/**< [AJUSTAR] velocidad de las pasadas. Tiene que estar en [SPEED_MIN_MM_S, SPEED_MAX_MM_S] */
#define BARRIDO_DIST_MIN_MM			40.0f	/**< [CONFIRMAR] un sensor mas cerca que esto corta el barrido: los dedos estan por chocar */

#define MAPA_CELDA_MM				BARRIDO_PASO_MM	/**< Lado de cada celda: una fila por pasada, para que no queden filas vacias */
#define MAPA_CELDAS_MAX				1600	/**< Cubre 760 x 380 mm: todo el rectangulo del espacio de trabajo */
#define MAPA_COLUMNAS_MAX			64		/**< Cada fila del comando MAP entra en una linea */
#define MAPA_PERCENTIL_MESA			80U		/**< Percentil de las distancias que se toma como la mesa */
#define MAPA_HIST_MM				4U		/**< Ancho de cada barra del histograma de distancias */
#define MAPA_HIST_BINS				256U	/**< Barras del histograma: llega a un metro */

/* Pasadas lentas sobre la caja, una a lo ancho y otra a lo largo, cada una de ida y de vuelta.
 * El margen tiene que cubrir el error de la ubicacion aproximada mas el radio de la mancha del
 * sensor: a cada lado de la caja, el sensor debe llegar a ver solamente la mesa. */
#define CENTRADO_VEL_MM_S			15.0f	/**< [AJUSTAR] velocidad de las pasadas. Tiene que estar en [SPEED_MIN_MM_S, SPEED_MAX_MM_S] */
#define CENTRADO_MARGEN_MM			40.0f	/**< [AJUSTAR] cuanto se pasa de los bordes de la caja */
#define CENTRADO_MUESTRAS_MAX		256U	/**< Lecturas por sensor en cada pasada: sobra para 110 mm a 15 mm/s */
#define CENTRADO_YAW_MAX_DEG		30.0f	/**< Una correccion de orientacion mayor que esto se considera un error de medicion */

/* ------------------------------------------------------------------------- */
/* Secuencias PICK, PLACE, PICKPLACE, PEN y DRAW                             */
/* ------------------------------------------------------------------------- */

/** 1 = PICK exige un sensor, verifica el objeto y calcula cuanto bajar; 0 = baja a Z_TOMA_MM. */
#define PICK_USA_SENSOR				1
#define PICK_DIST_TOMA_MM			30.0f	/**< [CONFIRMAR] distancia sensor-objeto con el objeto dentro de la garra */
/** PICK y SCAN se acercan hasta esta distancia y recien ahi confian en la lectura: la mancha del
 *  sensor tiene 22 mm y entra en la tapa. Desde mas arriba toma parte de la mesa y la caja parece
 *  mas lejos de lo que esta. */
#define PICK_DIST_APROX_MM			50.0f
/** Que parte de lo que falta recorre el primer acercamiento. Como esa lectura puede dar de mas, ir
 *  directo a PICK_DIST_APROX_MM dejaria la caja mas cerca de lo pensado y hasta contra la garra. */
#define PICK_APROX_FRACCION			0.5f
#define PLACE_HOLGURA_MM			3.0f	/**< [AJUSTAR] al soltar lo tomado por PICK, cuanto mas arriba de la altura de toma queda */
#define Z_SEGURA_MM					50.0f	/**< [CONFIRMAR] altura para trasladarse en XY */
#define Z_TOMA_MM					(-50.0f)	/**< [CONFIRMAR] altura de toma sin sensor */
#define Z_DEJAR_MM					(-50.0f)	/**< [CONFIRMAR] altura para soltar sin una toma previa */
#define Z_LAPIZ_ARRIBA_MM			0.0f	/**< [CONFIRMAR] marcador levantado */
#define Z_LAPIZ_ABAJO_MM			(-50.0f)	/**< [CONFIRMAR] marcador apoyado; la fuerza la fija el resorte del soporte */
#define SEQ_ESPERA_GRIPPER_MS		300U	/**< Tras mover la garra, antes de mover el eje Z */
#define DRAW_CUERDA_MM				5.0f	/**< Largo de cada tramo recto de un circulo */
#define DRAW_CUERDAS_MAX			180		/**< Mas tramos que esto se rechaza */
#define REPEAT_MAX_CICLOS			100		/**< Mayor cantidad de ciclos de REPEAT */

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
