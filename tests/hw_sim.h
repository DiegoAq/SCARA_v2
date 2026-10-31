/**
 * @file hw_sim.h
 * @brief Hardware simulado para probar en la PC la consola, el movimiento y las rutinas.
 *
 * Reemplaza uart.c, systick.c, pwm.c, as5600.c, i2c.c, motor.c, encoder.c y vl53l0x.c por
 * versiones en memoria: lo que se "tipea" entra por UART_LeerByte, lo que la consola responde se
 * acumula en un buffer, y los pulsos que irian a los servos quedan registrados para verificar
 * que ningun tick supere el limite de velocidad.
 */
#ifndef HW_SIM_H
#define HW_SIM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "as5600.h"
#include "config.h"
#include "encoder.h"
#include "motor.h"
#include "pwm.h"
#include "vl53l0x.h"

/** Mayor cambio de pulso admitido por tick: JOINT_VMAX_DPS, mas 1 us de redondeo. */
#define DELTA_US_MAX	((int)(JOINT_VMAX_DPS * (float)TICK_MS / 1000.0f \
						 * (float)(PULSE_MAX_US - PULSE_MIN_US) / SERVO_RANGE_DEG) + 1)

/** Mayor cambio de VELOCIDAD admitido entre ticks en JOINT/HOME/PULSE: JOINT_ACCEL_DPS2,
 *  mas 2 us porque cada pulso se redondea a 1 us. */
#define ACEL_US_MAX		((int)(JOINT_ACCEL_DPS2 * (float)TICK_MS / 1000.0f * (float)TICK_MS / 1000.0f \
						 * (float)(PULSE_MAX_US - PULSE_MIN_US) / SERVO_RANGE_DEG) + 2)

/* Consola */
extern char salida[];
extern size_t salida_len;
extern uint32_t ms_simulado;

/* Servos */
extern uint32_t pwm_us[PWM_CANT];
extern bool pwm_activo[PWM_CANT];
extern int delta_us_visto;
extern int acel_us_vista;

/* AS5600 */
extern bool enc_presente[I2C_CANT_BUSES];
extern bool bus_trabado[I2C_CANT_BUSES];
extern uint16_t enc_cuentas[I2C_CANT_BUSES];
extern as5600_iman_t enc_iman[I2C_CANT_BUSES];
/** true: en cada tick el encoder lee la junta real, derivada del pulso que le llega al servo. */
extern bool enc_sigue_servo;
/** Signo y cero con que el encoder ve la junta real (lo que CALIB tiene que encontrar). */
extern float enc_signo_real[I2C_CANT_BUSES];
extern float enc_cero_real[I2C_CANT_BUSES];
/** Grados reales de junta por grado comandado (ganancia del servo). */
extern float enc_ganancia_real[I2C_CANT_BUSES];
/** Desvio agregado a la junta real, para simular un servo que no llega (grados). */
extern float enc_desvio_deg[I2C_CANT_BUSES];

/* Motores del eje Z */
extern float motor_pos[MOTOR_CANT];			/**< Posicion real, en pulsos */
extern bool motor_trabado[MOTOR_CANT];		/**< No gira ni da pulsos */
extern bool motor_frenado[MOTOR_CANT];		/**< Esta frenado */
extern float motor_duty[MOTOR_CANT];		/**< Ultimo duty, ya con el signo de config.h */
extern int32_t encoder_pulsos[ENCODER_CANT];
extern int motor_inversiones_sin_freno;		/**< Veces que un sentido se invirtio sin frenar antes */

/* VL53L0X. Sin mundo, los dos ven lo mismo: vl53_mm con el eje Z en vl53_z_ref, y la lectura sigue
 * a la altura del eje, como si el objeto estuviera quieto. */
extern bool vl53_presente[VL53_CANT];
extern bool vl53_mudo[VL53_CANT];		/**< Paso el arranque, pero dejo de contestar por I2C */
extern bool vl53_ve_objeto;
extern uint16_t vl53_mm;
extern float vl53_z_ref;
extern int vl53_mediciones;

/* Mundo: una mesa con cajas. Los sensores miden desde la pose REAL que sale de los pulsos y de los
 * motores, con su separacion y su yaw reales, que pueden no ser los de config.h. */
#define MUNDO_CAJAS_MAX	4

/** Una caja sobre la mesa. */
typedef struct {
	float x, y;			/**< Centro (mm) */
	float yaw;			/**< Direccion del lado largo (grados) */
	float largo, ancho, alto;
} mundo_caja_t;

extern bool mundo_activo;
extern float mundo_z_mesa;					/**< Altura del eje Z a la que un sensor tocaria la mesa */
extern mundo_caja_t mundo_caja[MUNDO_CAJAS_MAX];
extern int mundo_cajas;
extern float mundo_separacion;				/**< Separacion real entre los haces (mm) */
extern float mundo_yaw_cero;				/**< GARRA_YAW_CERO_DEG real (grados) */
extern float mundo_desvio_a;				/**< Cuanto mide de mas el sensor A (mm) */
extern float mundo_ruido_mm;				/**< Ruido uniforme de cada lectura: +-esto (mm) */
extern int mundo_retardo_ticks;				/**< Lo que el brazo real tarda en seguir a los pulsos */

/** Reinicia todo el hardware simulado y los modulos de la aplicacion. */
void hw_sim_reiniciar(void);

/**
 * Hace correr el loop de control hasta que todo queda quieto o pasan @p max ticks.
 * En CADA tick verifica el limite de velocidad sobre los pulsos que salen a los servos.
 * \return ticks ejecutados. La salida de la consola se acumula (no se borra).
 */
int correr(int max);

/** "Tipea" un texto, procesa todo lo recibido y devuelve lo que respondio la consola. */
const char *tipear(const char *texto);

/** Cuenta las apariciones de @p buscado en @p texto. */
int contar(const char *texto, const char *buscado);

/** Borra lo que la consola respondio hasta ahora. */
void borrar_salida(void);

#endif /* HW_SIM_H */
