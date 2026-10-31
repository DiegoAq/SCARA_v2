/**
 * @file hw_sim.c
 * @brief Hardware simulado para probar en la PC la consola, el movimiento y las rutinas.
 */
#include "hw_sim.h"

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "cinematica.h"
#include "console.h"
#include "eje_z.h"
#include "gripper.h"
#include "i2c.h"
#include "motion.h"
#include "secuencia.h"
#include "servo.h"
#include "systick.h"
#include "uart.h"
#include "vl53l0x.h"

/* ---- UART y SysTick ------------------------------------------------------- */

static char entrada[1024];
static size_t entrada_pos;
static size_t entrada_len;
char salida[65536];
size_t salida_len;
uint32_t ms_simulado;

void UART_Init(void) {}

bool UART_LeerByte(uint8_t *byte)
{
	if (entrada_pos >= entrada_len) {
		return false;
	}
	*byte = (uint8_t)entrada[entrada_pos++];
	return true;
}

void UART_EnviarDatos(const uint8_t *datos, size_t longitud)
{
	assert(salida_len + longitud < sizeof(salida));
	memcpy(salida + salida_len, datos, longitud);
	salida_len += longitud;
	salida[salida_len] = '\0';
}

void UART_EnviarCadena(const char *texto)
{
	UART_EnviarDatos((const uint8_t *)texto, strlen(texto));
}

uint32_t UART_BytesPerdidosRx(void) { return 0U; }
uint32_t UART_BytesPerdidosTx(void) { return 0U; }
bool SysTick_Init(void) { return true; }
uint32_t Millis(void) { return ms_simulado; }

/* ---- AS5600 e I2C ----------------------------------------------------------- */

bool enc_presente[I2C_CANT_BUSES] = { true, true };
bool bus_trabado[I2C_CANT_BUSES];
uint16_t enc_cuentas[I2C_CANT_BUSES];
as5600_iman_t enc_iman[I2C_CANT_BUSES];
bool enc_sigue_servo;
float enc_signo_real[I2C_CANT_BUSES];
float enc_cero_real[I2C_CANT_BUSES];
float enc_ganancia_real[I2C_CANT_BUSES];
float enc_desvio_deg[I2C_CANT_BUSES];

i2c_res_t I2C_Init(i2c_bus_t bus) { (void)bus; return I2C_OK; }

i2c_res_t I2C_Sondear(i2c_bus_t bus, uint8_t dir7)
{
	if (bus_trabado[bus]) {
		return I2C_ERR_TIMEOUT;
	}
	return (enc_presente[bus] && dir7 == AS5600_DIRECCION) ? I2C_OK : I2C_ERR_NACK;
}

bool AS5600_Init(i2c_bus_t bus) { return enc_presente[bus]; }

bool AS5600_LeerCuentas(i2c_bus_t bus, uint16_t *cuentas)
{
	if (!enc_presente[bus]) {
		return false;
	}
	*cuentas = enc_cuentas[bus];
	return true;
}

bool AS5600_LeerAngulo(i2c_bus_t bus, float *grados)
{
	uint16_t c;

	if (!AS5600_LeerCuentas(bus, &c)) {
		return false;
	}
	*grados = (float)c * (360.0f / (float)AS5600_CUENTAS_VUELTA);
	return true;
}

as5600_iman_t AS5600_EstadoIman(i2c_bus_t bus)
{
	return enc_presente[bus] ? enc_iman[bus] : AS5600_SIN_RESPUESTA;
}

bool AS5600_ImanOK(i2c_bus_t bus) { return AS5600_EstadoIman(bus) == AS5600_IMAN_OK; }

/** Pone el encoder en la lectura que corresponde a una junta real. */
static void encoder_en_junta(int bus, float th_real)
{
	float enc = enc_cero_real[bus] + enc_signo_real[bus] * th_real;

	while (enc < 0.0f) {
		enc += 360.0f;
	}
	while (enc >= 360.0f) {
		enc -= 360.0f;
	}
	enc_cuentas[bus] = (uint16_t)(enc * (float)AS5600_CUENTAS_VUELTA / 360.0f + 0.5f)
					 % AS5600_CUENTAS_VUELTA;
}

/* ---- PWM ---------------------------------------------------------------------- */

uint32_t pwm_us[PWM_CANT];
bool pwm_activo[PWM_CANT];
int delta_us_visto;
int acel_us_vista;
static int delta_previo[PWM_CANT];

void PWM_Init(void)
{
	for (int i = 0; i < PWM_CANT; i++) {
		pwm_us[i] = 0U;
		pwm_activo[i] = false;
		delta_previo[i] = 0;
	}
}

void PWM_ConfigurarPulsoUs(pwm_id_t id, uint32_t us)
{
	/* Quien llama ya tiene que haber saturado: aca seria un pulso fuera de rango al servo. */
	assert(us >= PULSE_MIN_US && us <= PULSE_MAX_US);
	pwm_us[id] = us;
}

void PWM_Habilitar(pwm_id_t id, bool habilitar)
{
	pwm_activo[id] = habilitar;
}

/* ---- Motores y encoders opticos -------------------------------------------------- */

/* Una planta sencilla: velocidad de primer orden hacia un valor proporcional al duty. La inercia
 * hace que los pulsos sigan llegando despues de frenar, como en el eje real. */
#define PULSOS_POR_TICK_DUTY_1	20.0f	/* a duty 1, a regimen */
#define DUTY_ARRANQUE			0.2f	/* por debajo no vence el rozamiento */
#define ALFA_ARRASTRE			0.3f
#define ALFA_FRENO				0.7f

float motor_pos[MOTOR_CANT];
bool motor_trabado[MOTOR_CANT];
bool motor_frenado[MOTOR_CANT];
float motor_duty[MOTOR_CANT];
int32_t encoder_pulsos[ENCODER_CANT];
int motor_inversiones_sin_freno;
static float motor_vel[MOTOR_CANT];
static int32_t encoder_sentido[ENCODER_CANT];

void Motor_Init(void)
{
	for (int m = 0; m < MOTOR_CANT; m++) {
		motor_pos[m] = 0.0f;
		motor_vel[m] = 0.0f;
		motor_duty[m] = 0.0f;
		motor_frenado[m] = true;
		motor_trabado[m] = false;
	}
	motor_inversiones_sin_freno = 0;
}

void Motor_ConfigurarDuty(motor_id_t id, float duty)
{
	assert(id < MOTOR_CANT);
	assert(fabsf(duty) <= 1.0f);
	if (!motor_frenado[id] && motor_duty[id] * duty < 0.0f) {
		motor_inversiones_sin_freno++;
	}
	motor_duty[id] = duty;
	motor_frenado[id] = false;
}

void Motor_Frenar(motor_id_t id)
{
	assert(id < MOTOR_CANT);
	motor_duty[id] = 0.0f;
	motor_frenado[id] = true;
}

void Encoder_Init(void)
{
	for (int i = 0; i < ENCODER_CANT; i++) {
		encoder_pulsos[i] = 0;
		encoder_sentido[i] = 1;
	}
}

int32_t Encoder_Pulsos(encoder_id_t id) { return encoder_pulsos[id]; }
void Encoder_FijarSentido(encoder_id_t id, int32_t sentido) { encoder_sentido[id] = (sentido >= 0) ? 1 : -1; }
void Encoder_Reiniciar(encoder_id_t id) { encoder_pulsos[id] = 0; }

/** Un tick de la planta. Con un solo canal, cada ranura cuenta segun el sentido cargado. */
static void avanzar_motores(void)
{
	static const float signo[MOTOR_CANT] = { MOTOR_A_SIGN, MOTOR_B_SIGN };

	for (int m = 0; m < MOTOR_CANT; m++) {
		float logico = motor_duty[m] * signo[m];
		float objetivo = (motor_trabado[m] || fabsf(logico) < DUTY_ARRANQUE)
					   ? 0.0f : logico * PULSOS_POR_TICK_DUTY_1;
		float alfa = motor_frenado[m] ? ALFA_FRENO : ALFA_ARRASTRE;

		motor_vel[m] += (objetivo - motor_vel[m]) * alfa;
		if (motor_trabado[m]) {
			motor_vel[m] = 0.0f;
		}

		float antes = motor_pos[m];

		motor_pos[m] += motor_vel[m];
		encoder_pulsos[m] += encoder_sentido[m] * abs((int)floorf(motor_pos[m]) - (int)floorf(antes));
	}
}

/* ---- VL53L0X ----------------------------------------------------------------- */

#define TICKS_MEDICION	2
#define RETARDO_MAX		32
#define CONO_TAN		0.2217f		/* tan(12,5 grados): el cono del VL53L0X abre unos 25 grados */
#define ANILLOS_CONO	5			/* anillos de rayos dentro del cono */
#define PASOS_COSTADO	8			/* puntos de cada rayo entre la tapa y la mesa */

bool vl53_presente[VL53_CANT];
bool vl53_mudo[VL53_CANT];
bool vl53_ve_objeto;
uint16_t vl53_mm;
float vl53_z_ref;
int vl53_mediciones;
static bool vl53_midiendo[VL53_CANT];
static int vl53_ticks[VL53_CANT];
static float vl53_lectura[VL53_CANT];

bool mundo_activo;
float mundo_z_mesa;
mundo_caja_t mundo_caja[MUNDO_CAJAS_MAX];
int mundo_cajas;
float mundo_separacion;
float mundo_yaw_cero;
float mundo_desvio_a;
float mundo_ruido_mm;
int mundo_retardo_ticks;
static uint32_t pulsos_previos[RETARDO_MAX][JUNTA_CANT];
static int pulsos_pos;

int VL53L0X_Init(void)
{
	return (vl53_presente[VL53_A] ? 1 : 0) + (vl53_presente[VL53_B] ? 1 : 0);
}

bool VL53L0X_Listo(vl53_id_t id) { return vl53_presente[id]; }

/** Guarda los pulsos de este tick, para el brazo real que los sigue con retardo. */
static void guardar_pulsos(void)
{
	pulsos_pos = (pulsos_pos + 1) % RETARDO_MAX;
	for (int j = 0; j < JUNTA_CANT; j++) {
		pulsos_previos[pulsos_pos][j] = pwm_us[j];
	}
}

/** Junta real: la del pulso que llego hace mundo_retardo_ticks. */
static float junta_real(int j)
{
	uint32_t us = pwm_us[j];

	if (mundo_retardo_ticks > 0) {
		us = pulsos_previos[(pulsos_pos + RETARDO_MAX - mundo_retardo_ticks) % RETARDO_MAX][j];
	}
	return Servo_JuntaDePulso((servo_id_t)j, us);
}

static bool dentro_de_caja(const mundo_caja_t *c, float x, float y)
{
	float ux, uy;

	Cinematica_Direccion(c->yaw, &ux, &uy);

	float dx = x - c->x;
	float dy = y - c->y;

	return fabsf(dx * ux + dy * uy) <= 0.5f * c->largo && fabsf(-dx * uy + dy * ux) <= 0.5f * c->ancho;
}

/**
 * Altura de lo primero que toca un rayo. Baja por pasos desde la tapa mas alta: un rayo inclinado
 * puede pasar por al lado de la tapa y dar contra el costado.
 * @param h Altura del sensor sobre la mesa.
 * @param dx,dy Cuanto se corre el rayo en el plano por cada mm que baja.
 */
static float altura_del_rayo(float x, float y, float h, float dx, float dy)
{
	float techo = 0.0f;

	for (int i = 0; i < mundo_cajas; i++) {
		const mundo_caja_t *c = &mundo_caja[i];

		for (int p = 0; p <= PASOS_COSTADO; p++) {
			float alt = c->alto * (1.0f - (float)p / (float)PASOS_COSTADO);
			float bajada = h - alt;

			if (bajada >= 0.0f && dentro_de_caja(c, x + dx * bajada, y + dy * bajada)) {
				techo = (alt > techo) ? alt : techo;
				break;
			}
		}
	}
	return techo;
}

/** Lo que mide un sensor: el promedio de lo que tocan los rayos de su cono. */
static float distancia_en_el_mundo(vl53_id_t id)
{
	float th1 = junta_real(JUNTA_HOMBRO);
	float th2 = junta_real(JUNTA_CODO);
	float a = motor_pos[MOTOR_A];
	float b = motor_pos[MOTOR_B];
	float z = 0.5f * (a + b) / PULSOS_POR_MM;
	float rot = 0.5f * (a - b) / (ROT_SIGN * PULSOS_POR_GRADO);
	float x, y, ux, uy;

	(void)Cinematica_Directa(th1, th2, &x, &y);
	Cinematica_Direccion(th1 + th2 + rot + mundo_yaw_cero, &ux, &uy);

	float lado = (id == VL53_A) ? 0.5f : -0.5f;
	float sx = x + lado * mundo_separacion * ux - SENSOR_DESVIO_MM * uy;
	float sy = y + lado * mundo_separacion * uy + SENSOR_DESVIO_MM * ux;
	float h = z - mundo_z_mesa;
	float suma = 0.0f;
	int rayos = 0;

	/* El centro y anillos de 6, 12, 18... rayos: quedan repartidos parejo sobre la mancha. Con pocos,
	 * la lectura sale a escalones y un borde parece correrse varios mm. */
	suma += h - altura_del_rayo(sx, sy, h, 0.0f, 0.0f);
	rayos++;
	for (int anillo = 1; anillo <= ANILLOS_CONO; anillo++) {
		int n = 6 * anillo;

		for (int r = 0; r < n; r++) {
			float cx, cy;

			Cinematica_Direccion(360.0f * (float)r / (float)n, &cx, &cy);

			float t = CONO_TAN * (float)anillo / (float)ANILLOS_CONO;

			suma += h - altura_del_rayo(sx, sy, h, t * cx, t * cy);
			rayos++;
		}
	}

	float d = suma / (float)rayos;

	if (mundo_ruido_mm > 0.0f) {
		d += mundo_ruido_mm * (2.0f * (float)rand() / (float)RAND_MAX - 1.0f);
	}
	return (id == VL53_A) ? d + mundo_desvio_a : d;
}

static float medir_ahora(vl53_id_t id)
{
	return mundo_activo ? distancia_en_el_mundo(id) : (float)vl53_mm + EjeZ_Altura() - vl53_z_ref;
}

bool VL53L0X_IniciarMedicion(vl53_id_t id)
{
	if (!vl53_presente[id] || vl53_mudo[id]) {
		return false;
	}
	vl53_midiendo[id] = true;
	vl53_ticks[id] = TICKS_MEDICION;
	vl53_mediciones++;
	return true;
}

vl53_res_t VL53L0X_Consultar(vl53_id_t id, uint16_t *mm)
{
	if (!vl53_midiendo[id] || vl53_mudo[id]) {
		vl53_midiendo[id] = false;
		return VL53_ERROR;
	}
	if (vl53_ticks[id] > 0) {
		vl53_ticks[id]--;
		/* La lectura sale de lo que pasaba durante la medicion, no de cuando se consulta. */
		if (vl53_ticks[id] == TICKS_MEDICION / 2) {
			vl53_lectura[id] = medir_ahora(id);
		}
		return VL53_EN_CURSO;
	}
	vl53_midiendo[id] = false;
	if (!vl53_ve_objeto) {
		return VL53_SIN_OBJETO;
	}
	*mm = (uint16_t)((vl53_lectura[id] > 0.0f) ? vl53_lectura[id] + 0.5f : 0.0f);
	return VL53_LISTO;
}

/* ---- Ayudas -------------------------------------------------------------------- */

void hw_sim_reiniciar(void)
{
	salida_len = 0U;
	salida[0] = '\0';
	delta_us_visto = 0;
	acel_us_vista = 0;
	enc_sigue_servo = false;
	vl53_ve_objeto = true;
	vl53_mm = 100U;
	vl53_z_ref = Z_SEGURA_MM;
	vl53_mediciones = 0;
	for (int i = 0; i < VL53_CANT; i++) {
		vl53_presente[i] = true;
		vl53_mudo[i] = false;
		vl53_midiendo[i] = false;
	}
	mundo_activo = false;
	mundo_z_mesa = -100.0f;
	mundo_cajas = 0;
	mundo_separacion = SENSOR_SEPARACION_MM;
	mundo_yaw_cero = GARRA_YAW_CERO_DEG;
	mundo_desvio_a = 0.0f;
	mundo_ruido_mm = 0.0f;
	mundo_retardo_ticks = 0;
	pulsos_pos = 0;
	srand(1U);
	for (int b = 0; b < I2C_CANT_BUSES; b++) {
		enc_presente[b] = true;
		bus_trabado[b] = false;
		enc_cuentas[b] = 0U;
		enc_iman[b] = AS5600_IMAN_OK;
		enc_signo_real[b] = 1.0f;
		enc_cero_real[b] = 0.0f;
		enc_ganancia_real[b] = 1.0f;
		enc_desvio_deg[b] = 0.0f;
	}
	Motion_Init();
	Gripper_Init();
	EjeZ_Init();
	Secuencia_Init();
	Consola_Init(true);
}

/** Actualiza los encoders con la junta real que resulta de cada pulso. */
static void actualizar_encoders(void)
{
	if (!enc_sigue_servo) {
		return;
	}
	for (int j = 0; j < JUNTA_CANT; j++) {
		if (pwm_activo[j]) {
			float th = Servo_JuntaDePulso((servo_id_t)j, pwm_us[j]);

			encoder_en_junta(j, enc_ganancia_real[j] * th + enc_desvio_deg[j]);
		}
	}
}

int correr(int max)
{
	int n = 0;

	while (n < max) {
		uint32_t antes[PWM_CANT];
		bool activo_antes[PWM_CANT];

		memcpy(antes, pwm_us, sizeof(antes));
		memcpy(activo_antes, pwm_activo, sizeof(activo_antes));
		ms_simulado += TICK_MS;
		avanzar_motores();
		actualizar_encoders();
		Motion_Tick();
		guardar_pulsos();
		EjeZ_Tick();
		Gripper_Tick();
		Secuencia_Tick();
		Consola_DespuesDelTick();
		n++;

		for (int i = 0; i < PWM_CANT; i++) {
			int con_signo = (int)pwm_us[i] - (int)antes[i];
			int delta = abs(con_signo);
			int acel = abs(con_signo - delta_previo[i]);

			assert(delta <= DELTA_US_MAX);
			if (delta > delta_us_visto) {
				delta_us_visto = delta;
			}
			if (acel > acel_us_vista) {
				acel_us_vista = acel;
			}
			delta_previo[i] = con_signo;
		}
		if (!Motion_Ocupado() && !EjeZ_Ocupado() && !Gripper_Ocupado() && !Secuencia_Activa()) {
			break;
		}
	}
	return n;
}

const char *tipear(const char *texto)
{
	borrar_salida();
	entrada_len = strlen(texto);
	assert(entrada_len < sizeof(entrada));
	memcpy(entrada, texto, entrada_len);
	entrada_pos = 0U;

	/* Como el loop: varias vueltas de a CONSOLA_BYTES_POR_VUELTA. */
	while (entrada_pos < entrada_len) {
		Consola_Procesar();
	}
	return salida;
}

int contar(const char *texto, const char *buscado)
{
	int n = 0;

	for (const char *p = strstr(texto, buscado); p != NULL; p = strstr(p + 1, buscado)) {
		n++;
	}
	return n;
}

void borrar_salida(void)
{
	salida_len = 0U;
	salida[0] = '\0';
}
