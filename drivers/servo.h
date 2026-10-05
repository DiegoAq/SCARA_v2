/**
 * @file servo.h
 * @brief Servos de las juntas: conversion angulo de junta <-> pulso (SPEC 3.1) y limites.
 *
 *     servo_deg = ZERO + SIGN * theta
 *     pulso_us  = PULSE_MIN + servo_deg * (PULSE_MAX - PULSE_MIN) / SERVO_RANGE_DEG
 *
 * Este modulo NO limita la velocidad: quien lo llama (motion) entrega en cada tick un
 * angulo que ya respeta JOINT_VMAX_DPS. No toca registros (usa pwm.h), asi que tambien
 * se prueba en la PC.
 */
#ifndef SERVO_H
#define SERVO_H

#include <stdbool.h>
#include <stdint.h>

/** Servos de junta. */
typedef enum {
	SERVO_HOMBRO = 0,	/**< theta1 */
	SERVO_CODO,			/**< theta2 */
	SERVO_CANT
} servo_id_t;

/**
 * @brief Inicializa el PWM y deja los servos deshabilitados (sin pulsos).
 */
void Servo_Init(void);

/**
 * @brief Habilita o deshabilita los pulsos de los dos servos de junta.
 *
 * Al habilitar, cada servo recibe el ultimo pulso configurado: configurar la postura
 * ANTES de habilitar. Un servo va a maxima velocidad hacia el primer pulso que recibe.
 */
void Servo_Habilitar(bool habilitar);

/**
 * @brief Comanda un angulo de junta.
 * @param id Servo.
 * @param theta_deg Angulo de junta (grados), dentro de [JOINT_MIN, JOINT_MAX].
 * @return false (y no cambia nada) si el angulo esta fuera de los limites articulares.
 */
bool Servo_ConfigurarJunta(servo_id_t id, float theta_deg);

/**
 * @brief Comanda un pulso crudo (diagnostico y calibracion). No mira los limites articulares.
 * @param pulso_us Ancho en us; se satura a [PULSE_MIN_US, PULSE_MAX_US].
 */
void Servo_ConfigurarPulso(servo_id_t id, uint32_t pulso_us);

/** @brief Ultimo angulo de junta comandado (grados). */
float Servo_JuntaComandada(servo_id_t id);

/** @brief Ultimo pulso comandado (us). */
uint32_t Servo_PulsoComandado(servo_id_t id);

/**
 * @brief Pulso que corresponde a un angulo de junta, saturado a [PULSE_MIN_US, PULSE_MAX_US].
 */
uint32_t Servo_PulsoDeJunta(servo_id_t id, float theta_deg);

/**
 * @brief Angulo de junta que corresponde a un pulso (inversa de Servo_PulsoDeJunta).
 */
float Servo_JuntaDePulso(servo_id_t id, uint32_t pulso_us);

#endif /* SERVO_H */
