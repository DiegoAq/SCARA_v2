/**
 * @file test_parser.c
 * @brief Tests en PC del interprete de comandos.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "parser.h"

/** Arma argc/argv a partir de una lista de palabras. */
#define INTERPRETAR(cmd, ...) \
	Parser_Interpretar((int)(sizeof((const char *[]){ __VA_ARGS__ }) / sizeof(const char *)), \
					   (const char *const[]){ __VA_ARGS__ }, (cmd))

static bool cerca(float a, float b)
{
	return fabsf(a - b) <= 1e-4f * (1.0f + fabsf(b));
}

static void test_numeros(void)
{
	float f;
	int32_t n;

	assert(Parser_LeerFloat("20", &f) && cerca(f, 20.0f));
	assert(Parser_LeerFloat("-12.5", &f) && cerca(f, -12.5f));
	assert(Parser_LeerFloat("+0.125", &f) && cerca(f, 0.125f));
	assert(Parser_LeerFloat(".5", &f) && cerca(f, 0.5f));
	assert(Parser_LeerFloat("7.", &f) && cerca(f, 7.0f));
	assert(Parser_LeerFloat("300.123", &f) && cerca(f, 300.123f));

	assert(!Parser_LeerFloat("", &f));
	assert(!Parser_LeerFloat("-", &f));
	assert(!Parser_LeerFloat(".", &f));
	assert(!Parser_LeerFloat("12a", &f));
	assert(!Parser_LeerFloat("1e3", &f));
	assert(!Parser_LeerFloat("1.2.3", &f));
	assert(!Parser_LeerFloat("--5", &f));
	assert(!Parser_LeerFloat("12345678", &f));
	assert(!Parser_LeerFloat(NULL, &f));

	assert(Parser_LeerEntero("1500", &n) && n == 1500);
	assert(Parser_LeerEntero("-3", &n) && n == -3);
	assert(!Parser_LeerEntero("1500.5", &n));
	assert(!Parser_LeerEntero("", &n));
	assert(!Parser_LeerEntero("9999999999", &n));
	printf("OK  numeros\n");
}

static void test_move(void)
{
	comando_t c;

	assert(INTERPRETAR(&c, "MOVE", "300", "100") == PARSER_OK);
	assert(c.id == CMD_MOVE && cerca(c.x_mm, 300.0f) && cerca(c.y_mm, 100.0f) && !c.tiene_z);

	assert(INTERPRETAR(&c, "move", "x300", "Y-100.5", "z20") == PARSER_OK);
	assert(c.id == CMD_MOVE && cerca(c.x_mm, 300.0f) && cerca(c.y_mm, -100.5f));
	assert(c.tiene_z && cerca(c.z_mm, 20.0f));

	/* Por nombre, en cualquier orden. */
	assert(INTERPRETAR(&c, "Move", "Y100", "X300") == PARSER_OK);
	assert(cerca(c.x_mm, 300.0f) && cerca(c.y_mm, 100.0f));

	assert(INTERPRETAR(&c, "MOVE", "300") == PARSER_ERR_CANT_ARGS);
	assert(c.id == CMD_NINGUNO);
	assert(INTERPRETAR(&c, "MOVE", "1", "2", "3", "4") == PARSER_ERR_CANT_ARGS);
	assert(INTERPRETAR(&c, "MOVE", "X300", "X100") == PARSER_ERR_ARGUMENTO);
	assert(INTERPRETAR(&c, "MOVE", "X300", "Z10") == PARSER_ERR_ARGUMENTO);
	assert(INTERPRETAR(&c, "MOVE", "A300", "100") == PARSER_ERR_ARGUMENTO);
	assert(INTERPRETAR(&c, "MOVE", "300", "abc") == PARSER_ERR_ARGUMENTO);
	assert(INTERPRETAR(&c, "MOVE", "X", "100") == PARSER_ERR_ARGUMENTO);
	printf("OK  MOVE\n");
}

static void test_otros(void)
{
	comando_t c;

	assert(INTERPRETAR(&c, "joint", "-30", "45.5") == PARSER_OK);
	assert(c.id == CMD_JOINT && cerca(c.q1_deg, -30.0f) && cerca(c.q2_deg, 45.5f));
	assert(INTERPRETAR(&c, "JOINT", "10") == PARSER_ERR_CANT_ARGS);

	assert(INTERPRETAR(&c, "SPEED", "50") == PARSER_OK && c.id == CMD_SPEED && cerca(c.valor, 50.0f));
	assert(INTERPRETAR(&c, "accel", "120") == PARSER_OK && c.id == CMD_ACCEL && cerca(c.valor, 120.0f));
	assert(INTERPRETAR(&c, "SPEED") == PARSER_ERR_CANT_ARGS);
	assert(INTERPRETAR(&c, "SPEED", "rapido") == PARSER_ERR_ARGUMENTO);

	assert(INTERPRETAR(&c, "PULSE", "1", "1500") == PARSER_OK);
	assert(c.id == CMD_PULSE && c.servo == 1 && c.pulso_us == 1500);
	assert(INTERPRETAR(&c, "PULSE", "1", "1500.5") == PARSER_ERR_ARGUMENTO);

	assert(INTERPRETAR(&c, "TRACE", "on") == PARSER_OK && c.id == CMD_TRACE && c.activar);
	assert(INTERPRETAR(&c, "TRACE", "OFF") == PARSER_OK && !c.activar);
	assert(INTERPRETAR(&c, "TRACE", "1") == PARSER_ERR_ARGUMENTO);

	assert(INTERPRETAR(&c, "RAW") == PARSER_OK && c.id == CMD_RAW && !c.activar);
	assert(INTERPRETAR(&c, "RAW", "live") == PARSER_OK && c.id == CMD_RAW && c.activar);
	assert(INTERPRETAR(&c, "RAW", "1") == PARSER_ERR_ARGUMENTO);
	assert(INTERPRETAR(&c, "RAW", "LIVE", "extra") == PARSER_ERR_CANT_ARGS);

	assert(INTERPRETAR(&c, "GRIPPER", "open") == PARSER_OK && c.gripper == GRIPPER_ABRIR);
	assert(INTERPRETAR(&c, "GRIPPER", "CLOSE") == PARSER_OK && c.gripper == GRIPPER_CERRAR);
	assert(INTERPRETAR(&c, "GRIPPER", "35") == PARSER_OK);
	assert(c.gripper == GRIPPER_APERTURA && cerca(c.valor, 35.0f));

	assert(INTERPRETAR(&c, "ORIENT", "-45") == PARSER_OK && c.id == CMD_ORIENT && cerca(c.valor, -45.0f));

	static const struct { const char *txt; cmd_id_t id; } simples[] = {
		{ "HELP", CMD_HELP }, { "home", CMD_HOME }, { "Stop", CMD_STOP },
		{ "STATUS", CMD_STATUS }, { "ANGLE", CMD_ANGLE },
		{ "CALIB", CMD_CALIB }, { "i2cscan", CMD_I2CSCAN }, { "ZTEST", CMD_ZTEST },
		{ "SETZ0", CMD_SETZ0 },
	};
	for (unsigned i = 0; i < sizeof(simples) / sizeof(simples[0]); i++) {
		assert(INTERPRETAR(&c, simples[i].txt) == PARSER_OK && c.id == simples[i].id);
		assert(INTERPRETAR(&c, simples[i].txt, "1") == PARSER_ERR_CANT_ARGS);
	}
	printf("OK  resto de los comandos\n");
}

static void test_errores(void)
{
	comando_t c;

	assert(INTERPRETAR(&c, "MOVER", "1", "2") == PARSER_ERR_COMANDO && c.id == CMD_NINGUNO);
	assert(INTERPRETAR(&c, "MOV") == PARSER_ERR_COMANDO);
	assert(INTERPRETAR(&c, "") == PARSER_VACIO);
	assert(Parser_Interpretar(0, NULL, &c) == PARSER_VACIO);
	assert(INTERPRETAR(NULL, "HELP") == PARSER_ERR_COMANDO);
	printf("OK  errores\n");
}

int main(void)
{
	printf("== test_parser ==\n");
	test_numeros();
	test_move();
	test_otros();
	test_errores();
	printf("test_parser: TODO OK\n");
	return 0;
}
