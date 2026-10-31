# Tests en PC del nucleo portable (cinematica, planificador, parser) y de la aplicacion.
# Excluido del build de CubeIDE. Necesita gcc y make en el PATH:
#   make test     compila y corre todos los tests
#   make sim      compila sim_move (CSV de una trayectoria)
#   make clean

CC      = gcc
# -Wdouble-promotion / -Wfloat-conversion: en el firmware un double es emulacion
# por software; aca se detecta cualquier promocion accidental en el nucleo.
CFLAGS  = -std=c11 -O2 -Wall -Wextra -Werror -Wdouble-promotion -Wfloat-conversion -I../app
LDLIBS  = -lm

ifeq ($(OS),Windows_NT)
EXE = .exe
endif

NUCLEO  = ../app/cinematica.c ../app/planificador.c ../app/parser.c ../app/metricas.c ../app/mapa.c
HEADERS = $(wildcard ../app/*.h) $(wildcard ../drivers/*.h)
SIMPLES = test_cinematica$(EXE) test_planificador$(EXE) test_parser$(EXE) test_mapa$(EXE)
CON_HW  = test_consola$(EXE) test_robot$(EXE) test_supervision$(EXE) test_trim$(EXE) test_busqueda$(EXE)
TESTS   = $(SIMPLES) $(CON_HW)

# La aplicacion se prueba con el hardware simulado en hw_sim.c (UART, SysTick, PWM, AS5600, I2C,
# motores, encoders, VL53L0X). De drivers/ solo se compila servo.c, que no toca registros.
# microrl es de terceros: se compila sin -Werror.
APP         = hw_sim.c ../app/console.c ../app/motion.c ../app/eje_z.c ../app/gripper.c \
              ../app/secuencia.c ../app/laseres.c ../app/busqueda.c ../drivers/servo.c $(NUCLEO)
APP_INC     = -I../drivers -I../lib/microrl
MICRORL_OBJ = microrl.o

# La supervision depende de constantes de config.h: se prueba con los dos encoders habilitados.
CON_ENCODERS = -DENC1_HABILITADO=1 -DENC2_HABILITADO=1

# Con el montaje actual del codo, las pasadas sobre la caja no entran en el anillo de trabajo. La
# busqueda completa se prueba con el codo remontado, que es lo que hace falta para usarla.
CON_ALCANCE = -DELBOW_ZERO_SERVO_DEG=20.0f -DR_MIN_OP_MM=150.0f

.PHONY: all test sim clean

all: $(TESTS) sim_move$(EXE)

test: $(TESTS)
	./test_cinematica$(EXE)
	./test_planificador$(EXE)
	./test_parser$(EXE)
	./test_mapa$(EXE)
	./test_consola$(EXE)
	./test_robot$(EXE)
	./test_supervision$(EXE)
	./test_trim$(EXE)
	./test_busqueda$(EXE)

sim: sim_move$(EXE)

$(MICRORL_OBJ): ../lib/microrl/microrl.c ../lib/microrl/microrl.h ../lib/microrl/microrl_config.h
	$(CC) -std=c11 -O2 -Wall -Wextra -I../lib/microrl -c -o $@ $<

test_consola$(EXE) test_robot$(EXE): %$(EXE): %.c hw_sim.h $(APP) $(MICRORL_OBJ) $(HEADERS)
	$(CC) $(CFLAGS) $(APP_INC) -o $@ $< $(APP) $(MICRORL_OBJ) $(LDLIBS)

test_supervision$(EXE): test_supervision.c hw_sim.h $(APP) $(MICRORL_OBJ) $(HEADERS)
	$(CC) $(CFLAGS) $(APP_INC) $(CON_ENCODERS) -o $@ $< $(APP) $(MICRORL_OBJ) $(LDLIBS)

test_trim$(EXE): test_supervision.c hw_sim.h $(APP) $(MICRORL_OBJ) $(HEADERS)
	$(CC) $(CFLAGS) $(APP_INC) $(CON_ENCODERS) -DTRIM_ENABLE=1 -o $@ $< $(APP) $(MICRORL_OBJ) $(LDLIBS)

test_busqueda$(EXE): test_busqueda.c hw_sim.h $(APP) $(MICRORL_OBJ) $(HEADERS)
	$(CC) $(CFLAGS) $(APP_INC) $(CON_ALCANCE) -o $@ $< $(APP) $(MICRORL_OBJ) $(LDLIBS)

%$(EXE): %.c $(NUCLEO) $(HEADERS)
	$(CC) $(CFLAGS) -o $@ $< $(NUCLEO) $(LDLIBS)

clean:
	$(RM) $(TESTS) sim_move$(EXE) $(MICRORL_OBJ)
