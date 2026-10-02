# Tests en PC del nucleo portable (cinematica, planificador, parser) y de la consola.
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

NUCLEO  = ../app/cinematica.c ../app/planificador.c ../app/parser.c
HEADERS = $(wildcard ../app/*.h)
TESTS   = test_cinematica$(EXE) test_planificador$(EXE) test_parser$(EXE) test_consola$(EXE)

# La consola se prueba con la UART y el SysTick simulados dentro de test_consola.c:
# de drivers/ solo se usan los headers. microrl es de terceros: se compila sin -Werror.
CONSOLA     = ../app/console.c ../app/motion.c ../app/parser.c
CONSOLA_INC = -I../drivers -I../lib/microrl
MICRORL_OBJ = microrl.o

.PHONY: all test sim clean

all: $(TESTS) sim_move$(EXE)

test: $(TESTS)
	./test_cinematica$(EXE)
	./test_planificador$(EXE)
	./test_parser$(EXE)
	./test_consola$(EXE)

sim: sim_move$(EXE)

$(MICRORL_OBJ): ../lib/microrl/microrl.c ../lib/microrl/microrl.h ../lib/microrl/microrl_config.h
	$(CC) -std=c11 -O2 -Wall -Wextra -I../lib/microrl -c -o $@ $<

test_consola$(EXE): test_consola.c $(CONSOLA) $(MICRORL_OBJ) $(HEADERS)
	$(CC) $(CFLAGS) $(CONSOLA_INC) -o $@ $< $(CONSOLA) $(MICRORL_OBJ) $(LDLIBS)

%$(EXE): %.c $(NUCLEO) $(HEADERS)
	$(CC) $(CFLAGS) -o $@ $< $(NUCLEO) $(LDLIBS)

clean:
	$(RM) $(TESTS) sim_move$(EXE) $(MICRORL_OBJ)
