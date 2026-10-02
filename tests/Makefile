# Tests en PC del nucleo portable (cinematica, planificador, parser).
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
TESTS   = test_cinematica$(EXE) test_planificador$(EXE) test_parser$(EXE)

.PHONY: all test sim clean

all: $(TESTS) sim_move$(EXE)

test: $(TESTS)
	./test_cinematica$(EXE)
	./test_planificador$(EXE)
	./test_parser$(EXE)

sim: sim_move$(EXE)

%$(EXE): %.c $(NUCLEO) $(HEADERS)
	$(CC) $(CFLAGS) -o $@ $< $(NUCLEO) $(LDLIBS)

clean:
	$(RM) $(TESTS) sim_move$(EXE)
