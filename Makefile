.DEFAULT_GOAL:=bmp180

CC:=arm-none-eabi-gcc
CFLAGS=-I.
CPPFLAGS:=-c -Wall -g3 -fstack-check -ftrack-macro-expansion=0 \
	-fno-diagnostics-show-caret

DEPS:= bmp180Driver.h bmp180Simulator.h
OBJ_DRIVER:= bmp180Driver.o
OBJ_TEST:= $(OBJ_DRIVER) main.o bmp180Simulator.o
%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) $(CPPFLAGS)

bmp180: $(OBJ_DRIVER)
	$(AR) rcs libbmp180.a $(OBJ_DRIVER)

test: $(OBJ_TEST)
	$(CC) $(OBJ_TEST) -o test