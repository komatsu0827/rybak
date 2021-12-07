PREFIX = /usr
TARGET = sh4-linux

CC 	 = $(PREFIX)/bin/$(TARGET)-gcc
LIBS     = $(PREFIX)/$(TARGET)/lib
LIB	= -lm

DEBUG	 = 
WARN     = -Wall
OPTIMIZE = -O
DEFINES  = -D MULTI
CFLAGS   = $(DEBUG) $(WARN) $(DEFINES) $(OPTIMIZE) $(INCLUDES)
ARTLIB = ../artdriver/art_syscalls.o
OBJS = 1_main.o 2_calc_CPG.o 3_sensor.o 4_ctrl_Muscle.o 5_function.o 6_calc_Table.o 7_UDP_trans.o
ALL = main

all: $(ALL)

main : 1_main.o 2_calc_CPG.o 3_sensor.o 4_ctrl_Muscle.o 5_function.o 6_calc_Table.o 7_UDP_trans.o $(ARTLIB) ../lib/hrp3pio.o ../lib/tsc-sh.o ../artdriver/artapi.a
1_main.o : 1_main.c 2_calc_CPG.c 3_sensor.c 4_ctrl_Muscle.c 5_function.c 6_calc_Table.c 7_UDP_trans.c 8_define.h 9_user_param.h
2_calc_CPG.o : 2_calc_CPG.c 8_define.h 9_user_param.h
3_sensor.o : 3_sensor.c 8_define.h 9_user_param.h
4_ctrl_Muscle.o : 4_ctrl_Muscle.c 8_define.h 9_user_param.h
5_function.o : 5_function.c 8_define.h 9_user_param.h
6_calc_Table.o : 6_calc_Table.c 8_define.h 9_user_param.h
7_UDP_trans.o : 7_UDP_trans.c 8_define.h 9_user_param.h


$(OBJS) :
	$(CC) $(CFLAGS) -c $<

$(ALL) :
	$(CC) -static -o $@ $^ $(LIB)

clean:
	rm -f $(OBJS) *~
	rm -f $(ALL) 


