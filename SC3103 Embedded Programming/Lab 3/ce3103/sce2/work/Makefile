# Compiler flags
CC = gcc
CFLAGS = -Wall -O0
SOURCES = hello-main.c hello-funct1.c hello-funct2.c
OBJECTS = $(SOURCES:.c=.o)

.PHONY: all clean debug

all: helloworld

debug: helloworld-d

gprof: CFLAGS += -pg
grpof: helloworld-d

helloworld: $(OBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS) -o helloworld
	
hello-main.o: hello-main.c hello.h
	$(CC) $(CFLAGS) -c hello-main.c

hello-funct1.o: hello-funct1.c hello.h
	$(CC) $(CFLAGS) -c hello-funct1.c

helloworld-d: $(OBJECTS)
	$(CC) $(CFLAGS) -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@


clean:
	rm -f $(OBJECTS) helloworld helloworld-d

