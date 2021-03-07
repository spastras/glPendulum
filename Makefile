CC=gcc
CFLAGS=-O0 -Wall -Wno-deprecated-declarations
ifeq ($(shell uname -s), Darwin)
	LDFLAGS=-lpthread -lm -framework OpenGL -framework GLUT
else
	LDFLAGS=-lpthread -lm -lGL -lGLU -lglut
endif
RM=rm -f

export CC CFLAGS LDFLAGS RM

all: glPendulum

glPendulum: glPendulum.o
	$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@

%.o: %.c %.h
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	$(RM) glPendulum.o glPendulum
