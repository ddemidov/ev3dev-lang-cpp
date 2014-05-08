CFLAGS=-O2 -march=armv5
CCFLAGS=-std=c++11
DEPS=ev3dev.h
OBJ=ev3dev.o
LIBS=-lstdc++

%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) $(CCFLAGS)

ev3dev-lang-test: $(OBJ) ev3dev-lang-test.o
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)

ev3dev-lang-demo: $(OBJ) ev3dev-lang-demo.o
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)

remote_control-test: $(OBJ) remote_control-test.o
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)

.PHONY: all clean

clean:
	rm -f *.o *~ ev3dev-lang-test ev3dev-lang-demo

all:  ev3dev-lang-test ev3dev-lang-demo remote_control-test
