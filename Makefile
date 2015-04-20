
CFLAGS=-O2 -march=armv5
CCFLAGS=-std=c++11 -D_GLIBCXX_USE_NANOSLEEP
DEPS=ev3dev.h
LIBS=-Llib -lev3dev -lstdc++ -lm
RANLIB=ranlib

obj/%.o: %.cpp $(DEPS)
	mkdir -p $(@D)
	$(CC) -c -o $@ $< $(CFLAGS) $(CCFLAGS)

lib/libev3dev.a: obj/ev3dev.o
	mkdir -p $(@D)
	$(AR) rc $@ $^ && $(RANLIB) $@

bin/ev3dev-lang-test: lib/libev3dev.a obj/ev3dev-lang-test.o
	mkdir -p $(@D)
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)

bin/ev3dev-lang-demo: lib/libev3dev.a obj/ev3dev-lang-demo.o
	mkdir -p $(@D)
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS) -lpthread

bin/remote_control-test: lib/libev3dev.a obj/remote_control-test.o
	mkdir -p $(@D)
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)

bin/drive-test: lib/libev3dev.a obj/drive-test.o
	mkdir -p $(@D)
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS) -lpthread

bin/button-test: lib/libev3dev.a obj/button-test.o
	mkdir -p $(@D)
	$(CC) -o $@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)

.PHONY: all clean

clean:
	rm -f obj/* lib/* bin/* *~

all: \
	lib/libev3dev.a \
	bin/ev3dev-lang-test \
	bin/ev3dev-lang-demo \
	bin/remote_control-test \
	bin/drive-test \
	bin/button-test
