CXXFLAGS = -O2 -Wall
LDFLAGS = -lm
all: test

clean:
	rm -f test *.o
