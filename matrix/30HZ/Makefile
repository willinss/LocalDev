CFLAGS = -Wall -O3 -DPLATFORM_LINUX -DTINYXML_CFG -I/usr/local/include

LDFLAGS = -lpthread

CFLAGS+=`pkg-config --cflags opencv`

LDFLAGS+=`pkg-config --libs opencv`

CXX = g++

all : test.will

test.will : filter.o
	$(CXX) -o test.will filter.o $(LDFLAGS)

filter.o : filter.cpp
	$(CXX) $(CFLAGS) -c filter.cpp


clean:
	-rm *.o
	-rm *.will
