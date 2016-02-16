CFLAGS = -Wall -O3 -DPLATFORM_LINUX -DTINYXML_CFG -I/usr/local/include

LDFLAGS = -lpthread

CFLAGS+=`pkg-config --cflags opencv`

LDFLAGS+=`pkg-config --libs opencv`

CXX = g++

all : ca.will

ca.will : ca.o
	$(CXX) -o ca.will ca.o $(LDFLAGS)

ca.o : ca.cpp
	$(CXX) $(CFLAGS) -c ca.cpp


clean:
	-rm *.o
	-rm *.will
