CFLAGS = -Wall -O3 -DPLATFORM_LINUX -DTINYXML_CFG -I/usr/local/include

LDFLAGS = -lpthread

CFLAGS+=`pkg-config --cflags opencv`

LDFLAGS+=`pkg-config --libs opencv`

CXX = g++

all : cv.will ca.will

cv.will : cv.o
	$(CXX) -o cv.will cv.o  $(LDFLAGS)

ca.will : ca.o
	$(CXX) -o ca.will ca.o $(LDFLAGS)

cv.o : cv.cpp
	$(CXX) $(CFLAGS) -c cv.cpp

ca.o : ca.cpp
	$(CXX) $(CFLAGS) -c ca.cpp


clean:
	-rm *.o
	-rm *.will
