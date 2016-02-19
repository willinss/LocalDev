CFLAGS = -Wall -O3 -DPLATFORM_LINUX -DTINYXML_CFG -I/usr/local/include

LDFLAGS = -lpthread

CFLAGS+=`pkg-config --cflags opencv`

LDFLAGS+=`pkg-config --libs opencv`

CXX = g++

all : filter.will

filter.will : filter.o
	$(CXX) -o filter.will filter.o $(LDFLAGS)

filter.o : filter.cpp
	$(CXX) $(CFLAGS) -c filter.cpp
#all : CVfilter.will CAfilter.will MCVfilter.will MCAfilter.will collect.will 

#CVfilter.will : CVfilter.o
#	$(CXX) -o CVfilter.will CVfilter.o $(LDFLAGS)

#CVfilter.o : CVfilter.cpp
#	$(CXX) $(CFLAGS) -c CVfilter.cpp

#CAfilter.will : CAfilter.o
#	$(CXX) -o CAfilter.will CAfilter.o $(LDFLAGS)

#CAfilter.o : CAfilter.cpp
#	$(CXX) $(CFLAGS) -c CAfilter.cpp

#MCVfilter.will : MCVfilter.o
#	$(CXX) -o MCVfilter.will MCVfilter.o $(LDFLAGS)

#MCVfilter.o : MCVfilter.cpp
#	$(CXX) $(CFLAGS) -c MCVfilter.cpp

#MCAfilter.will : MCAfilter.o
#	$(CXX) -o MCAfilter.will MCAfilter.o $(LDFLAGS)

#MCAfilter.o : MCAfilter.cpp
#	$(CXX) $(CFLAGS) -c MCAfilter.cpp

#collect.will : collect.o
#	$(CXX) -o collect.will collect.o $(LDFLAGS)

#collect.o : collect.cpp
#	$(CXX) $(CFLAGS) -c collect.cpp

clean:
	-rm *.o
	-rm *.will
