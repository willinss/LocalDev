TARGET = ../output/Data_Collection

OBJECTS = DJI_Pro_App.o DJI_Pro_Hw.o DJI_Pro_Link.o DJI_Pro_Codec.o DJI_Pro_Rmu.o D_C.o tinyxml2.o DJI_Pro_Sample.o

CFLAGS = -Wall -O3 -DPLATFORM_LINUX -DTINYXML_CFG -Isrc/ -I/DJI_LIB -I/usr/local/include/opencv2 

LDFLAGS = -lpthread

CFLAGS+=`pkg-config --cflags opencv`

LDFLAGS+=`pkg-config --libs opencv`

Complile_Flag = Release

CXX = g++

$(TARGET) : $(OBJECTS)
	$(CXX) -o $(TARGET) $(OBJECTS) $(LDFLAGS)
	
D_C.o : ../D_C.cpp
	$(CXX) $(CFLAGS) -c ../D_C.cpp

DJI_Pro_App.o : ../DJI_LIB/DJI_Pro_App.cpp
	$(CXX) $(CFLAGS) -c ../DJI_LIB/DJI_Pro_App.cpp
	
DJI_Pro_Hw.o : ../DJI_LIB/DJI_Pro_Hw.cpp
	$(CXX) $(CFLAGS) -c ../DJI_LIB/DJI_Pro_Hw.cpp

DJI_Pro_Link.o : ../DJI_LIB/DJI_Pro_Link.cpp
	$(CXX) $(CFLAGS) -c ../DJI_LIB/DJI_Pro_Link.cpp
	
DJI_Pro_Codec.o : ../DJI_LIB/DJI_Pro_Codec.cpp
	$(CXX) $(CFLAGS) -c ../DJI_LIB/DJI_Pro_Codec.cpp

DJI_Pro_Rmu.o: ../DJI_LIB/DJI_Pro_Rmu.cpp
	$(CXX) $(CFLAGS) -c ../DJI_LIB/DJI_Pro_Rmu.cpp

tinyxml2.o: ../tinyxml2.cpp
	$(CXX) $(CFLAGS) -c ../tinyxml2.cpp

DJI_Pro_Sample.o : ../DJI_Pro_Sample.cpp
	$(CXX) $(CFLAGS) -c ../DJI_Pro_Sample.cpp
	
clean:
	-rm D_C.o
	-rm $(TARGET)
