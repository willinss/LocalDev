TARGET = ../output/DJI_Onboard_API_Cmdline_Test

OBJECTS = main.o DJI_Pro_App.o DJI_Pro_Hw.o DJI_Pro_Link.o DJI_Pro_Sample.o DJI_Pro_Codec.o DJI_Pro_Rmu.o tinyxml2.o ImageThread.o Config.o Features.o GraphUtils.o HaarFeature.o HaarFeatures.o HistogramFeatures.o ImageRep.o LaRank.o MultiFeatures.o RawFeatures.o Sampler.o Tracker.o

CFLAGS = -Wall -O3 -DPLATFORM_LINUX -DTINYXML_CFG -Isrc/ -I/src/DJI_LIB -I/usr/local/include/opencv2 -I/usr/local/include/Eigen

LDFLAGS = -lpthread

CFLAGS+=`pkg-config --cflags opencv`

LDFLAGS+=`pkg-config --libs opencv`

Complile_Flag = Release

CXX = g++

$(TARGET) : $(OBJECTS)
	$(CXX) -o $(TARGET) $(OBJECTS) $(LDFLAGS)
	
main.o : ../src/main.cpp
	$(CXX) $(CFLAGS) -c ../src/main.cpp

DJI_Pro_App.o : ../src/DJI_LIB/DJI_Pro_App.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_LIB/DJI_Pro_App.cpp
	
DJI_Pro_Hw.o : ../src/DJI_LIB/DJI_Pro_Hw.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_LIB/DJI_Pro_Hw.cpp

DJI_Pro_Link.o : ../src/DJI_LIB/DJI_Pro_Link.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_LIB/DJI_Pro_Link.cpp

DJI_Pro_Sample.o : ../src/DJI_Pro_Sample.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_Pro_Sample.cpp
	
DJI_Pro_Codec.o : ../src/DJI_LIB/DJI_Pro_Codec.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_LIB/DJI_Pro_Codec.cpp

DJI_Pro_Rmu.o: ../src/DJI_LIB/DJI_Pro_Rmu.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_LIB/DJI_Pro_Rmu.cpp

ImageThread.o : ../src/ImageThread.cpp
	$(CXX) $(CFLAGS) -c ../src/ImageThread.cpp

tinyxml2.o : ../src/tinyxml2.cpp
	$(CXX) $(CFLAGS) -c ../src/tinyxml2.cpp 

Config.o : ../src/Config.cpp
	$(CXX) $(CFLAGS) -c ../src/Config.cpp 

Features.o : ../src/Features.cpp
	$(CXX) $(CFLAGS) -c ../src/Features.cpp

GraphUtils.o : ../src/GraphUtils.cpp
	$(CXX) $(CFLAGS) -c ../src/GraphUtils.cpp

HaarFeature.o : ../src/HaarFeature.cpp
	$(CXX) $(CFLAGS) -c ../src/HaarFeature.cpp

HaarFeatures.o : ../src/HaarFeatures.cpp
	$(CXX) $(CFLAGS) -c ../src/HaarFeatures.cpp

HistogramFeatures.o : ../src/HistogramFeatures.cpp
	$(CXX) $(CFLAGS) -c ../src/HistogramFeatures.cpp

ImageRep.o : ../src/ImageRep.cpp
	$(CXX) $(CFLAGS) -c ../src/ImageRep.cpp

LaRank.o : ../src/LaRank.cpp
	$(CXX) $(CFLAGS) -c ../src/LaRank.cpp

MultiFeatures.o : ../src/MultiFeatures.cpp
	$(CXX) $(CFLAGS) -c ../src/MultiFeatures.cpp

RawFeatures.o : ../src/RawFeatures.cpp
	$(CXX) $(CFLAGS) -c ../src/RawFeatures.cpp

Sampler.o : ../src/Sampler.cpp
	$(CXX) $(CFLAGS) -c ../src/Sampler.cpp

Tracker.o : ../src/Tracker.cpp
	$(CXX) $(CFLAGS) -c ../src/Tracker.cpp
	
clean:
	-rm main.o
	-rm ImageThread.o
	-rm $(TARGET)
