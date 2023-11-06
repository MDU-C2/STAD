CXX = g++
CXXFLAGS = -std=c++17
LIBS = -lbluetooth -lpthread
INCLUDES = -Iinclude

# List of source files
SRCS = main.cpp comms.cpp src/attitude.cpp src/dllvalidator.cpp src/packet.cpp src/rtcmmessage.cpp src/types.cpp src/error_detection.cpp src/packetfinder.cpp src/searcher.cpp src/util.cpp src/compositedata.cpp src/event.cpp src/port.cpp src/sensors.cpp src/utilities.cpp src/conversions.cpp src/ezasyncdata.cpp src/position.cpp src/serialport.cpp src/vntime.cpp src/criticalsection.cpp src/memoryport.cpp src/rtcmlistener.cpp src/thread.cpp

# Output binary
OUTPUT = communication

$(OUTPUT): $(SRCS)
	$(CXX) $(CXXFLAGS) $(SRCS) $(INCLUDES) -o $(OUTPUT) $(LIBS)

.PHONY: clean

clean:
	rm -f $(OUTPUT)