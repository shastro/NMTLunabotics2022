CXX=g++
CFLAGS=-g -Wall -lpthread -lsFoundation20
OBJFLAGS=$(CFLAGS) -I../../teknic/inc/inc-pub -c

SOURCES=$(wildcard joystick/*.cpp) $(wildcard motor/*.cpp) $(wildcard *.cpp)
OBJECTS=$(patsubst %.cpp,%.o,$(SOURCES))
TARGET=bluetooth_motor_control

$(TARGET): $(OBJECTS)
	$(CXX) $^ $(CFLAGS) -o $@

%.o: %.cpp
	$(CXX) $^ $(OBJFLAGS) -o $@

clean:
	$(RM) $(OBJECTS) $(TARGET)
