################################################
#Set the compiler
CXX= g++
# Enter Compiler flags here
CXXFLAGS=-c -g -Wall
# Compilation (add flags as needed)
CV_FLAGS=`pkg-config opencv --cflags`
# Linker flags (add flags as needed)
CV_LIBS=`pkg-config opencv --libs`
######### DO NOT EDIT ABOVE ####################



# List all .cpp files in SOURCES, separated by spaces
SOURCES=main.cpp Kalman.cpp
OBJECTS=$(SOURCES:.cpp=.o)
# List all .h files in DEPS
DEPS=Kalman.h
EXECUTABLE=kalmanTrack
######### DO NOT EDIT BELOW ####################



all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS) 
	$(CXX) $(OBJECTS) -o $@ $(CV_LIBS)

# Set the dependency for 'file'.o to 'file'.cpp as well as any files in $DEPS [file.h]
%.o: %.cpp $(DEPS)
	$(CC) $(CXXFLAGS) $(CV_FLAGS) $< -o $@

clean:
	rm -rf *.o $(EXECUTABLE)
