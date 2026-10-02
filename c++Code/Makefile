TARGET := main

.PHONY: all run clean

all: $(TARGET)
	./$(TARGET)

$(TARGET): main.cpp recruit.hpp
	g++ -O3 main.cpp recruit.hpp -o main

run: all

clean:
	rm -f $(TARGET)
