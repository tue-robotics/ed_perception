


LFLAGS += -L../../common -lcommon

IFLAGS += -I./ -I../../common

CFLAGS += -O0 -Wall
CXXFLAGS += $(CFLAGS)

TARGET = plot_segments

OBJS =  plot_segments.o 

all: $(TARGET) 

$(TARGET): $(OBJS) 
	g++ $(CXXFLAGS) $(IFLAGS) -o  $(TARGET) $(OBJS) $(LFLAGS)

plot_segments.o: plot_segments.cpp
	g++ $(CXXFLAGS) $(IFLAGS) -c -o plot_segments.o plot_segments.cpp


clean:
	rm -f $(TARGET)  *.o








