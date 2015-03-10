


COMMON_DIR = ../../common/

LFLAGS += -L$(COMMON_DIR)
LFLAGS += -lcommon

IFLAGS += -I./ -I$(COMMON_DIR)

CFLAGS += -O0 -Wall
CXXFLAGS += $(CFLAGS)

TARGET = segment_scans

OBJS =  segment_scans.o 


all: $(TARGET) 

$(TARGET): $(OBJS) 
	g++ $(CXXFLAGS) $(IFLAGS) -o  $(TARGET) $(OBJS) $(LFLAGS)

segment_scans.o: segment_scans.cpp
	g++ $(CXXFLAGS) $(IFLAGS) -c -o segment_scans.o segment_scans.cpp


clean:
	rm -f $(TARGET)  *.o








