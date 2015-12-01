

COMMON=../../common

LFLAGS += -L../../features -L../../linalg -L../../common -lcommon -lfeatures -llinalg -lgsl -lgslcblas -lm    

IFLAGS += -I./ -I$(COMMON) -I../../features -I../../common

CFLAGS += -O3 -Wall
CXXFLAGS += $(CFLAGS)

TARGET = extract_features

OBJS +=  extract_features.o 

all: $(TARGET) 

$(TARGET): $(OBJS) 
	g++ $(CXXFLAGS) $(IFLAGS) -o  $(TARGET) $(OBJS) $(LFLAGS)

extract_features.o: extract_features.cpp 
	g++ $(CXXFLAGS) $(IFLAGS) -c -o extract_features.o extract_features.cpp

clean:
	rm -f $(TARGET)  *.o








