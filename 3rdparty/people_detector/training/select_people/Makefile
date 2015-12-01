



LFLAGS += -L../../features -lfeatures
LFLAGS += -L../../common -lcommon

IFLAGS += -I./ -I../../common -I../../features

CFLAGS += -O0 -Wall
CXXFLAGS += $(CFLAGS)

TARGET = select_people

OBJS =  select_people.o 

all: $(TARGET) 

$(TARGET): $(OBJS) 
	g++ $(CXXFLAGS) $(IFLAGS) -o  $(TARGET) $(OBJS) $(LFLAGS)

select_people.o: select_people.cpp
	g++ $(CXXFLAGS) $(IFLAGS) -c -o select_people.o select_people.cpp

clean:
	rm -f $(TARGET)  *.o








