



LFLAGS += -L./ 
LFLAGS += -L../features 
LFLAGS += -L../linalg
LFLAGS += -L../common
 
LFLAGS += -lPeopleDetector 
LFLAGS += -lfeatures 
LFLAGS += -lcommon
LFLAGS += -llinalg 
LFLAGS += -lgsl -lgslcblas -lm    

IFLAGS += -I./ -I../common -I../features

CFLAGS += -O3 -Wall
CXXFLAGS += $(CFLAGS)

TARGET = classify

LIB_OBJS +=  WeakHypothesis.o 
LIB_OBJS +=  PeopleDetector.o 
OBJS +=  classify.o 

LIB=libPeopleDetector.a


all: $(LIB)  $(TARGET)

$(LIB): $(LIB_OBJS)
	rm -f $(LIB)
	ar -r $(LIB) $(LIB_OBJS)
	ranlib $(LIB)


$(TARGET): $(OBJS) $(LIB) 
	g++ $(CXXFLAGS) $(IFLAGS) -o  $(TARGET) $(OBJS) $(LFLAGS)


classify.o: classify.cpp 
	g++ $(CXXFLAGS) $(IFLAGS) -c -o classify.o classify.cpp

WeakHypothesis.o: WeakHypothesis.cpp ../features/libfeatures.a
	g++ $(CXXFLAGS) $(IFLAGS) -c -o WeakHypothesis.o WeakHypothesis.cpp


PeopleDetector.o: PeopleDetector.cpp WeakHypothesis.o
	g++ $(CXXFLAGS) $(IFLAGS) -c -o PeopleDetector.o PeopleDetector.cpp

clean:
	rm -f $(TARGET) $(LIB) *.o *.*~








