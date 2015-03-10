



LFLAGS += -L./ 
LFLAGS += -L../features 
LFLAGS += -L../linalg
LFLAGS += -L../common
LFLAGS += -L../PeopleDetector

 
LFLAGS += -lPeopleDetector 
LFLAGS += -lfeatures 
LFLAGS += -lcommon
LFLAGS += -llinalg 
LFLAGS += -lgsl -lgslcblas -lm    

IFLAGS += -I./ 
IFLAGS += -I../common 
IFLAGS += -I../features
IFLAGS += -I../PeopleDetector

CFLAGS += -O3 -Wall
CXXFLAGS += $(CFLAGS)


all:  template


template: template.o 
	g++ $(CXXFLAGS) $(IFLAGS) -o  template template.o $(LFLAGS)

template.o: template.cpp 
	g++ $(CXXFLAGS) $(IFLAGS) -c -o template.o template.cpp

clean:
	rm -f template *.o *.*~








