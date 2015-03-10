#include <PeopleDetector.h>
#include <iostream>

using namespace std;


//-----------------------------------------------------
// MAIN
//-----------------------------------------------------
int main(int argc, char **argv) {
	
	
	FILE *f_hypotheses=NULL;
	char *hypotheses_filename=NULL;
	int num_hypotheses;
	
	// Use
	if (argc < 3) {
		cerr << "USE: " << argv[0] << " <hypotheses_file> <number of weak hypotheses>" << endl;
		return 1;
	}

	//---------------------------------------
	// Input to the detector
	//
	// We need the hypotheses file learnt by AdaBoost
	// We have to select the number of weak hypotheses to use. This number should be <= to the number of hypoheses learnt by AdaBoost.
	//---------------------------------------			
	hypotheses_filename = argv[3];
	num_hypotheses = atoi(argv[4]);

	cerr << "hypotheses_filename: " << hypotheses_filename << endl;
	cerr << "num_hypotheses: " << num_hypotheses << endl;
	
	// create a People Detector object
	PeopleDetector pd;

	// list to store segments
	dyntab_segments *list_segments=NULL;
	
	try {
		f_hypotheses = fopen(hypotheses_filename, "r");
		if ( f_hypotheses == NULL ) {
			cerr << "ERROR opening " << hypotheses_filename << endl;
			throw -1;
		}	
		
		// configure the people detector object		
		pd.load(f_hypotheses, num_hypotheses);

		list_segments= new dyntab_segments(100);
		
		// getting range messages from your system and classify them
		while (1) {
			
			// get the message
			//msg = getMessage();		
			
			// transforn the message into a format readable by the detector
			//num_readings = msg->num_readings; // number of beams
			//double *angles = new double(num_readings); // array of angles
			//double *ranges = new double(num_readings); // array of measurements
					
			
			// This is a list of segments.
			// For more information have a look at ../common/dynamictable.h/hxx
			
					
			
			// segment the scan
			//pd.segmentScan(threshold, num_readings, angles, ranges, list_segments);
			
			// Classiy segments
			for (int i=0; i < list_segments->num(); i++) {
				Segment *s = list_segments->getElement(i);
				// discard segments with less than three points
				if ( s->num() < 3 ) {
					s->type = -1;
				}
				else {	
					pd.classify(s); 
					if ( s->type == 1) {
						// person found
					}
					else {
						// not a person
					}
				}		
			}	
			
			// delete the list of segments 
			list_segments->setAutoDelete(true);
			delete list_segments;
			list_segments = NULL;
			
			// free memory
			//delete [] ranges;
			//delete [] angles;
		}
	}
	catch (int e) {
	}

	fclose( f_hypotheses );


	return 0;
}

