

#include <stdio.h>
#include <math.h>

#include <iostream>
#include <fstream>

#include <dynamictable.h>
#include <Segment.h>
#include <AllFeatures.h>

using namespace std;

typedef DynamicTable<Feature> dyntab_features;

dyntab_features *list_features=NULL;
FILE *f_segments=NULL, *f_output=NULL;

//-----------------------------------------------------
// 
//-----------------------------------------------------
int checkSegment(Segment *s) {

	// segments with more than 2 points
	if ( s->num() < 3) {
		return 0;  // too few points
	}
	
	// check for strange segments
	for (int i=0; i<s->num(); i++) {	
		Beam *b = s->getElement(i);
		if ( b->range < 0.001  ) {
			return 0; // error range
		}
	}
	
	
	return 1; // ok
}



//-----------------------------------------------------
// 
//-----------------------------------------------------
void createFeatureList()
{
	list_features = new dyntab_features(20);

	// add features
	Feature_1 *f_1 = new Feature_1;
	list_features->add(f_1);	

	Feature_2 *f_2 = new Feature_2;
	list_features->add(f_2);	

	Feature_3 *f_3 = new Feature_3;
	list_features->add(f_3);	

	Feature_6 *f_6 = new Feature_6;
	list_features->add(f_6);	

	Feature_7 *f_7 = new Feature_7;
	list_features->add(f_7);	

	Feature_8 *f_8 = new Feature_8;
	list_features->add(f_8);	

	Feature_9 *f_9 = new Feature_9;
	list_features->add(f_9);	

	Feature_10 *f_10 = new Feature_10;
	list_features->add(f_10);	

	Feature_11 *f_11 = new Feature_11;
	list_features->add(f_11);	

	Feature_12 *f_12 = new Feature_12;
	list_features->add(f_12);	

	Feature_13 *f_13 = new Feature_13;
	list_features->add(f_13);	
} 


//-----------------------------------------------------
// 
//-----------------------------------------------------
int readSegments() 
{
	int scan_index, type, n_beams;
	int ret;
	Beam *b;
	Feature *f;
	float value, angle, range;
	int c=0;
	float timestamp;
	int readings, position;
	int n_levels; // always 1
	int level; // always 0
	int n_segments;  // total number of segments
	

	while ( !feof(f_segments) ) {
		//start reading segments
		ret = fscanf(f_segments, "%d %f %d %d %d %d %d", &scan_index, &timestamp, &n_levels, &level, &n_segments, &type,  &n_beams);
	
		//cerr << scan_index << " " << timestamp << " " <<  n_levels << " " << level << " " << n_segments << " " << type  << " " << n_beams << endl;

		if ( ret == 7 ) {
			c++;
			cerr << c << endl;
		
			Segment *s = new Segment;
			s->scan_id = scan_index;
			s->type = type;
			

			for (int i=0; i<n_beams; i++) {			
				// read the first point
				b = new Beam;
				fscanf(f_segments, "%d %d %f %f", &readings, &position, &angle, &range );
				b->range = range;
				b->angle = angle;
				s->add(b);				
			}

			if ( checkSegment(s) ) {
				
				fprintf(f_output, "%d %f %d %d", scan_index, timestamp, type, n_beams);
				for (int i=0; i<n_beams; i++) {			
					b = s->getElement(i);
 					fprintf(f_output, " %.8f %.8f", b->angle, b->range );				
				}
				
				// extract features and write
				fprintf(f_output, " %d", list_features->num() );	
				for (int i=0; i<list_features->num(); i++) {
					f = list_features->getElement(i);
					value = f->calculateFeature(s);
					fprintf(f_output, " %.8f", value );	
				}
				//cerr <<  " features: " << list_features->num();
				fprintf(f_output, "\n");
			}	

			if ( s != NULL ) {
				delete s;
				s = NULL;
			}
		} // end if ( ret ==54 ) {			
	} //end while ( !feof(f_segments) )

	return 1;
}


//-----------------------------------------------------
// MAIN
//-----------------------------------------------------
int main(int argc, char **argv) {
	
	char *segments_filename;
	char *output_filename;


	if (argc < 3) {
		cerr << "USE: " << argv[0] << " <input_file> <output_file>" << endl;
		return 1;
	}

	// parameters preprocessing	
	segments_filename = argv[1];
	output_filename = 	argv[2];

	// echo parameters
	cerr << "input_filename: " << segments_filename << endl;
	cerr << "output_filename: " << output_filename << endl;
	
	try {
		f_segments = fopen(segments_filename, "r");
		if ( f_segments == NULL ) {
			cerr << "ERROR opening " << segments_filename << endl;
			throw -1;
		}	

		f_output = fopen(output_filename, "w");
		if ( f_output == NULL ) {
			cerr << "ERROR opening " << output_filename << endl;
			throw -1;
		}	

		createFeatureList();
		readSegments();
	}
	catch (int e) {
	}

	fclose( f_segments );
	fclose( f_output );

	if (list_features != NULL ) {
		list_features->setAutoDelete(true);
		delete list_features;
	} 

	return 0;
}





















