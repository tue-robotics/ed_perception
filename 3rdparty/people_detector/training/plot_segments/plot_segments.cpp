

#include <stdio.h>
#include <math.h>

#include <iostream>
#include <fstream>
#include <string.h>

#include <MultiScan.h>

using namespace std;

MultiScan *mScan=NULL;

double threshold_left;
double threshold_right;
double threshold_opposite;
double shift;

float delay;

int show_type;

FILE *fin;

//-----------------------------------------------------
// Plot 1 scan
//-----------------------------------------------------
void outputScan()
{

	double shift = 0;
	double x, y;
	double z[6]={0.30, 1.33, 1.51, 0.0, 0.0, 0.0};	

	cout << "set nokey" << endl;
	cout << "set size square" << endl;
	cout << "set xrange [-4.5:4.5]" << endl;
	cout << "set yrange [-4.5:4.5]" << endl;
	cout << "set zrange [0:2]" << endl;
	cout << "set xlabel \"X\" " << endl;
	cout << "set ylabel \"Y\" " << endl;
	cout << "set zlabel \"Z\" " << endl;


//	cout << "set label 1 \"" << mScan->scan_id << "\" at 3,3,1.7" << endl;
	
//	cout << "splot '-' with linespoints ps 1 pt 7  lc 0";
	cout << "splot '-'";


	for(int i=0; i< mScan->n_scans; i++) {
		for(int j=0; j < mScan->scans[i]->num(); j++) {
			//cout << ", '-' with linespoints ps 1 pt 7 lc " << j+1;	
			cout << ", '-'"; 
		}
	}
	cout << endl;

	// just for the center
	cout << " 0 0 0" << endl;
	cout << "e" << endl;

	for(int i=0; i< mScan->n_scans; i++) {
		for(int j=0; j < mScan->scans[i]->num(); j++) {
			Segment *s = mScan->scans[i]->getElement(j);
			if ( (s->type == show_type) || (show_type == -1) ) {
				for ( int k=0; k<s->num(); k++) {
						Beam *b = s->getElement(k);
						x = b->range * cos(b->angle+shift);
						y = b->range * sin(b->angle+shift);
						cout << x << " " << y << " " << z[i] << endl;
				}
			}
			else {
				cout << 0 << " " << 0 << " " << 0 << endl;
			}
			cout << "e" << endl;
		}								
	} 

	usleep( (int)(delay*1000000.0) );
//	cout << "unset label 1" << endl;
} 


//-----------------------------------------------------
// 
//-----------------------------------------------------
int readSegments() 
{
	int scan_index;
	int type;
	int n_beams;	
	int ret;	
	float angle, range;
	float timestamp;
	int n_lasers;
	int level;
	int n_segments;
	Segment *s;
	int num_readings;
	int position;

	while ( !feof(fin) ) {
		//start reading segments
		ret = fscanf(fin, "%d %f %d %d %d %d %d", &scan_index, &timestamp, &n_lasers, &level, &n_segments, &type, &n_beams);
		if ( ret == 7 ) {

			if (mScan!=NULL) {
				if ( scan_index != mScan->scan_id ) {
					outputScan();
					delete mScan;
					mScan=NULL;
				}
			}
			
			if (mScan==NULL) {
				mScan = new MultiScan(n_lasers);			
				mScan->scan_id = scan_index;

			//cerr << "mScan->scan_id: " << mScan->scan_id << endl;

				mScan->timestamp = timestamp;		
			}	

			// read multi scan
			s = new Segment;
			s->type = type;
			for(int k=0; k<n_beams; k++) {
				Beam *b = new Beam;;
				fscanf(fin, " %d %d %f %f", &num_readings, &position, &angle, &range );
				b->range = range;
				b->angle = angle;
				s->add(b);				
			}
			mScan->scans[level]->add(s);
		} // end if ( ret == 3 ) {			
	} //end while ( !feof(fin) )
	
	if ( mScan != NULL ) {
		outputScan();			
		delete mScan;
		mScan = NULL;
	}

	return 1;
}


//-----------------------------------------------------
// MAIN
//-----------------------------------------------------
int main(int argc, char **argv) {
	
	char *in_filename;
	delay = 0.0;
	shift = -M_PI_2;

	if (argc < 3) {
		cerr << "USE: " << argv[0] << " <input_file> <class> " << endl;
		cerr << "   options: " << endl;
		cerr << "      -delay seconds" << endl; 
		cerr << "      -shift angle(degrees)" << endl;
		return 1;
	}

	// some parameters preprocessing	
	in_filename = argv[1];
	show_type = atoi(argv[2]);
	
	//parameters
	cerr << "filename: " << in_filename  << endl;
	cerr << "class: " << show_type << endl;
	
	for (int i=3; i<argc; i++) {
		if ( strcmp(argv[i],"-delay")==0 ) {
			delay = atof(argv[i+1]);
		}
		if ( strcmp(argv[i],"-shift")==0 ) {
			shift = atof(argv[i+1]);
			shift = 2.0*M_PI*shift / 360.0; // radians 
		}
	}


	try {
		fin=fopen(in_filename, "r");
		if ( fin == NULL ) {
			fprintf(stderr, "ERROR opening %s\n", in_filename);
			throw -1;
		}
		
		readSegments();
	}
	catch (int e) {
	}

	if ( fin != NULL) {
		fclose(fin);
		fin=NULL;
	}


	return 0;
}





















