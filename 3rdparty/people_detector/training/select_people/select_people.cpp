

#include <stdio.h>
#include <math.h>

#include <iostream>
#include <fstream>

#include <dynamictable.h>
#include <MultiScan.h>

#include <string.h>

#include <Feature_6.h>

using namespace std;

typedef DynamicTable<Segment> dyntab_segments;

double threshold_left;
double threshold_right;
double threshold_opposite;
double threshold_front;
double angle_step;

MultiScan *mScan=NULL;

FILE *fout=NULL;
FILE *fin=NULL;

int flag_gnuplot;

//-----------------------------------------------------
// Plot 1 scan
//-----------------------------------------------------
void outputScan()
{

	double shift = 0;
	double x, y;
	double z[6]={0.30, 1.33, 1.51, 0.0, 0.0, 0.0};	

	if ( flag_gnuplot ) {

		cout << "set nokey" << endl;
		cout << "set size square" << endl;
		cout << "set xrange [-4.5:4.5]" << endl;
		cout << "set yrange [-4.5:4.5]" << endl;
		cout << "set xlabel \"X\" " << endl;
		cout << "set ylabel \"Y\" " << endl;
		cout << "set zlabel \"Z\" " << endl;

//		cout << "splot '-' with linespoints ps 1 pt 7  lc 0";
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
				if (s->type == 1)
					for ( int k=0; k<s->num(); k++) {
						Beam *b = s->getElement(k);
						x = b->range * cos(b->angle+shift);
						y = b->range * sin(b->angle+shift);
						cout << x << " " << y << " " << z[i] << endl;
					}
				else {
					cout << 0 << " " << 0 << " " << 0 << endl;
				}
				cout << "e" << endl;
			}								
		}
	} // end if ( flag_gnuplot )


	if ( fout != NULL ) {
		for(int i=0; i< mScan->n_scans; i++) {			
			for(int j=0; j < mScan->scans[i]->num(); j++) {
				Segment *s = mScan->scans[i]->getElement(j);
				fprintf(fout, "%d %.8f %d", mScan->scan_id, mScan->timestamp, mScan->n_scans);
				fprintf(fout, " %d", i); //level i
				fprintf(fout, " %d", mScan->scans[i]->num() ); // # segments for level i				
				fprintf(fout, " %d", s->type ); // # type of segment j in level i
				fprintf(fout, " %d", s->num() ); // # beams for segment j in level i
				for ( int k=0; k<s->num(); k++) {
					Beam *b = s->getElement(k);
					fprintf(fout, " %d %d %.8f %.8f", b->num_readings, b->position, b->angle, b->range);
				}
				fprintf(fout, "\n");	
			}								
		}		
	} // end if ( flag_log)
} 


//-----------------------------------------------------
// 
//-----------------------------------------------------
int checkBeam(Beam *b){


	double x, y;
	double angle_step_rad = M_PI_2;

	x = b->range * cos(b->angle + angle_step_rad);
	y = b->range * sin(b->angle + angle_step_rad);

	if ( (x > -threshold_left && x < threshold_right) 
		 && 
		 (y > threshold_front && y < threshold_opposite) ) {
		return 1;
	}
	else {
		return 0;
	}

// 	if ( b->range > 0.0 && b->range < 2.0 )
// 		return 1;
// 	else
// 		return 0;

}


//-----------------------------------------------------
// MAIN
//-----------------------------------------------------
int readSegments() 
{
	int scan_index;
	int n_beams;	
	int ok=1;
	int ret;	
	float angle, range;
	Feature_6 f6;
	double l;
	float timestamp;
	int n_lasers;
	int level;
	int n_segments;
	Segment *s;
	int n_readings;
	int position;
	
	mScan = NULL;
	while ( !feof(fin) ) {
		//start reading segments
		ret = fscanf(fin, "%d %f %d %d %d %d", &scan_index, &timestamp, &n_lasers, &level, &n_segments, &n_beams);
			
		if ( ret == 6 ) {
			cerr << ".sp" << endl;
			// new scan
			
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
			s->type=1;  // in principle a person	
			for(int k=0; k<n_beams; k++) {
				Beam *b = new Beam;
				fscanf(fin, " %d %d %f %f", &n_readings, &position, &angle, &range );
				b->range = range;
				b->angle = angle;
				b->num_readings = n_readings;
				b->position = position;
				s->add(b);				
				if ( !checkBeam(b) ) {
					s->type=0;  // not a person
				}
			}
			
/*			// check segment length	
			l=f6.calculateFeature(s);
			if ( l < 0.02 ) {
				s->type = 0;
			}
			// only segments with more than 2 points
			if ( s->num() > 2 ) {
				mScan->scans[level]->add(s);			
			}			*/

			mScan->scans[level]->add(s);			
		} // end if ( ret == 3 ) {			
	} //end while ( !feof(fin) )

	if (mScan!=NULL) {
		outputScan();
		delete mScan;
		mScan=NULL;
	}

	return ok;
}


//-----------------------------------------------------
// MAIN
//-----------------------------------------------------
int main(int argc, char **argv) {
	
	char *in_filename;
	char *out_filename;
	

	if (argc < 7) {
		cerr << "USE: " << argv[0] << " " << "<input_file> <output_file> <threshold_left (m)>  <threshold_right (m)> <threshold_opposite (m)> <threshold_front (m)> [options]" << endl;
		cerr << "	options:" << endl;
		cerr << "		-gnuplot" << endl; 	
		return 1;
	}

	// some parameters preprocessing	
	in_filename = argv[1];
	out_filename = argv[2];
	threshold_left = atof(argv[3]);
	threshold_right = atof(argv[4]);
	threshold_opposite = atof(argv[5]);
	threshold_front = atof(argv[6]);		
	
	// parameters
	cerr << "input filename : " << in_filename << endl;
	cerr << "output filename : " << out_filename << endl;
	cerr << "threshold_left: " << threshold_left << endl;
 	cerr << "threshold_right: " << threshold_right << endl;
	cerr << "threshold_opposite: " << threshold_opposite << endl;
	cerr << "threshold_front: " << threshold_front << endl;

	for (int i=7; i<argc; i++) {
		if ( strcmp(argv[i],"-gnuplot")==0 ) {
			flag_gnuplot = 1;
		}
	}


	try {
		fin=fopen(in_filename, "r");
		if ( fin == NULL ) {
			fprintf(stderr, "ERROR opening %s\n", in_filename);
			throw -1;
		}
		
		fout=fopen(out_filename, "w");
		if ( fout == NULL ) {
			fprintf(stderr, "ERROR opening %s\n", out_filename);
			throw -1;
		} 

		readSegments();
	}
	catch (int e) {
	}
	
	if( fin!= NULL ) {
		fclose(fin);
	}

	if( fin!= NULL ) {
		fclose(fout);
	}

	return 0;
}





















