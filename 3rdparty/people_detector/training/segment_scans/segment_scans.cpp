

#include <stdio.h>
#include <math.h>

#include <iostream>
#include <fstream>

#include <string.h>

#include <dynamictable.h>
#include <MultiScan.h>

using namespace std;

int flag_gnuplot;
int flag_log;
FILE *fout=NULL;
FILE *fin=NULL;

int scan_counter;
 
MultiScan *mScan=NULL;

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
				for ( int k=0; k<s->num(); k++) {
					Beam *b = s->getElement(k);
					x = b->range * cos(b->angle+shift);
					y = b->range * sin(b->angle+shift);
					cout << x << " " << y << " " << z[i] << endl;
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
// MAIN
//-----------------------------------------------------
void readScans(double threshold) 
{
	int ret;
	int scan_id;
	int n_lasers;
	float timestamp;
	int n_readings;
	float angle, range;
	Segment *s;
	Beam *b, *b_prev;

	while ( !feof(fin) ) {
		//start reading one scan
	
		ret = fscanf(fin, "%d %f %d %d", &scan_id, &timestamp, &n_lasers, &n_readings);

		//cerr <<  scan_id << " " << timestamp << " " << n_lasers << " " <<  n_readings << endl;

		if ( ret == 4 ) {

			cerr << ".ss" << endl;
			
			mScan= new MultiScan(n_lasers);

			mScan->scan_id = scan_id;
			mScan->timestamp = timestamp;
	
			for(int l=0; l<n_lasers; l++) {				
				s = new Segment;
				//start with a segment
				for (int i=0; i<n_readings; i++) {			
					// read the first point
					b = new Beam;
					fscanf(fin, " %f %f", &angle, &range );
					b->range = range;
					b->angle = angle;	
					b->num_readings = n_readings;
					b->position = i;				
					if ( s->num() > 0 ) {
						if ( fabs(b_prev->range - b->range) < threshold ) {
							// same segment
							b_prev = b;
							s->add(b);					
						}
						else {
							mScan->scans[l]->add(s);
							s = new Segment;
							s->add(b);
							b_prev = b;
						} 
					}	
					else {
						b_prev = b;
						s->add(b);	
					}
				}
				mScan->scans[l]->add(s);
			}
			outputScan();
			if ( mScan != NULL ) {
				delete mScan;
				mScan = NULL;
			}
		} // end if ( ret == 4 ) {			
	} //end while ( !feof(stdin) )

}


//-----------------------------------------------------
// MAIN
//-----------------------------------------------------
int main(int argc, char **argv) {

	char* out_filename;
	char* in_filename;
	fout = NULL;
	double threshold; 
	
	
	flag_gnuplot =0;	
	flag_log =0;

	if (argc < 4) {
		cerr << "USE: " << argv[0] << " " << " <input_file> <output_file> <threshold (m)>  [options]" << endl;
		cerr << "	options:" << endl;
		cerr << "		-gnuplot" << endl; 	
		return 1;
	}

	// some parameters preprocessing	
	in_filename = argv[1];
	out_filename = argv[2];
	threshold = atof(argv[3]);	
	
	
	for (int i=4; i<argc; i++) {
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
		
		readScans(threshold);
	}
	catch (int e) {
	}

	
	if ( mScan != NULL ) {
		delete mScan;
		mScan = NULL;
	}

	if (fout != NULL) {
		fclose(fout);
	}

	if (fin != NULL) {
		fclose(fin);
	}

	return 0;
}





















