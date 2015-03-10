all:
	cd extract_features && make
	cd plot_segments && make
	cd segment_scans && make
	cd select_people && make

clean:
	cd extract_features && make clean && rm -f *~
	cd plot_segments && make clean && rm -f *~
	cd segment_scans && make clean && rm -f *~
	cd select_people && make clean && rm -f *~
	rm -f *~