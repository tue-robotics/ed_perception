 

-----------------------------------------------
Author:

Oscar Martinez Mozos

Email: omozos@gmail.com
Homepage: http://www.informatik.uni-freiburg.de/~omartine
-----------------------------------------------


This file describes how to prepare data and train a classifier for leg detection. I think the programs used in the different steps in this task are simple enough to quickly understand what they do. Have a look at the code.


1. Preparing the classifier.
	1.1 Getting people data.
	1.2 Converting the scans from ROS.
	1.3 Segmenting the scans.
	1.4 Selecting people.
	1.5 Plotting segments.
	1.6 Extracting features.
	1.7 Training the classifier.
	1.8.Format of the different files.

2. References.


----------------------------
1. Preparing the classifier
----------------------------

-------------------------
1.1 Getting people data
-------------------------
An easy way to get training data of legs is to use the method presented in the section 5.1 of [1]. In summary, one leaves an empty rectangle in front of the laser and persons walk inside this rectangle. Then all the beams ending inside the rectangle belong to people. In my case a rectangle in front of the laser is defined by the following 4 parameters:

threshold_left = distance to the left of the laser
threshold_right = distance to the right of the laser
threshold_opposite = distance to the opposite edge
threshold_front = distance to the edge directly in fron of the laser

Have a look at the drawing "training_data.pdf" in this directoy to get an explanation of each parameter.

To get data use the provided "logger_node" and write data to a file.


-------------------------
1.3 Segmenting the scans
-------------------------

Segment the scans using a jump codition between beams [2]. I usually set the threshold to 0.15 m:

# ./segment_scans/segment_scans scans.dat segments.dat 0.15


You can plot the process of segmentation in Gnuplot using the -gnuplot option:

# ./segment_scans/segment_scans scans.dat segments.dat 0.15 -gnuplot | gnuplot


-------------------------
1.4 Selecting people
-------------------------

(See Sect 1.1)

An easy way to get training data of legs is to use the method presented in the section 5.1 of the paper [1]. In summary, one leaves an empty rectangle in front of the laser and persons walk inside this rectangle. Then all the beams ending inside the rectangle belong to people.
 
The program select_people selects people using this method. It uses 4 parameters :

threshold_left = distance to the left of the laser
threshold_right = distance to the right of the laser
threshold_opposite = distance to the opposite edge
threshold_front = distance to the edge directly in fron of the laser

These thresholds define a rectangle in front of the laser. Have a look at the drawing "training_data.pdf" in this directoy to get an explanation of each parameter. For example, to select the segments inside the rectangle defined by 0.85 x 1.35 x 2.6 x 0.05, use:

# ./select_people/select_people segments.dat people.dat 0.85 1.35 2.6 0.05 

Segments inside the rectangle receive the label =1 (people), and segments outside the rectangle receive the label=0 (non-people). 

To get more negative data just record scans in places with no people, and follow the previous steps. When selecting people just put all thresholds to zero and all segments will get the label =0:

# ./select_people/select_people segments_no_people.dat no_people.dat 0.0 0.0 0.0 0.0 


------------------------
1.5 Plotting segments
-------------------------
I wrote a simple utility to plot the segments in Gnuplot and check that they are correct: 

# ./plot_segments/plot_segments people.dat <class> -delay 0.05 -gnuplot  | gnuplot

You can select the <class> of segments to plot: 1 for people, 0 for non-people and -1 for all. The parameter -delay is used to show the different scans with a certain pause between them.


-------------------------
1.6 Extracting features
-------------------------
Now we need to extract the feature vector for each segment. I use features 1,2,3,6,7,8,9,10,11,12,13 from [2]. The following command should do the work:

# ./extract_features/extract_features people.dat features_people.dat

The file "features_people.dat" contains now the features.



---------------------------
1.7 Training the classifier
---------------------------

To train the AdaBoost classifier we need positive and negative examples. In our case positive and negative segments. Positive segments withe their corresponding features are in the previous file "features.dat". In this file theres is one line for each segment containing the features and their type: +1 for positive examples (people), and 0 for negative examples (non-people).

To increase the number of negative examples (which is always good), you can concatenate the file with only negative examples:

# cat features_people.dat features_non_people.dat >> trainig.dat 

The training is done in MATLAB. The scripts are located under the directoy AdaBoost. Have a look at the script "run.m", I think it is quite clear. The line:

export_hypotheses(hypo_file,best_features);

creates the hypotheses file that can be used directly with the people_detector node in ROS.



-------------------------
1.8. Format of the files
-------------------------

Here I give an overview of the different file formats used during training. In any case jus have a look a t the programs that generated them.


* When converting the scans from ROS (Sect. 1.2) we get a file (new_scans.dat) with the following format:

[scan_id] [timestamp] [n_lasers] [n_readings] [angle(0)] [range(0)] ...  [angle(n_readings)] [range(n_readings)] [NEWLINE]
[scan_id] [timestamp] [n_lasers] [n_readings] [angle(0)] [range(0)] ...  [angle(n_readings)] [range(n_readings)] [NEWLINE]
.
.


One line for each scan. With the following variables:

[scan_id]: identification for the scan. Currrently not used.
[timestamp]: timestamp of the scan. Currrently not used.
[n_lasers]: number of laser. Currently set to 1.
[n_readings]: number of beams in the laser
[angle(i)]: angle of beam i
[range(i)]: range of beam i


* When segmenting the scans (Sect. 1.3) the resulting format (segments.dat) is:

[scan_id] [timestamp] [n_lasers] [laser] [n_segments] [n_beams_segment(0)] [num_readings] [position] [angle(0,0)] [range(0,0)] ... [angle(0,n_beams_segment(0))] [range(0,n_beams_segment(0)] [NEWLINE]

[scan_id] [timestamp] [n_lasers] [laser] [n_segments] [n_beams_segment(1)] [num_readings] [position] [angle(0,1)] [range(0,1)] ... [angle(0,n_beams_segment(1))] [range(0,n_beams_segment(1)] 
.
.
.


One line for each segment with the following values:

[scan_id]: identification for the scan. Currrently not used.
[timestamp]: timestamp of the scan. Currrently not used.
[n_lasers]: number of lasers. Currently set to 1.
[laser]: selected laser. Currently 0.
[n_segments]: number os segments in the scan.
[n_beams_segment(i)]: number of beams in segment i
[num_readings]: ignore this value
[position]: ignore this value
[angle(i,j)]: angle of beam j in segment i
[range(i,j)]: range of beam j in segment i


* When selecting the people (Sect. 1.4) the resulting format (people.dat) is:

[scan_id] [timestamp] [n_lasers] [laser] [n_segments] [type(0)] [n_beams_segment(0)] [num_readings] [position] [angle(0,0)] [range(0,0)] ... [angle(0,n_beams_segment(0))] [range(0,n_beams_segment(0)] [NEWLINE]

[scan_id] [timestamp] [n_lasers] [laser] [n_segments] [type(1)] [n_beams_segment(1)]  [num_readings] [position] [angle(0,1)] [range(0,1)] ... [angle(0,n_beams_segment(1))] [range(0,n_beams_segment(1)] 
.
.
.


Exactly the same format as the previous but uncluding a type for each segment:

[type(i)] = type of segment i. +1 = people, 0 = non-people



* When extracting the features (Sect. 1.6) the resulting format (features_people.dat) is:

[scan_id] [timestamp] [type(0)] [n_beams_segment(0)] [num_readings] [position] [angle(0,0)] [range(0,0)] ... [angle(0,n_beams_segment(0))] [range(0,n_beams_segment(0)]  [num_features(0)]  [features(0,k)] .. [features(0,num_features(0))] [NEWLINE]
.
.
.


One line for each segment containing:

[scan_id]: identification for the scan. Currrently not used.
[timestamp]: timestamp of the scan. Currrently not used.
[type(i)] = type of segment i. +1 = people, 0 = non-people
[n_beams_segment(i)]: number of beams in segment i
[num_readings]: ignore this value
[position]: ignore this value
[angle(i,j)]: angle of beam j in segment i
[range(i,j)]: range of beam j in segment i
[num_features(i)]= number of features for the segment
[features(i,k)]= value of feature k for the segment i



----------------------
2. References
----------------------

[1] O. M. Mozos, R. Kurazume, and T. Hasegawa,
Multi-part people detection using 2D range data,
International Journal of Social Robotics, vol. 2, pp. 31-40, March 2010.

[2] K. O. Arras, O. M. Mozos, and W. Burgard,
Using boosted features for the detection of people in 2D range data,
in Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pp. 3402-3407, 2007.
























