

positive_label = 1;

% indexes
i_class = 1;  % original type
i_label = 2;  % (-1, +1)
i_weight = 3;  
i_new_type = 4; % (-1, +1)
i_shift =4 ;


%------------------------------------------
% load training_set
%------------------------------------------

file = '/home/oscar/projects/TUM/people_detection_ROS/training/t_bv_lab.txt';
hypo_mat_file = '/home/oscar/projects/TUM/people_detection_ROS/training/hypo.mat';   
hypo_file = '/home/oscar/projects/TUM/people_detection_ROS/training/hypo.dat';   
 
[ patterns n_positive n_negative] = load_segments(file, positive_label);


[r total] = size( patterns(1,:) );
n_features = total - i_shift;

%------------------------------------------
% run adaboost
%   best_features has format:
%     [index alpha threshold misclassified direction sum_weights]
%------------------------------------------
n_weak = 200;
best_features = adaboost( patterns, n_positive, n_negative, n_features, n_weak, i_class, i_label, i_weight, i_new_type, i_shift);

save(hypo_mat_file, 'best_features');

export_hypotheses(hypo_file,best_features);
 
%------------------------------------------
% classify the test examples
%------------------------------------------
file = '/home/oscar/projects/TUM/people_detection_ROS/training/t_bv_lab.txt';
hypo_mat_file = '/home/oscar/projects/TUM/people_detection_ROS/training/hypo.mat';   

[patterns_test n_positive n_negative ] = load_segments(file, positive_label);
load(hypo_mat_file);

classify(100, best_features, patterns_test, positive_label, i_class, i_label, i_weight, i_new_type, i_shift);



    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    