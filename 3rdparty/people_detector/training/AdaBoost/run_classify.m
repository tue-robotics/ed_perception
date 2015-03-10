


positive_label = 1;

% indexes
i_class = 1;  % original type
i_label = 2;  % (-1, +1)
i_weight = 3;  
i_new_type = 4; % (-1, +1)
i_shift =4 ;


%------------------------------------------
% classify the test examples
%------------------------------------------
%file = '/home/oscar/projects/Kyushu_uni/v_2/experiments/multi_level/1/by_level/training_level_2_oscar.dat';
file = '/home/oscar/projects/Kyushu_uni/v_2/experiments/multi_level/4/test_0/by_level/training_level_1_oscar.dat';
%hypo_mat_file= '/home/oscar/projects/Kyushu_uni/v_2/experiments/multi_level/1/by_level/hypo_level_0.mat';   
hypo_mat_file= '/home/oscar/projects/Kyushu_uni/v_2/experiments/multi_level/4/training/by_level/hypo_level_1.mat';

[patterns_test n_positive n_negative ] = load_segments(file, positive_label);
load(hypo_mat_file);

classify(100, best_features, patterns_test, positive_label, i_class, i_label, i_weight, i_new_type, i_shift);

