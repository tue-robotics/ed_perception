
%--------------------------------------
%  generate training and test set
%
%--------------------------------------

    %------------------------------------------
    %load data
    %------------------------------------------
    
    clear;
    h = waitbar(0, 'Loading Scans');
    
    exp_flur_1_person
     
    close(h);
    
    
    %------------------------------------------
    %preprocessing
    %final format for one cluster
    %[id class label weight new_type f_1 ... f_n] 
    %------------------------------------------        
    index_test = 1;
    index_training = 1; 
    [r n_scans] = size(scan);

    training_size = n_scans ./ 2;
    h = waitbar(0, 'Creating training set');    
    for i=1:training_size
        training_set(index_training) = scan(i);
        index_training = index_training +1;
        waitbar( i ./ training_size );
    end
    close(h);
        
    test_size = n_scans - i;
    h = waitbar(0, 'Creating test set');    
    for j=i+1:n_scans
        test_set(index_test) = scan(j);
        index_test = index_test +1;
        waitbar( index_test ./ test_size );
    end
    close(h);    
    
%     test_file = '~/code/oscar/phd/mit_Kai/people_tracking/data/exp_buero_2_personen/with_naive_class_und_motion/test_set.mat';
%     training_file = '~/code/oscar/phd/mit_Kai/people_tracking/data/exp_buero_2_personen/with_naive_class_und_motion/training_set.mat';

    test_file = '~/code/oscar/phd/mit_Kai/people_tracking/data/exp_corridor_1_person/with_naive_classification_and_more_features/2_parts_sets/test_set.mat';
    training_file = '~/code/oscar/phd/mit_Kai/people_tracking/data/exp_corridor_1_person/with_naive_classification_and_more_features/2_parts_sets/training_set.mat';

    scan = test_set;
    save(test_file, 'scan');
    
    
    scan = training_set;
    save(training_file, 'scan');
    
    
    
    
    
    
    
    
    
    
    
    
    