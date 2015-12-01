
%--------------------------------------
%  generate training and test set
%
%--------------------------------------

    %------------------------------------------
    %load data
    %------------------------------------------
    
    
    h = waitbar(0, 'Loading Scans');
    
    %exp_corridor_1_person_adaboostfile
    %exp_buero_2_personen_adaboostfile;
    
    %exp_flur_1_person;
    
    exp_buero_2_personen
    
    close(h);
    
    per_test = 50;

    %------------------------------------------
    %preprocessing
    %final format for one cluster
    %[id class label weight new_type f_1 ... f_n] 
    %------------------------------------------        
    index_test = 1;
    index_training = 1; 
    h = waitbar(0, 'Creating training and test sets');
    [r n_scans] = size(scan);
    for i=1:n_scans
       if ( (rand(1) .* 100) < per_test )
           test_set(index_test) = scan(i);
           index_test = index_test + 1;
       else
           training_set(index_training) = scan(i);
           index_training = index_training +1;
       end
    waitbar(i/n_scans);
    end
    close(h);
    
    
    test_file = '~/code/oscar/phd/mit_Kai/people_tracking/data/exp_buero_2_personen/with_naive_class_und_motion/test_set.mat';
    training_file = '~/code/oscar/phd/mit_Kai/people_tracking/data/exp_buero_2_personen/with_naive_class_und_motion/training_set.mat';
    
    scan = test_set;
    save(test_file, 'scan');
    
    
    scan = training_set;
    save(training_file, 'scan');
    
    
    
    
    
    
    
    
    
    
    
    
    