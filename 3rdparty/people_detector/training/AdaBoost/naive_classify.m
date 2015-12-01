

function naive_classify(patterns_test)

    % indexes
    i_id = 1;
    i_class = 2;
    i_label = 3;
    i_class_naive_s = 4;
    i_class_naive_m = 5;
    i_weight = 6;
    i_new_type = 7;
    i_shift = 7;

    
    
    n_positives  = sum( patterns_test(:,i_label)==1 );    
    n_negatives  = sum( patterns_test(:,i_label)==-1 );
   
    error = 0;
    false_positives = 0;
    false_negatives = 0;       
    
    [n_patterns_test c] = size( patterns_test );
    h= waitbar(0, 'Classifying test set');
    for i=1:n_patterns_test 
        hand_class = patterns_test(i, i_class);
        
        %naive_class = patterns_test(i, i_class_naive_s);
        naive_class = patterns_test(i, i_class_naive_m);

        if (  hand_class ~= naive_class ) 
           error = error + 1; 
           if ( naive_class == 1 )
               false_positives = false_positives + 1;
           else
               false_negatives = false_negatives + 1;
           end
        end       
        waitbar(i / n_patterns_test);
    end
    close(h);

    true_positives = n_positives - false_negatives;
    true_negatives = n_negatives - false_positives;

    false_positives;
    n_negatives;
    
    false_negatives;
    n_positives;
        
    false_postives_rate = (false_positives / n_negatives) .* 100;
    false_negatives_rate = (false_negatives / n_positives) .* 100;
    
    
    s1 = 'Confussion matrix';
    s2 = ['            Positives    Negatives    Total '];
    s3 = ['Positives   ' mat2str(true_positives) '           ' mat2str(false_negatives) '          ' mat2str(n_positives) ];
    s4 = ['Negatives   ' mat2str(false_positives) '           ' mat2str(true_negatives) '          ' mat2str(n_negatives) ];
    strvcat(s1, s2, s3, s4)  
    
    
    
    classifcation_rate = (1.0 - ( error ./ n_patterns_test )) .* 100    
    
end %function
























