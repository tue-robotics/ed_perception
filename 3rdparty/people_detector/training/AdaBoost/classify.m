

function classify( n_weak, best_features, patterns_test, positive_lclass, i_class, i_label, i_weight, i_new_type, i_shift)

    % erros
    error = 0;
    false_positives = 0;
    false_negatives = 0;
   
    n_positives  = sum( patterns_test(:,i_label)==1 );    
    n_negatives  = sum( patterns_test(:,i_label)==-1 );
    
    [r total] = size( patterns_test(1,:) );
    
    [n_patterns_test c] = size( patterns_test );
    fprintf('Classifying test set...');
    for i=1:n_patterns_test 
        label = patterns_test(i, i_label);
        [new_label conf] = adaboost_classify( best_features, n_weak, patterns_test(i, i_shift+1:total) );
        
        if (  label ~= new_label ) 
           error = error + 1; 
           if ( new_label == 1 )
               false_positives = false_positives + 1;
           else
               false_negatives = false_negatives + 1;
           end
        end
        
        % save the new label 
        patterns_test(i, i_label) = new_label;

        fprintf('Classifiying example %d/%d\n', i, n_patterns_test);
    end
    fprintf('Classifying test set...DONE\n');

    true_positives = n_positives - false_negatives;
    true_negatives = n_negatives - false_positives;

    false_postives_rate = (false_positives / n_negatives) .* 100;
    false_negatives_rate = (false_negatives / n_positives) .* 100;
    
    n_negatives;
    
    n_positives;

    s1 = 'Confussion matrix';
    s2 = ['            Positives    Negatives    Total '];
    s3 = ['Positives   ' mat2str(true_positives) '           ' mat2str(false_negatives) '          ' mat2str(n_positives) ];
    s4 = ['Negatives   ' mat2str(false_positives) '           ' mat2str(true_negatives) '          ' mat2str(n_negatives) ];
    strvcat(s1, s2, s3, s4)  
    
    
    
    classifcation_rate = (1.0 - ( error ./ n_patterns_test )) .* 100

end