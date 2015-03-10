

function classify_test( n_weak, best_features, positive_label, test_file, results_file)

    fprintf('Loading test file...');
    load(test_file);
    fprintf('DONE\n');
    
    
    
    % errors
    error = 0;
    false_positives = 0;
    false_negatives = 0;
    n_positives = 0;
    n_negatives = 0;
    
    
    fprintf('Classifying test set...\n');
    total_clusters = 0;
    
    n_steps = size(exp.step, 2);
    for i=1:n_steps
        fprintf('Classifying Scan %d\n', i); 
        n_clusters = size(exp.step(i).clusters, 2);
        for j=1:n_clusters
           class = exp.step(i).clusters(j).class;
           [r c] = size( class );
           if ( c ~= 0 ) 
               
               if ( class == positive_label )
                    label = 1;
                    n_positives = n_positives + 1;
               else
                    label = -1;
                    n_negatives = n_negatives + 1;
               end   
            
               pattern = [exp.step(i).clusters(j).f(:).value]; 
               pattern = [pattern(1:2) pattern(4:end)]; % ignore feature number 3: distance to sensor
               [new_label conf] = adaboost_classify( best_features, n_weak, pattern );            

               if (  label ~= new_label ) 
                   error = error + 1; 
                   if ( new_label == 1 )
                        false_positives = false_positives + 1;
                   else
                        false_negatives = false_negatives + 1;
                   end
               end            

               % keep new classification
               total_clusters = total_clusters + 1;
               exp.step(i).clusters(j).adaboost_label = new_label;
           end    
        end
    end
    fprintf('Classifying scans...DONE\n');
    fprintf('Saving new data...');
    save(results_file, 'exp', '-v6');
    fprintf('DONE\n');

    
    true_positives = n_positives - false_negatives;
    true_negatives = n_negatives - false_positives;

    false_postives_rate = (false_positives / n_negatives) .* 100;
    false_negatives_rate = (false_negatives / n_positives) .* 100;
    
    s1 = 'Confussion matrix';
    s2 = ['            Positives    Negatives    Total '];
    s3 = ['Positives   ' mat2str(true_positives) '           ' mat2str(false_negatives) '          ' mat2str(n_positives) ];
    s4 = ['Negatives   ' mat2str(false_positives) '           ' mat2str(true_negatives) '          ' mat2str(n_negatives) ];
    strvcat(s1, s2, s3, s4)  
    
        
    classifcation_rate = (1.0 - ( error ./ total_clusters )) .* 100
end