


function  [class_type confidence] = adaboost_classify( weak_classifiers, n_weak,  pattern )
         
    % indexes
    sum = 0;
    for i=1:n_weak        
        i_feature = weak_classifiers(i, 1);
        [type conf] = weak_classify( weak_classifiers(i, 3:6), pattern(i_feature) );
        alpha = weak_classifiers(i, 2);
        sum = sum + alpha .* type;
    end  
    
    class_type = sign(sum);
    
    % confidence
    if ( class_type > 0 )
        confidence = exp( sum ) ./ ( exp(-sum) + exp (sum) );
    else        
        confidence = exp( -sum ) ./ ( exp(-sum) + exp (sum) );
    end
    
    % return


end % function