
%
%  train a classifier which is just a threshold
%

%
% params: 
%    pattern(:,3) = [label( 1 = pos, -1 = neg),  weight, feature_value]
%

function res = weak_train( patterns )
	
	sum_weights = sum( patterns(:, 2) );

    % leave out NaN values
    %patterns( find( isnan(patterns(:,3)) ),: ) =[];
   
    % normalize weights with non-NaN values
    %sum_weights = sum( patterns(:, 2) );
	%patterns(:, 2) =  patterns(:, 2) ./ sum_weights;    
    
	patterns = sortrows( patterns, 3);
    
	
	% number of postive patterns
	[n_patterns c] = size( patterns );
	[r n_pos ]= size ( find( patterns(:,1) == 1 )' );
	n_neg = n_patterns - n_pos;
	
	% Look for a threshold that best classify the examples.
	% Two sets are maintained:
	% 	left: elements to the left of the threshold
	% 	right: elements to the right of the threshold
	% pos_left, neg_left;
	% pos_right, neg_right;

	% values to the left and right of threshold. 1 value = weight
	% double pos_left_value, neg_left_value;
	% double pos_right_value, neg_right_value;

	% at the begining left is empty
	pos_left=0;
	neg_left=0;
	pos_right = n_pos;
	neg_right = n_neg;
	% values == weights
	pos_left_value=0.0;
	neg_left_value=0.0;

	% sum of weights of positive examples
	pos_right_value = sum ( patterns(find(patterns(:,1) == 1), 2 ));
	
	% sum of weights of negative examples
	neg_right_value = sum ( patterns(find(patterns(:,1) == -1), 2 ));
	
	min_pos = -1;  % position with minimum misclassification
	min_mis = pos_right_value + neg_right_value;  % minimum misclasification
	mis=-1;
	direction =-1;

	
	for i=1:n_patterns
		% two cases
		% 1: consider values >= than position as positive		
		mis = neg_right_value + pos_left_value;
		if ( mis < min_mis ) 
			min_mis = mis;
			direction = 1;
			min_pos = i;
        end
		
		% 2: consider values < than position as positive
		mis = neg_left_value + pos_right_value;
		if ( mis < min_mis ) 
			min_mis = mis;
			direction = 2;
			min_pos = i;
        end
	
		% advance pos
		if ( patterns(i, 1) == 1 ) 
			pos_right_value = pos_right_value - patterns(i, 2);
			pos_left_value = pos_left_value + patterns(i, 2);
		else 
			neg_right_value = neg_right_value - patterns(i, 2);
			neg_left_value = neg_left_value + patterns(i, 2);
        end
	
	end % for i=1:n_patterns
	
	
	misclassified = min_mis;
	if ( min_pos == 1 ) 
		threshold = patterns(min_pos, 3);	
    elseif ( min_pos == n_patterns ) 
		threshold = patterns(min_pos, 3);
    else 
		threshold = ( patterns(min_pos-1, 3) + patterns(min_pos, 3) ) ./ 2.0;
    end

    res = [threshold misclassified direction sum_weights];

end % end function






















