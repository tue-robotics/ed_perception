

%
%  Adaboost algorithm in the generalized form. 
%
%  "Semantic Place Classification of Indoor Environments with Mobile Robots
%  using Boosting" Axel Rottmann, Oscar Martinez Mozos, Cyrill Stachniss, Wolfram Burgard.
%
% format of one pattern
%   [id class label weight new_type f_1 ... f_n]


% indexes
% i_class   : original type
% i_label   : (-1, +1)
% i_weight  
% i_new_type: (-1, +1)
% i_shift   : starting index of features

function [best_features] = adaboost( patterns, n_positive, n_negative, n_features, n_iterations, i_class, i_label, i_weight,  i_new_type, i_shift)


	n_patterns = n_positive + n_negative;

    	
	%
	% Inital weights
	%
	fprintf('Initializing weights...');		
	for i=1:n_patterns
		if ( patterns(i, i_label) == 1 )
%             patterns(i, i_weight) = 0.9;
%             patterns(i, i_weight) = 0.9 .* (1 / n_positive);
 			patterns(i, i_weight) = 1 /  (2 .* n_positive);
        else
%             patterns(i, i_weight) = 0.1;
%             patterns(i, i_weight) = 0.1 .* (1 / n_negative);
 			patterns(i, i_weight) = 1 / (2 .* n_negative);
        end	
	end % for i=1:n_clusters	
	fprintf('DONE\n');		
	
    
	%
	% iterate
	%
    fprintf('Boosting...\n');
   	best_features=[];
	for t=1:n_iterations	
		%
		% normalize weigths
		%
		sum_weights = sum( patterns(:, i_weight) );
		patterns(:, i_weight) =  patterns(:, i_weight) ./ sum_weights;
		
		%
		%  For each feature j train a clasifier h_j in the form
		%
        %  [threshold misclassified direction sum_weights]
        %
		trainned_features = zeros(n_features, 4);
		r = zeros(1, n_features);
	
		for j=1:n_features
            arg = patterns(:, [i_label i_weight i_shift+j] );
            trainned_features(j, :) = weak_train( arg );            
			
			%
			%  For each classifier calculate r_j   
			%
			r(j) = 0;
			for i=1:n_patterns
				[type confidence] = weak_classify( trainned_features(j, :), patterns(i, i_shift+j) );
				r(j) = r(j) + patterns(i, i_weight) .* patterns(i, i_label) .* type ;
			end	% i=1:n_patterns						
		end % for j=1:n_features	

		%
		% choose classifier which maximizes |r(j)|
		%
		[ m best_j] = max( abs(r) );
				
		%
		% update weights
		%
        denominator =  1 - r(best_j);
        % just in case it is zero
        denominator = denominator + 0.0000001;
		alpha = 0.5 .* log( (1 + r(best_j) ) ./ denominator );
		
		for i=1:n_patterns
			weight = patterns(i, i_weight);	
			[ new_type confidence ] = weak_classify( trainned_features(best_j, :), patterns(i, i_shift + best_j) ); 
			old_type = patterns(i, i_label);
			new_weight = weight .* exp( (-alpha) .* old_type .* new_type );
            patterns(i, i_weight) = new_weight;
		end 
		
		%
		% store the best
		%
		best_features = [best_features; best_j alpha trainned_features(best_j,:) ];
		
        fprintf('Boosting iteration %d/%d\n',t,n_iterations);
	end % for t=1:n_iterations
	fprintf('DONE\n');
    
end %function


















