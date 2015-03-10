


function [ type confidence] = weak_classify( weak_classifier, value)
	
	threshold = weak_classifier(1);
	misclassified = weak_classifier(2);
	direction = weak_classifier(3);
	sum_weights = weak_classifier(4);
	

	if ( direction==1 ) 
		% positives are >= than threshold
		if ( value >= threshold ) 
			type = 1;  % positive		
		else 
			type = -1;
		end
	elseif ( direction==2 ) 
		% positives are < than threshold
		if ( value < threshold ) 
			type = 1; % positive
		else 
			type = -1; % negative
		end
	end

	confidence = 1.0 - (misclassified / sum_weights);

	if ( type == -1 ) 
		% negative
		confidence = (-confidence);
    end


end % function




























