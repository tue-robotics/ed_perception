

function export_hypotheses(filename, best_features)

    fid = fopen(filename, 'w');
    
    [r c] = size(best_features);
    for i=1:r
        fprintf(fid, '%d %f %f %f %d %f\n', best_features(i,:));
    end    
    
end %function   