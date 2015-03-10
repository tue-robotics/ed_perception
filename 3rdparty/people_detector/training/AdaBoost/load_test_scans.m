


function [patterns n_positive n_negative n_scans] = load_test_scans( file, positive_label)

    %------------------------------------------
    % load training_set
    %------------------------------------------
    clear scan;
    fprintf('Loading scans...');
    load(file);
    fprintf('DONE\n');

    %------------------------------------------
    % preprocessing
    % final format for one cluster
    % [id class label weight new_type f_1 ... f_n] 
    %------------------------------------------
    positive_label = 1;

    patterns =[];
    pattern = [];
    n_positive = 0;
    n_negative = 0;
    
    fprintf('Relabeling...\n');
    [r n_scans] = size(scan);
    for i=1:n_scans        
        [r n_clusters] = size( scan(i).clusters  );
        for j=1:n_clusters
            class = scan(i).clusters(j).class;
            [r c] = size( class );
            if ( c ~= 0 ) 
                id = scan(i).clusters(j).id;
                if ( class == positive_label )
                    label = 1;
                    n_positive = n_positive + 1;
                else
                    label = -1;
                    n_negative = n_negative + 1;
                end   
                weight = -1;              
                new_type = 0;
                
                features = scan(i).clusters(j).f();
                [ r n_features ] = size(features);                                
                                
                % features = features(1:n_features - 1);  % take out motion feature
                
                features = [features(1:2) features(4:end)];
                
                c_naive_s = scan(i).clusters(j).class_naive_s;
                c_naive_m = scan(i).clusters(j).class_naive_m;
                
                [ r n_features ] = size(features);
                
                % include the scan id, for Kai Arras
                id_scan = i;
                pattern = [  id class label c_naive_s c_naive_m weight id_scan features];
                
                patterns = [patterns; pattern];
            end
        end
       fprintf('loading scan %d/%d\n',i,n_scans);
    end
    fprintf('DONE\n');
end















