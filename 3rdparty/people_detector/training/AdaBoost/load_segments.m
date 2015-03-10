


function [patterns n_positive n_negative] = load_segments( file, positive_label)

    %------------------------------------------
    % load training_set
    %------------------------------------------
    fprintf('Loading segments...');
    
    patterns=[];
    
    fid = fopen(file, 'r');
    n_positive = 0;
    n_negative = 0;
    
    segment_number=0;
    while 1
        param = fscanf(fid, '%d %f %d %d',[4 1]);
        if isempty(param), break, end
        
        segment_number = segment_number + 1
        scan_id = param(1);
        timestamp = param(2);   
        type = param(3);
        n_beams = param(4);    
        
        if (type == positive_label)
            n_positive = n_positive + 1;
            label = 1;
        else
            n_negative = n_negative + 1;
            label = -1;
        end         
        
        segment_beams = fscanf(fid, ' %g %g', [2, n_beams]);
        
        n_features = fscanf (fid, ' %d', 1);
        segment_features = fscanf(fid, ' %g', n_features);
        % [ type(original type) label(+1,-1) weight classification(-1,+1)
        % f_1 ... f_n]
        weight = -1;
        classification = 0;
        [patterns] = [patterns;  type label weight classification segment_features'];
    end

    fprintf('DONE\n');

end















