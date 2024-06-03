function [tform_FPFH, movingReg_FPFH, rmse_FPFH] = helperCPDFPFH(moving, fixed, tformType)
    arguments
        moving
        fixed
        tformType = 'Nonrigid'
    end

    % ========== params ==========
    FPFH_NUM_NEIGHBOR = 50;
    FPFH_RADIUS = 1;                % 1
    FEAT_MATCH_THRESH = 0.01;       % 0.02
    FEAT_MATCH_REJ_RATIO = 0.5;
    CPD_OUTLIER_RATIO = 0.8;
    % ============================

    movingFeatures = extractFPFHFeatures(moving, 'NumNeighbors', FPFH_NUM_NEIGHBOR, 'Radius', FPFH_RADIUS);
    fixedFeatures = extractFPFHFeatures(fixed, 'NumNeighbors', FPFH_NUM_NEIGHBOR, 'Radius', FPFH_RADIUS);

    [matchingPairs, scores] = pcmatchfeatures(fixedFeatures,movingFeatures, ...
        fixed,moving,Method="Exhaustive", RejectRatio=FEAT_MATCH_REJ_RATIO, MatchThreshold=FEAT_MATCH_THRESH);

    fixed_FPFH = select(fixed, matchingPairs(:,1));
    moving_FPFH = select(moving, matchingPairs(:,2));
    disp("Matched Pair: "+ size(matchingPairs, 1));

       
    [tform_FPFH_interm,movingReg_FPFH_interm,rmse_FPFH_interm] = pcregistercpd(moving_FPFH, fixed_FPFH, ...
                                                                               'Transform', tformType, ...
                                                                               'OutlierRatio', CPD_OUTLIER_RATIO);
    


    if(strcmpi('Rigid', tformType))
        tform_FPFH = tform_FPFH_interm;
    elseif(strcmpi('Affine', tformType))
         tform_FPFH = tform_FPFH_interm;
    else
        moving_pts = moving.Location;
        moving_match_FPFH = zeros(moving.Count,2);
        
        for n = 1:moving.Count
            [indice,dists] = findNearestNeighbors(moving_FPFH,moving_pts(n,:),1);
            moving_match_FPFH(n,1) = n;
            moving_match_FPFH(n,2) = indice;
        end
    
        tform_FPFH = zeros(moving.Count,3);
        for n = 1:moving.Count
            tform_FPFH(n,:) = tform_FPFH_interm(moving_match_FPFH(n,2),:);
        end
    end


    movingReg_FPFH = pctransform(moving, tform_FPFH);
    
    rmse_FPFH = cast(...
            vision.internal.pc.rmse(removeInvalidPoints(movingReg_FPFH), fixed), ...
            'like', movingReg_FPFH.Location);
    
end