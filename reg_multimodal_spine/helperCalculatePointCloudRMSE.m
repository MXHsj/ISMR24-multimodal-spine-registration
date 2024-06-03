function rmse = helperCalculatePointCloudRMSE(ptCloud1, ptCloud2)
    
    % Check if the point clouds have the same number of points
    if size(ptCloud1.Location, 1) ~= size(ptCloud2.Location, 1)
        error('Point clouds must have the same number of points');
    end

    error = ptCloud1.Location - ptCloud2.Location;
    error_mag = sqrt(error(:,1).^2+error(:,2).^2+error(:,3).^2);
    rmse = rms(error_mag);

end

