% Remove the nearest points of ptCloud1 from ptCloud2
function ptCloud2Filtered = helperRemoveNearestPoints(ptCloud1, ptCloud2, threshold)
    % Create a KDTreeSearcher object for the first point cloud
    searcher = KDTreeSearcher(ptCloud1.Location);

    % For each point in the second point cloud, find the nearest neighbor in the first point cloud
    [~, distances] = knnsearch(searcher, ptCloud2.Location);

    % Find the points in the second point cloud that are within the threshold distance
    toRemove = distances < threshold;

    % Remove these points from the second point cloud
    ptCloud2Filtered = select(ptCloud2, ~toRemove);
end

