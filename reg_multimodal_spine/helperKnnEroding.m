%HELPERKNNENRODING eroding the ptCloud2 by k nearest neighbors within
%threshold
function ptCloudErode = helperKnnEroding(ptCloud1, ptCloud2, threshold, k)
    if nargin<4
        k = 3;
    end
    
    % Create a KDTreeSearcher object for the first point cloud
    searcher = KDTreeSearcher(ptCloud2.Location);

    % For each point in the second point cloud, find the nearest neighbor in the first point cloud
    [~, distances] = knnsearch(searcher, ptCloud1.Location, 'k', k);

    % Find the points in the second point cloud that are within the threshold distance
    toErode = distances < threshold;
    
    ptCloudErode = select(ptCloud1, toErode(:,1));
    for i = 2:k
        ptErode =  select(ptCloud1, toErode(:,i));
        ptCloudErode = pcmerge(ptCloudErode, ptErode, 0.1);
    end

    % Remove these points from the second point cloud
    
end

