% ========================================================================
% file name:    cloud2cloud.m
% desciption:   closed form pointcloud registration knowning point correspondence
% author:       Xihan Ma
% date:         2023-11-20
% ========================================================================

%%% CLOUDTOCLOUD 
% computes rigid transformation between 2 pt clouds
%
%   params - 3xn arrays of n points
%
%   returns - 4x4 homogeneous transformation matrix from B to A (B WRT A)
%
%   returns - RMS fiducial registration error (FRE)
%
%           - the rms distance between corresponding fiducial pts after
%           registration
%
% GSF 4/19/2021

function [transform, RMS_FRE] = cloud2cloud(cloudA, cloudB)

numEl = size(cloudA,2);

% find centroid (A_bar) of A
centroidA = sum(cloudA,2)./numEl;

% find centroid (B_bar) of B
centroidB = sum(cloudB,2)./numEl;

% Solve for R Using Arun Method
H = zeros(3,3); % create a coefficient matrix
for i=1:numEl
    % Solve for points wrt to centroids
    ATilde = cloudA(:,i)-centroidA;
    BTilde = cloudB(:,i)-centroidB;
  
    % Fill in H matrix
    for j=1:3	
		for k=1:3
			H(j,k)=H(j,k)+ATilde(j)*BTilde(k);
		end
    end
end

% SVD of H = USV^t
[U,~,V] = svd(H);

% R = VU^t
R = V*U';

% Verify Det(R) = 1. If not, algor. may fail
Det = det(R);
thresh = 0.001; % set threshold to avoid numerical precision throwing error
if ( (Det>(1+thresh)) || (Det<(1-thresh)) )
    disp('WARNING: Determinant of R is not equal to 1. Flipping the sign for last column');
    R(:,end) = -R(:,end);
end

% p = b - R*a
p = centroidB - R*centroidA;

% incorporate R and p into homogenous transformation
transform = eye(4,4);
transform(1:3,1:3) = R;
transform(1:3,4) = p;

% Produce the RMS errors 
% The rms distance bewteen the corresponding fiducial points after registration

RMSE_Sum = 0;

for i=1:numEl
    
    % Apply transform to points in cloud A
    TransformedCloudAPoint = R*cloudA(:,i)+p;
    
    % Solve for error between transfromed Cloud A and Cloud B
    error = TransformedCloudAPoint - cloudB(:,i);
    RMSE_Sum = RMSE_Sum + sqrt(mean(error.^2));
    
end
RMS_FRE = RMSE_Sum/numEl;

end
