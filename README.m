% Tools for EuRoC dataset
%
% Goal: produce depth or disparity images suitable for MonoDepth
%
% Usage:
%   pointcloud_load(data, dataset_path): add point cloud to data struct


% worldToImage: project world points on cameraParams, trans, rot!
%   Does not remove depth information, as the image points are in the same
%   order as the world points!

% Check out https://nl.mathworks.com/matlabcentral/fileexchange/55031-pointcloud2image-x-y-z-numr-numc
% for inspiration:
%   - Use knnsearch to find closest point (image coordinates!) to pixel
%       - Find all ties, use min(z) to find closest

% Why interpolate? Need to remove points that lie behind closest object!