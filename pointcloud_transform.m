function [ pc_S ] = pointcloud_transform( pc_R, T_SR, varargin )
%POINTCLOUD_REPROJECT Transform point cloud from frame R to frame S

p_R = pc_R.Location';
p_R(4,:) = 1; % Make homogenous

p_S = T_SR * p_R;

pc_S = pointCloud(p_S(1:3,:)');

end

