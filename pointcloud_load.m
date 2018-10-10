function [ data ] = pointcloud_load( data, dataset_path )
%LOAD_POINTCLOUD Load EuRoC pointcloud data
%
% Load pointcloud data from the EuRoC MAV dataset and append it to the
% data struct that is returned by dataset_load().
%
% data: current dataset struct
% dataset_path: path to the dataset folder (parent of the 'mav0' folder)

for ibody = 1:length(data)
    for isensor = 1:length(data.body{ibody}.sensor)
        if strcmp(data.body{ibody}.sensor{isensor}.sensor_type, 'pointcloud')
            % Found point cloud data, load
            pointcloud_path = fullfile(dataset_path, ...
                data.body{ibody}.name, ...
                data.body{ibody}.sensor{isensor}.name, ...
                'data.ply');
            data.body{ibody}.sensor{isensor}.data = pcread(pointcloud_path);
        end
    end
end

end

