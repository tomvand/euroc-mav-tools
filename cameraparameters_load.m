function [ data ] = cameraparameters_load( data )
%LOAD_POINTCLOUD Load EuRoC cameraParameters data
%
% Create cameraParameters for all cameras in the dataset

for ibody = 1:length(data)
    for isensor = 1:length(data.body{ibody}.sensor)
        if strcmp(data.body{ibody}.sensor{isensor}.sensor_type, 'camera')
            cam = data.body{ibody}.sensor{isensor};
            assert(strcmp(cam.camera_model, 'pinhole') && ...
                strcmp(cam.distortion_model, 'radial-tangential'));
            K = [cam.intrinsics{1}, 0, cam.intrinsics{3};
                 0, cam.intrinsics{2}, cam.intrinsics{4};
                 0, 0, 1]'; % Note: MATLAB's definition is transposed for some reason...
            cp = cameraParameters( ...
                'IntrinsicMatrix', K, ...
                'RadialDistortion', [cam.distortion_coefficients{1:2}], ...
                'TangentialDistortion', [cam.distortion_coefficients{3:4}]);
            data.body{ibody}.sensor{isensor}.cameraParams = cp;
        end
    end
end

end

