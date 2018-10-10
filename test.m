% Script for random tests/development

cfg = euroc_mav_tools_config;
addpath(genpath(cfg.tools_path));

dataset_path = fullfile(cfg.data_path, ...
    'vicon_room1/V1_01_easy/V1_01_easy/');
%     'vicon_room2/V2_01_easy/V2_01_easy/');
%     'vicon_room1/V1_02_medium/V1_02_medium/');


fprintf(' >> loading dataset ''%s''...\n', dataset_path);
data = dataset_load(dataset_path);

fprintf(' >> loading cameraParameters... ');
data = cameraparameters_load(data);
fprintf('ok\n');

fprintf(' >> loading pointcloud... ');
data = pointcloud_load(data, dataset_path);
fprintf('ok\n');

fprintf(' >> convert visual-inertial to T_RS... ');
for ibody = 1:length(data)
    for isensor = 1:length(data.body{ibody}.sensor)
        if strcmp(data.body{ibody}.sensor{isensor}.sensor_type, 'visual-inertial')
            data.body{ibody}.sensor{isensor}.data.C_RS = ...
                all_q2C(data.body{ibody}.sensor{isensor}.data.q_RS);
            data.body{ibody}.sensor{isensor}.data.T_RS = ...
                all_pC2T(data.body{ibody}.sensor{isensor}.data.p_RS_R, ...
                data.body{ibody}.sensor{isensor}.data.C_RS);
        end
    end
end
fprintf('ok\n');

fprintf(' >> find T_BR for visual-inertial... ');
for ibody = 1:length(data)
    for isensor = 1:length(data.body{ibody}.sensor)
        if strcmp(data.body{ibody}.sensor{isensor}.sensor_type, 'visual-inertial')
            N = size(data.body{ibody}.sensor{isensor}.data.T_RS, 3);
            data.body{ibody}.sensor{isensor}.data.T_BR = NaN(4,4,N);
            for i=1:N
                T_SR = T_inv(data.body{ibody}.sensor{isensor}.data.T_RS(:,:,i));
                T_BR = data.body{ibody}.sensor{isensor}.T_BS * T_SR;
                data.body{ibody}.sensor{isensor}.data.T_BR(:,:,i) = T_BR;
            end
            T_BR = data.body{ibody}.sensor{isensor}.data.T_BR; % Keep for later processing...
            t = data.body{ibody}.sensor{isensor}.data.t;
        end
    end
end
fprintf('ok\n');

% fprintf(' >> convert pose to T_RS... ');
% for ibody = 1:length(data)
%     for isensor = 1:length(data.body{ibody}.sensor)
%         if strcmp(data.body{ibody}.sensor{isensor}.sensor_type, 'pose')
%             data.body{ibody}.sensor{isensor}.data.C_RS = ...
%                 all_q2C(data.body{ibody}.sensor{isensor}.data.q_RS);
%             data.body{ibody}.sensor{isensor}.data.T_RS = ...
%                 all_pC2T(data.body{ibody}.sensor{isensor}.data.p_RS_R, ...
%                 data.body{ibody}.sensor{isensor}.data.C_RS);
%         end
%     end
% end
% fprintf('ok\n');
% 
% fprintf(' >> find T_BR for pose... ');
% for ibody = 1:length(data)
%     for isensor = 1:length(data.body{ibody}.sensor)
%         if strcmp(data.body{ibody}.sensor{isensor}.sensor_type, 'pose')
%             N = size(data.body{ibody}.sensor{isensor}.data.T_RS, 3);
%             data.body{ibody}.sensor{isensor}.data.T_BR = NaN(4,4,N);
%             for i=1:N
%                 T_SR = T_inv(data.body{ibody}.sensor{isensor}.data.T_RS(:,:,i));
%                 T_BR = data.body{ibody}.sensor{isensor}.T_BS * T_SR;
%                 data.body{ibody}.sensor{isensor}.data.T_BR(:,:,i) = T_BR;
%             end
%             T_BR = data.body{ibody}.sensor{isensor}.data.T_BR; % Keep for later processing...
%             t = data.body{ibody}.sensor{isensor}.data.t;
%         end
%     end
% end
% fprintf('ok\n');

fprintf(' >> find T_SR for cameras... ');
for ibody = 1:length(data)
    for isensor = 1:length(data.body{ibody}.sensor)
        if strcmp(data.body{ibody}.sensor{isensor}.sensor_type, 'camera')
            fprintf('%s... ', data.body{ibody}.sensor{isensor}.name);
            T_SB = T_inv(data.body{ibody}.sensor{isensor}.T_BS);
            N = size(T_BR, 3); % From previous block
            data.body{ibody}.sensor{isensor}.data.highrate.T_SR = NaN(4,4,N);
            data.body{ibody}.sensor{isensor}.data.highrate.t = t;                
            for i=1:N
                data.body{ibody}.sensor{isensor}.data.highrate.T_SR(:,:,i) = T_SB * T_BR(:,:,i);
            end
        end
    end
end
fprintf('ok\n');

fprintf(' >> Downsample T_SR to camera rate... ');
for ibody = 1:length(data)
    for isensor = 1:length(data.body{ibody}.sensor)
        if strcmp(data.body{ibody}.sensor{isensor}.sensor_type, 'camera')
            index = interp1(double(data.body{ibody}.sensor{isensor}.data.highrate.t), ...
                1:length(data.body{ibody}.sensor{isensor}.data.highrate.t), ...
                double(data.body{ibody}.sensor{isensor}.data.t), 'nearest', 'extrap');
            data.body{ibody}.sensor{isensor}.data.T_SR = ...
                data.body{ibody}.sensor{isensor}.data.highrate.T_SR(:,:,index);
        end
    end
end
fprintf('ok\n');

ti = 1;
% ti = 1430;
% ti = 2000;
fprintf(' >> TEST: transform point cloud to camera frame... ');
pc_cam = pointcloud_transform(data.body{1}.sensor{4}.data, data.body{1}.sensor{1}.data.T_SR(:,:,ti));
p = pc_cam.Location;
keep = p(:,3) > 0;
pc_cam = pointCloud(p(keep,:));
figure;
pcshow(pc_cam);
xlabel('x');
ylabel('y');
zlabel('z');
fprintf('ok\n');

fprintf(' >> TEST: project point cloud to depth map... ');
cam = data.body{1}.sensor{1};
img = NaN(cam.resolution{2}, cam.resolution{1});
imgpts = worldToImage(data.body{1}.sensor{1}.cameraParams, eye(3), zeros(3,1), pc_cam.Location, 'ApplyDistortion', true);
for i=1:size(imgpts,1)
    c = round(imgpts(i,1)) + 1;
    r = round(imgpts(i,2)) + 1;
    if c >= 1 && c < size(img, 2) && r >= 1 && r < size(img, 1)
        z = pc_cam.Location(i,3);
        if isnan(img(r,c)) || z < img(r,c)
            img(r,c) = z;
        end
    end
end
% Depth map
figure;
imagesc(img);
colorbar;
axis equal;
fprintf('ok\n');

fprintf(' >> TEST: get camera image... ');
img_gray = imread(fullfile(dataset_path, ...
    data.body{1}.name, ...
    data.body{1}.sensor{1}.name, ...
    'data', ...
    data.body{1}.sensor{1}.data.filenames{ti}));
figure;
imshow(img_gray);
fprintf('ok\n');

% fprintf(' >> TEST: show ground-truth trajectory... ');
% figure;
% pcshow(data.body{1}.sensor{4}.data); hold all;
% dataset_plot_body_trajectory(data.body{1});
% xlim([-5 5]);
% zlim([-1 2]);
% fprintf('ok\n');