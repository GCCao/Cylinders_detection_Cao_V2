clc; clear; close;







dataset = "all";
plotResults = 1;

if strcmp(dataset, "Dragon") || strcmp(dataset, "all")
    disp('Processing dataset "Dragon"')
    XFix = dlmread("../data/dragon1.xyz");
    XMov = dlmread("../data/dragon2.xyz");
    [H, XMovT] = simpleicp(XFix, XMov); %这里采用的是 point-to-plane distance
end

% if strcmp(dataset, "Airborne Lidar") || strcmp(dataset, "all")
%     disp('Processing dataset "Airborne Lidar"')
%     XFix = dlmread("../data/airborne_lidar1.xyz");
%     XMov = dlmread("../data/airborne_lidar2.xyz");
%     [H, XMovT] = simpleicp(XFix, XMov);
% end
% 
% if strcmp(dataset, "Terrestrial Lidar") || strcmp(dataset, "all")
%     disp('Processing dataset "Terrestrial Lidar"')
%     XFix = dlmread("../data/terrestrial_lidar1.xyz");
%     XMov = dlmread("../data/terrestrial_lidar2.xyz");
%     [H, XMovT] = simpleicp(XFix, XMov);
% end
% 
% if strcmp(dataset, "Bunny") || strcmp(dataset, "all")
%     disp('Processing dataset "Bunny"')
%     XFix = dlmread("../data/bunny_part1.xyz");
%     XMov = dlmread("../data/bunny_part2.xyz");
%     [H, XMovT] = simpleicp(XFix, XMov, 'maxOverlapDistance', 1);
% end
% 
% if strcmp(dataset, "Multisensor") || strcmp(dataset, "all")
%     disp('Processing dataset "Multisensor"')
%     XFix = dlmread("../data/multisensor_lidar.xyz");
%     XMov = dlmread("../data/multisensor_radar.xyz");
%     [H, XMovT] = simpleicp(XFix, XMov, 'maxOverlapDistance', 2);
% end

% Plot original and adjusted point clouds with open3d viewer
if plotResults
    %需要用到网友分享的Point cloud tools for Matlab，所以需要加一个路径    
    % addpath(genpath('D:\Work_mix\X学生工作\Z赵妍\论文撰写测试代码\Point_cloud_tools_for_Matlab_master'));

    pcFix = pointCloud(XFix, 'Label', 'XFix');
    pcMov = pointCloud(XMov, 'Label', 'XMov');
    pcMovT = pointCloud(XMovT, 'Label', 'XMovT');
    pcFix.plot('Color', 'r', 'MarkerSize', 5);
    % pcMov.plot('Color', 'g', 'MarkerSize', 5);
    pcMovT.plot('Color', 'b', 'MarkerSize', 5); %变换坐标后的移动点云
end

ee=0;