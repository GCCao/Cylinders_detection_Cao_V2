

clear all 
close all

ptCloudMov = pcread("Tube1_scaned.ply"); %去读导管的三维扫描点云数据，整体导管
% % %--------先定义参数-----------------
maxDistance = 0.05;
K = 4;
percentage_Sampling = 0.1; %分配采样率，0.1表示对原始点云进行10%的采样
Max_iteration = 100000000;

Points_temp = pcdownsample(ptCloudMov,'random',percentage_Sampling);

% Compute normal property if it is not available
if isempty(Points_temp.Normal)
    % Use 6 neighboring points to estimate a normal vector
    Points_temp.Normal = surfaceNormalImpl(Points_temp, 6);
end

bestPlaneModels = {};%用于存储当前分辨率点云检测平面


remainingPointCloud = Points_temp.Location;
remainingPointCloudNormal = ptCloudMov.Normal;

figure(1)
pcshow(remainingPointCloud,[0.5,0.5,0.5],'MarkerSize',50); %显示点云数据


%（1） 获取remainingPointCloud点云中的点的数量
numPoints = size(remainingPointCloud, 1);

iteration_count = 0;
while (iteration_count < Max_iteration)
    %（2） 在currentClusterPoints中随机选择K个不同的索引，用作采样
    SeedPointIndexs = randperm(numPoints, K);

    %（3） 根据索引获取采样点和法向量
    subpoints = remainingPointCloud(SeedPointIndexs, :);
    subnormal = remainingPointCloudNormal(SeedPointIndexs, :);


   [isCoplanar, planeParams] = checkCoplanarity(subpoints, subnormal);
    if isCoplanar>0
        indicesInPlane = findPointsInPlane(planeParams, remainingPointCloud);
        Points_InPlane = remainingPointCloud(indicesInPlane,:);
        hold on
        pcshow(Points_InPlane,[1,0.5,0.5],'MarkerSize',50); %显示点云数据
        ee=0;

    end
    iteration_count = iteration_count+1;
end
    ee=0;