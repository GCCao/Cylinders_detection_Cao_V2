clear all
close all

% Testing for singular values during cylindrical fitting
%测试圆柱拟合出现奇异值的情况

ptCloudMov = pcread("Tube3_scaned.ply"); %去读导管的三维扫描点云数据，整体导管
% ptCloudMov = pcread("Tube2_scaned_local2_Cloud.ply"); %去读导管的三维扫描点云数据，整体导管

% pppp = ptCloudMov.Location ;
% save('ptCloudMov.mat', 'pppp');

percentage = 0.1;
ptCloudMov = pcdownsample(ptCloudMov,'random',percentage)  %下采样到原来的1%
pcshow(ptCloudMov.Location,[1 0 0],'MarkerSize',50); %显示点云数据
  
load("cylinderParam.mat");
load("point.mat");
hold
pcshow(point,[0 1 0],'MarkerSize',50); %显示点云数据

[cylinder_model,inlier,outlier,sigma] = Least_squares_adjustment_of_Points_to_Cylindrical_Surfaces_XIA(cylinderParam,point);

ee=0;
