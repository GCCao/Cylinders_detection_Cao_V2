 clc; clear all; close all;


 addpath('../adjustment_Cylindrical_Surfaces');
 addpath('../PipelinesData_Cao');

 %--------读取数据，并降采样---------------------
 % Load point cloud data into the workspace.
 % ptCloudMov = pcread("pipelines1_pointcloud_grab.ply"); 
 ptCloudMov = pcread("Tube1_scaned.ply"); 

 
  

%------------------------------采用多次采样的方式，检测不同分辨率下的点云中圆柱体
fzoom = ptCloudMov.Count/100000;
if fzoom>1

    Sampling_Rate = [1 1 1]/fzoom; %保障采样后能维持在10万个点
else
    Sampling_Rate = [1 1 1];
end

percentage = Sampling_Rate(1);
Points_temp = pcdownsample(ptCloudMov,'random',percentage);

%尺度化点云，必须极小的圆柱出现
%对于cloudcompare转换成的点云，总是容易被随机尺度化，导致尺寸倍缩放的很小
P_out = scalePointCloud(Points_temp.Location);
ptCloudMov = pointCloud(P_out);
if isempty(ptCloudMov.Normal)
    % Use 6 neighboring points to estimate a normal vector
    ptCloudMov.Normal = surfaceNormalImpl(ptCloudMov, 6);
end
ptCloudMov_Init = ptCloudMov;


%----------------------------------------先进行平面检测，排除干扰
params.distThreshold = 0.3;    % 距离阈值
params.minPoints = 200;         % 最小点数
params.maxPlanes = 6;          % 最大平面数
params.confidence = 0.99;       % RANSAC 置信度
params.maxIterations = 1000;    % RANSAC 最大迭代次数
params.angleThreshold = 5;     % 法向量夹角阈值（度）
params.invalidRatioThreshold = 0.6; % 无效点比例阈值
params.maxIteration = 1000;    % 主循环最大迭代次数


tic
[planes, remainingPoints] = detectMultiplePlanesWithNormals(ptCloudMov.Location, ptCloudMov.Normal,params);
toc
Thstd = 0.03;
[validplanes,remainingPoints] = checkPlanesValidity(planes, remainingPoints, Thstd);

ptCloudMov = pointCloud(remainingPoints);


%%%%%%---------------------------------  平面检测结束





%------------------------------采用多次采样的方式，检测不同分辨率下的点云中圆柱体
fzoom = ptCloudMov.Count/300000;
if fzoom>1

    Sampling_Rate = [1 1 1]/fzoom; %保障采样后能维持在10万个点
else
    Sampling_Rate = [1 1 1];
end




% % %--------先定义参数-----------------
neighboringSize = 10;
maxDistance = 0.05;
minR = 2;   %最小圆柱半径
maxR = 500; %最大圆柱半径
minClustersDistance = 20; %聚类时用于分割点云族的阈值，这个值与扫描点云的密度有关系。通常扫描点云密度为0.2mm，原则上，只要minDistance>0.2即可，
% 但考虑下采样需求，比如下采样率0.1，同时扫描点云密度为0.2mm，则minClustersDistance>(0.2/0.1)
% 为了保险，通常可以设置的更大一些，minClustersDistance  = 10*(点云密度/采样率);
minSizeValidCluster = 100;%对于数量小于minSizeValidCluster的点云族，认为不可靠，不再参与后续的计算，删除掉

DesiredNumCylinders = 100; %期望从点云中检测到的最多圆柱体数量

ExistingNumCylinders = 1000000; %最近一次检测之前的圆柱数量，初值为一个很大值，主要是为了while判断

% 采用2sigma进行圆柱检测（主要差异在于圆柱拟合的sigma值）
magnification_sigma = 3;

%定义同名圆柱体之间的角度差异，以及Ci中心点到Cj轴线之间的距离，设置阈值
angleThreshold = 5; %不超angleThreshold度
distanceThreshold = 0.10; %距离之差不要超过半径的百分比

%投影弧长占比的阈值，低于这个阈值，认为这部分点云大概率不属于圆柱
arc_length_ratio_proj = 0.25;



% %圆柱体误差的特征矢量
% FeatureVectors =[];


%多尺度采样后检测到的圆柱体
detectedCylinders_MulitiSampling = {};


tic
for i = 1:size(Sampling_Rate,2)

    k=1;

    Sampling_Order = i; %主要用于画图时，figure知道在哪个图中画

    %单一尺寸采样点云检测到的圆柱数量
    detectedCylinders = {};
    detectedCylinders_pointCloud = {};

    times_num_detectedCylinders_remains_unchanged = 0;%用于记录被检测到圆柱体数量保持不变的次数

    percentage = Sampling_Rate(i);
    Points_temp = pcdownsample(ptCloudMov,'random',percentage);

    % Compute normal property if it is not available
    if isempty(Points_temp.Normal)
        % Use 6 neighboring points to estimate a normal vector
        Points_temp.Normal = surfaceNormalImpl(Points_temp, 6);
    end

    bestCylinderModels = {};%用于存储当前分辨率点云检测到圆柱体   
    


    remainingPointCloud = Points_temp.Location;
    % remainingPointCloudNormal = ptCloudMov.Normal;
    G = 0;




    while (size(remainingPointCloud,1)<200 || size(detectedCylinders,2)>=DesiredNumCylinders ||  times_num_detectedCylinders_remains_unchanged>=3)~= 1
        %终止条件： 剩余点的数量小于某个值      或者   检测到的圆柱数量大于等于期望数量              或者        不再有新的圆柱体增加

        ExistingNumCylinders = size(detectedCylinders,2);

        %(1)  把点云进行聚类，并删除尺寸小的族。对于数量小于minSizeValidCluster的族，认为不可靠，不再参与后续的计算，当删除，同时更新remainingPointCloud
        [clusterLabels, numClusters, remainingPointCloud] = filterAndRelabelClusters(remainingPointCloud,minClustersDistance,minSizeValidCluster);


        %(2)  依次对每个点族进行圆柱拟合，输出检测到圆柱，以及剩余的点云remainingPointCloud
        % [detectedCylinders,detectedCylinders_pointCloud,remainingPointCloud] = DetectCylindersBasedonMultiClusters(clusterLabels,detectedCylinders,detectedCylinders_pointCloud,numClusters, remainingPointCloud,minClustersDistance,minSizeValidCluster,magnification_sigma,Sampling_Order,neighboringSize);
        [Cylinders,Cylinders_pointCloud,remainingPointCloud] = DetectCylindersBasedonMultiClusters_new(clusterLabels,numClusters, remainingPointCloud,minClustersDistance,minSizeValidCluster,magnification_sigma,Sampling_Order,neighboringSize,arc_length_ratio_proj);

        %(3)  净化点云，更新圆柱的参数。有一些处于弯曲部分的点，但是满足距离和法向约束要求，被误以为是圆柱体上的点，应当被删除
        %                             这类点的特点是，在圆弧上的占比比较小，通常少于1/5
        [Cyl,Cyl_pointCloud,indices_in_Cylinders_pointCloud] = Upgrade_cylindricalmodel_and_pointcloud_by_removing_small_arclength_ratio(Cylinders,Cylinders_pointCloud,arc_length_ratio_proj);


        for jj=1:size(Cyl,2)
            detectedCylinders{k} = Cyl{jj};
            detectedCylinders_pointCloud{k} = Cyl_pointCloud{jj};
            k=k+1;
        end

        if size(detectedCylinders,2) == ExistingNumCylinders
            times_num_detectedCylinders_remains_unchanged = times_num_detectedCylinders_remains_unchanged +1;
        end


        % figure(2)
        % pcshow(remainingPointCloud,[0 1 0],'MarkerSize',50); %显示点云数据

        ee=0;
    end
    detectedCylinders_MulitiSampling{i} = detectedCylinders;
    detectedCylinders_pointCloud_MulitiSampling{i} = detectedCylinders_pointCloud;
    

end
toc


%在圆柱中心点绘制箭头和标号
for k=1:size(detectedCylinders_MulitiSampling,2)
    detectedCylinders = detectedCylinders_MulitiSampling{k};

    % %保存数据
    % save('ptCloudMov_Init.mat', 'ptCloudMov_Init');   
    % FinalCylinders = detectedCylinders;
    % save('FinalCylinders.mat', 'FinalCylinders');


    Sampling_Order = k; %主要用于画图时，figure知道在哪个图中画
    for i=1:size(detectedCylinders,2)
        bestCylinderModel = detectedCylinders{i};
        Draw_arrows_and_labels_at_the_centerpoint_of_the_cylinder(bestCylinderModel,i,Sampling_Order);
    end
end




%基于多尺度采样证实圆柱体真假,原理上，物理意义上存在的圆柱体，在不同采样分辨率下，都能稳定存在，而弯曲等部分的圆柱体比较随机
if  size(detectedCylinders_MulitiSampling,2)>2
    [VerifiedCylinders_Multi_scale_Samp,VerifiedCylinders_pointCloud_Multi_scale_Samp] =  Verify_Cylinders_Based_on_Multi_scale_Samp_new(detectedCylinders_MulitiSampling,detectedCylinders_pointCloud_MulitiSampling,angleThreshold,distanceThreshold);
else
    VerifiedCylinders_Multi_scale_Samp = detectedCylinders;
    VerifiedCylinders_pointCloud_Multi_scale_Samp = detectedCylinders_pointCloud;
end


%在圆柱中心点绘制箭头和标号
figureNum =1;
for i=1:size(VerifiedCylinders_Multi_scale_Samp,2)
    bestCylinderModel = VerifiedCylinders_Multi_scale_Samp{i};    
    Draw_arrows_VerifiedCylinders_Multi_scale_Samp(bestCylinderModel,i,figureNum);
end




%%%%圆柱合并。根据圆柱的嵌套、直径包含关系，剔除一部分疑似干扰的圆柱
Angular_Difference_Thre = 3;%单位
% MergedCylinders = CylindersMerging(VerifiedCylinders_Multi_scale_Samp,Angular_Difference_Thre);
[MergedCylinders,MergedCylinders_pointCloud] = CylindersMerging(VerifiedCylinders_Multi_scale_Samp,VerifiedCylinders_pointCloud_Multi_scale_Samp,Angular_Difference_Thre);


%删除同时满足这两种条件的圆柱：（1）直径与高度之比小于1，（2）圆柱的轴线角度与其最近邻域圆柱的轴线不垂直也不平行
[FinalCylinders, FinalCylinders_pointCloud] = Delete_short_cylinders(MergedCylinders,MergedCylinders_pointCloud);

%绘制合并后的圆柱的彩色点云
figureNum =4;
figure(figureNum)
pcshow(ptCloudMov_Init.Location,[0.5,0.5,0.5],'MarkerSize',10); %显示点云数据
for i=1:size(FinalCylinders_pointCloud,2)
    P = FinalCylinders_pointCloud{i};
    hold on
    color = rand(1, 3);    
    pcshow(P,color,'MarkerSize',50);
    bestCylinderModel = FinalCylinders{i};
    Draw_arrows_VerifiedCylinders_Multi_scale_Samp(bestCylinderModel,i,figureNum);
    set(gca, 'view', [20, 60]);
end


e=0; 