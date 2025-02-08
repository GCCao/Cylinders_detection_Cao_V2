
function [bestCylinderModel,bestCylinderInliersPoints,bestCylinderInliersIndex,sigma] = Fine_Cylinder_cluster(cylinderParam,AllassociatedPointsOfSeed,Indexs_AllassociatedPointsOfSeed_incurrent,minClustersDistance,minSizeValidCluster,SeedPointIndexs,currentClusterPoints,magnification_sigma)

%找到最可能属于圆柱的点，剔除孤立的离群点，并根据可信点进行最小二乘拟合
%输入：圆柱的初始参数cylinderParam是1*7数组(  cylinderParam(1:3)是轴线上一点，cylinderParam(4:6)是轴线方向， cylinderParam(7)是半径)
%      候选的圆柱点AllassociatedPointsOfSeed是n*3数组、不同聚类中点之间的最小欧拉距离minDistance

%输出：圆柱的最优参数bestCylinderModel、最可信的圆柱点bestCylinderInliersIndex、最可信圆柱点在原始点云中的索引bestCylinderInliersIndex

%基本思路：根据3点法得到了椭圆的参数，有一定的偏差，因此前面基于此找到的圆柱点云，也存在较多误判，
% 比如，有些本来不属于圆柱，但是在圆柱轴线延长方向，有些点距离延长的圆柱表面比较近，也可能被列入候选，
% 因此，必须剔除。剔除之后，再进行最近二乘拟合，就比较可信。


%定义每次参与圆柱拟合的点的数量的上限
maxNumFitPoints = 4000;





%注意：labels是在剩余点云中的标签，indices是在上一级点云中的标签，剩余点云是上一级点云的子集
p = currentClusterPoints(SeedPointIndexs(1),:); %提取出种子点集的第一个点的坐标

% 计算点p与点云P中每个点之间的欧几里得距离 
distances = sqrt(sum((AllassociatedPointsOfSeed - p) .^ 2, 2));

% 找到最小距离值
[minD, minIndex] = min(distances); %minIndex是第一个种子点在AllremainPoints的索引


%%---------------删除离群点，也就是哪些本来不属于圆柱的点
%（1）先采用聚类的方法，删除一部分显著的离群点，换句话说，是哪些与种子点不属于同一个聚类点
AllassociatedPointsOfSeed_pointCloud = pointCloud(AllassociatedPointsOfSeed);
[labels,numClusters] = pcsegdist(AllassociatedPointsOfSeed_pointCloud,minClustersDistance);

% figure
% pcshow(AllassociatedPointsOfSeed_pointCloud.Location,labels);
% colormap(hsv(numClusters));
% title('Point Cloud Clusters')

%如果某点的所属聚类与种子点的聚类，属于同一个标签，则认为该点是合法的聚类点
label_SeedPoint = labels(minIndex);

%合法聚类点在AllassociatedPointsOfSeed中的索引
Indexs_ValidClusterPoints_inAssoc = find(labels == label_SeedPoint);  

%合法聚类点在remainingPointCloud中的索引（z）
Indexs_ValidClusterPoints_incurrent = Indexs_AllassociatedPointsOfSeed_incurrent(Indexs_ValidClusterPoints_inAssoc);

%提取合法聚类点的坐标
ValidClusterPoints = AllassociatedPointsOfSeed(Indexs_ValidClusterPoints_inAssoc,:);

if size(ValidClusterPoints,1)<minSizeValidCluster
    bestCylinderModel = {};
    bestCylinderInliersPoints = [];
    bestCylinderInliersIndex = [];
    sigma =[];
    return;

end



% % （2）删除对候选圆柱体支撑性不足的点，即距离圆柱较远，或者法向与轴线夹角之差与90度相差较大的点
% % To measure the support ofa candidate, we use the number of points that fall within an ε-band around the shape.
% % To ensure that the points inside the band roughly follow the curvature pattern ofthe given primitive, weonly count
% % those points inside the band whose normals do not deviate from the normal of the shape more than a given angle α.
% [ValidCylinderPoints, Indexs_ValidCylinderPoints_inremain] = delete_non_support_points(model,ValidClusterPoints,Indexs_ValidClusterPoints_inremain,cylinderParam);


% ValidCylinderPoints = ValidClusterPoints;
% Index_ValidCylinderPoints_incurrent = Indexs_ValidClusterPoints_incurrent;

%  （2）控制真实参与拟合的点。对参与圆柱拟合点的数量进行控制，保障不能超过阈值maxNumFitPoints，否则实施降采样，确保拟合的效率。
%       降采样之后，得到的圆柱模型之后，利用圆柱模型，重新对原数据进行内点和外点计算，以确保删除过程的有效性
if size(ValidClusterPoints,1)>maxNumFitPoints
    percentage = maxNumFitPoints/size(ValidClusterPoints,1);
    Points_temp = pointCloud(ValidClusterPoints);
    Points_temp = pcdownsample(Points_temp,'random',percentage);

    FitParticipatePoints = Points_temp.Location;    %真实参与拟合的点
else
    FitParticipatePoints = ValidClusterPoints;
end

%(3)对所有合法纯粹圆柱点进行圆柱拟合,再次剔除一部分离群点
% [cylinder_model,Xsigma_inliers,Xsigma_outlier,sigma] = Least_squares_adjustment_of_Points_to_Cylindrical_Surfaces_XIA(cylinderParam,ValidClusterPoints,magnification_sigma);
[cylinder_model,Xsigma_inliers,Xsigma_outlier,sigma] = Least_squares_adjustment_of_Points_to_Cylindrical_Surfaces_NEW(cylinderParam,FitParticipatePoints,magnification_sigma);

if ~isempty(cylinder_model)
    if cylinder_model.Height<0.001
        ee=0;
    end
end

%%---判断点云在平面的投影后形成的圆，圆度是否满足要求
%img= compute_roundness_of_projection_of_pointcloud(P,planePara,cylinder_model)
 % color = rand(1, 3);
 % figure(100),pcshow(ValidClusterPoints,color,'MarkerSize',50);
if ~isempty(cylinder_model)    
    roundness = compute_roundness_of_projection_of_pointcloud(Xsigma_inliers,cylinder_model.Orientation,cylinder_model);
end


if isempty(cylinder_model) || roundness>0.8
    bestCylinderModel = {};
    bestCylinderInliersPoints = [];
    bestCylinderInliersIndex = [];
    return;
end


%-------计算真正的内点。由于圆柱的拟合模型是有FitParticipatePoints产生的，不能代表所有的点贡献，
%       因此需要重新调用ValidClusterPoints参与内点的计算

Index_bestCylinderInliers_inValidClusterPoints = Find_points_Xsigma_frome_cylindersurface(sigma,magnification_sigma,cylinder_model,ValidClusterPoints);


%找到内点在currentClusterPoints中的索引
Index_bestCylinderInliers_incurrent = Indexs_ValidClusterPoints_incurrent(Index_bestCylinderInliers_inValidClusterPoints,:);


%inlier不作为最终的内点，只是作为最小二乘的输出

% figure;
% pcshow(inlier,[0,1,0],'MarkerSize',50); %显示点云数据
% 
% hold on
% pcshow(outlier,[1,0,0],'MarkerSize',50); %显示点云数据
% 
% 
% hold on
% 
% plot(cylinder_model)
% legend('inlier point', 'outlier', 'center line of cylinder', 'cylinder model', 'TextColor', 'w')
% xlabel('x(m)');
% ylabel('y(m)');
% zlabel('z(m)');
% axis equal



bestCylinderModel = cylinder_model
bestCylinderInliersPoints = currentClusterPoints(Index_bestCylinderInliers_incurrent,:);
bestCylinderInliersIndex = Index_bestCylinderInliers_incurrent;
end