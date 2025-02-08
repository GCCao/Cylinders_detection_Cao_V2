
function [bestCylinderModel,bestCylinderInliersPoints,bestCylinderInliersIndex,sigma] = Fine_Cylinder(cylinderParam,AllassociatedPointsOfSeed,Indexs_AllassociatedPointsOfSeed_inremain,minClustersDistance,SeedPointIndexs,remainingPointCloud)
%找到最可能属于圆柱的点，剔除孤立的离群点，并根据可信点进行最小二乘拟合
%输入：圆柱的初始参数cylinderParam是1*7数组(  cylinderParam(1:3)是轴线上一点，cylinderParam(4:6)是轴线方向， cylinderParam(7)是半径)
%      候选的圆柱点AllassociatedPointsOfSeed是n*3数组、不同聚类中点之间的最小欧拉距离minDistance

%输出：圆柱的最优参数bestCylinderModel、最可信的圆柱点bestCylinderInliersIndex、最可信圆柱点在原始点云中的索引bestCylinderInliersIndex

%基本思路：根据3点法得到了椭圆的参数，有一定的偏差，因此前面基于此找到的圆柱点云，也存在较多误判，
% 比如，有些本来不属于圆柱，但是在圆柱轴线延长方向，有些点距离延长的圆柱表面比较近，也可能被列入候选，
% 因此，必须剔除。剔除之后，再进行最近二乘拟合，就比较可信。



%注意：labels是在剩余点云中的标签，indices是在上一级点云中的标签，剩余点云是上一级点云的子集
p = remainingPointCloud(SeedPointIndexs(1),:); %提取出种子点集的第一个点的坐标

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
Indexs_ValidClusterPoints_inremain = Indexs_AllassociatedPointsOfSeed_inremain(Indexs_ValidClusterPoints_inAssoc);

%提取合法聚类点的坐标
ValidClusterPoints = AllassociatedPointsOfSeed(Indexs_ValidClusterPoints_inAssoc,:);


% %（2）再采用点云的法向量与轴线的角偏差作为判断依据，淘汰一部分处于导管曲部分的点
% ValidClusterPoints_pointCloud = pointCloud(ValidClusterPoints);
% % Compute normal property if it is not available
% if isempty(ValidClusterPoints_pointCloud.Normal)
%     % Use 6 neighboring points to estimate a normal vector
%     ValidClusterPoints_pointCloud.Normal = surfaceNormalImpl(ValidClusterPoints_pointCloud, 6);
% end
% 
% 
% Index_valid_CylinderPoints_inValidClusterPoints = int32([]);
% ValidCylinderPoints = [];
% k=1;
% for i=1:ValidClusterPoints_pointCloud.Count
%     N1 = model(4:6);
%     N2 = ValidClusterPoints_pointCloud.Normal(i,:);
% 
%     %计算第i个点法向量与轴线的夹角
%     T_angle = 20;
%     [bool_valid_angle,theta_deg] = angle_between_two_normals(N1,N2,T_angle);
% 
%     %如果夹角合理，就保留改点
%     if bool_valid_angle
%         % Flag_normal(i,1) = 1;
%         ValidCylinderPoints(k,:) = ValidClusterPoints_pointCloud.Location(i,:);
%         Index_valid_CylinderPoints_inValidClusterPoints(k,:) = i;
%         k = k+1;
%     end
% 
%     dd=0;
% end
% 
%  hold on
% pp =  pointCloud(ValidCylinderPoints);
% pcshow(pp.Location,[0,1,0],'MarkerSize',50); %显示点云数据
           
% %（2）将所有的合法聚类点投影到轴线上
% 
% for i=1: size(ValidClusterPoints,1)
%     p = ValidClusterPoints(i,:);    
%     Points_proed(i,:) = ProjectPointOnLine(model(1:3),model(4:6),p);
% end
% 
%  hold on
% pp =  pointCloud(Points_proed);
% pcshow(pp.Location,[0,1,0],'MarkerSize',50); %显示点云数据
           

%(2)删除圆柱点云中含有管路弯曲段的点云，使得圆柱点云更加纯粹
% [ValidCylinderPoints,Index_ValidCylinderPoints_inremain] = delete_Points_in_Curved_Segment(ValidClusterPoints,Indexs_ValidClusterPoints_inremain,cylinderParam);
%ValidCylinderPoints作为最终的内点，这是为了便于删除所有已经检测过的圆柱区域，避免下次迭代重复采样


% % （2）删除对候选圆柱体支撑性不足的点，即距离圆柱较远，或者法向与轴线夹角之差与90度相差较大的点
% % To measure the support ofa candidate, we use the number of points that fall within an ε-band around the shape.
% % To ensure that the points inside the band roughly follow the curvature pattern ofthe given primitive, weonly count
% % those points inside the band whose normals do not deviate from the normal of the shape more than a given angle α.
% [ValidCylinderPoints, Indexs_ValidCylinderPoints_inremain] = delete_non_support_points(model,ValidClusterPoints,Indexs_ValidClusterPoints_inremain,cylinderParam);


ValidCylinderPoints = ValidClusterPoints;
Index_ValidCylinderPoints_inremain = Indexs_ValidClusterPoints_inremain;

%(3)对所有合法纯粹圆柱点进行圆柱拟合,再次剔除一部分离群点
%[cylinder_model,Three_sigma_inliers,Three_sigma_outlier,sigma] = Least_squares_adjustment_of_Points_to_Cylindrical_Surfaces(ValidCylinderPoints);
[cylinder_model,Three_sigma_inliers,Three_sigma_outlier,sigma] = Least_squares_adjustment_of_Points_to_Cylindrical_Surfaces_XIA(cylinderParam,ValidClusterPoints,3);

%找到Three_sigma_inliers中的点在ValidClusterPoints中的索引
% Index_bestCylinderInliers_inValidClusterPoints = Find_points_3sigma_frome_cylindersurface(sigma,cylinder_model,ValidClusterPoints);

if isempty(cylinder_model)
    bestCylinderModel = [];
    bestCylinderInliersPoints = [];
    bestCylinderInliersIndex = [];
    return;
end

%找到Three_sigma_inliers中的点在ValidClusterPoints中的索引
% Index_bestCylinderInliers_inValidClusterPoints = Find_points_2sigma_frome_cylindersurface(sigma,cylinder_model,ValidClusterPoints);
Index_bestCylinderInliers_inValidClusterPoints = Find_points_Xsigma_frome_cylindersurface(sigma,3,cylinder_model,ValidClusterPoints);


%找到one_sigma_inliers中的点在remainingPointCloud中的索引
Index_bestCylinderInliers_inremain = Indexs_ValidClusterPoints_inremain(Index_bestCylinderInliers_inValidClusterPoints,:);


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



bestCylinderModel = cylinder_model;
bestCylinderInliersPoints = remainingPointCloud(Index_bestCylinderInliers_inremain,:);
bestCylinderInliersIndex = Index_bestCylinderInliers_inremain;
end