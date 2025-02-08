function expandedPureCylinderInliersIndex =  Delete_the_visited_pointcloud_by_expanded_cylinder_cluster(bestCylinderModel,currentClusterPoints,magnification_sigma,sigma,SeedPointIndexs,minClustersDistance,arc_length_ratio_proj)
%删除对后续圆柱检测有干扰的点，为了删除的比较彻底，将最小二乘检测到的圆柱进行膨胀，即把半径增加3倍sigma
%凡是在膨胀圆柱体内部的点，都将被删除，也就是不再参与后续圆柱体检测

model = [bestCylinderModel.Center bestCylinderModel.Orientation/norm(bestCylinderModel.Orientation) bestCylinderModel.Radius];



%--------------------
%注意：labels是在剩余点云中的标签，indices是在上一级点云中的标签，剩余点云是上一级点云的子集
p = currentClusterPoints(SeedPointIndexs(1),:); %提取出种子点集的第一个点的坐标

% 计算点p与点云P中每个点之间的欧几里得距离
distances = sqrt(sum((currentClusterPoints - p) .^ 2, 2));

% 找到最小距离值
[minD, minIndex] = min(distances); %minIndex是第一个种子点在AllremainPoints的索引


%%---------------删除离群点，也就是哪些本来不属于圆柱的点
%（1）先采用聚类的方法，删除一部分显著的离群点，换句话说，是哪些与种子点不属于同一个聚类点
currentClusterPoints_pointCloud = pointCloud(currentClusterPoints);
[labels,numClusters] = pcsegdist(currentClusterPoints_pointCloud,minClustersDistance);

% figure
% pcshow(AllassociatedPointsOfSeed_pointCloud.Location,labels);
% colormap(hsv(numClusters));
% title('Point Cloud Clusters')

%如果某点的所属聚类与种子点的聚类，属于同一个标签，则认为该点是合法的聚类点
label_SeedPoint = labels(minIndex);

% %合法聚类点在AllassociatedPointsOfSeed中的索引
% Indexs_ValidClusterPoints_inAssoc = find(labels == label_SeedPoint);  

% %合法聚类点在remainingPointCloud中的索引（z）
% Indexs_ValidClusterPoints_inremain = Indexs_AllassociatedPointsOfSeed_inremain(Indexs_ValidClusterPoints_inAssoc);

% %提取合法聚类点的坐标
% ValidClusterPoints = points(Indexs_ValidClusterPoints_inAssoc,:);

% 
% points = ValidClusterPoints;

%-----计算点到圆柱轴线的距离
% Calculate the distance from the point P0 to the axis P2-P1 of the Cylinder
% D = ||(P2-P1) x (P1-P0)|| / ||P2-P1||
p1p0 = [currentClusterPoints(:,1)-model(1), currentClusterPoints(:,2)-model(2), currentClusterPoints(:,3)-model(3)];
p2p1 = model(4:6);
c = [p1p0(:,2)*p2p1(3) - p1p0(:,3)*p2p1(2), ...
    p1p0(:,3)*p2p1(1) - p1p0(:,1)*p2p1(3), ...
    p1p0(:,1)*p2p1(2) - p1p0(:,2)*p2p1(1)];
% p2p1 is a unit vector, so the denominator is not needed
D = sum(c.*c, 2);
D = sqrt(D);


%圆柱体的首尾点
startp = bestCylinderModel.Parameters(1:3);
endp = bestCylinderModel.Parameters(4:6);


%----计算点到轴线的投影
p_result = [];
% tic
% for i=1:size(currentClusterPoints,1)
%     p_result(i,:) = ProjectPointOnLine(startp,bestCylinderModel.Orientation,currentClusterPoints(i,:));
% end
% toc

p_result = projectPointsToLine_new(currentClusterPoints, startp, bestCylinderModel.Orientation);


%----计算点在圆柱轴线的投影点到圆柱起点和终点的距离
Distances_startp = sqrt(sum((p_result - startp) .^ 2, 2));
Distances_endp = sqrt(sum((p_result - endp) .^ 2, 2));

%计算点的法向量与轴线的夹角
diff_angles = find_diff_angles(currentClusterPoints,startp,endp); 


%如果点位于膨胀圆柱体内部,并且聚类的标签与种子点属于同一个标签，则被删除
%关联点在currentClusterPoints的索引
KKKKKK = abs(D-model(7));
%expandedCylinderInliersIndex = find( abs(D-model(7))<3*sigma & (labels == label_SeedPoint) & (Distances_startp +Distances_endp)<bestCylinderModel.Height+0.001 ); 
expandedCylinderInliersIndex = find( abs(D-model(7))<magnification_sigma*sigma & (labels == label_SeedPoint) & (Distances_startp +Distances_endp)<bestCylinderModel.Height+0.001 &(diff_angles<10)); 






%找到圆柱两端附近并且位于弯曲段上的点云。这部分点云在圆周方向往往占据的弧长较小（像翅膀wing），可以依据这个观察，将其记录，最后排除
[WingPoints,WingPointsIndex] = Find_Wing_points_Index(expandedCylinderInliersIndex,currentClusterPoints,startp,endp,arc_length_ratio_proj);
%此处，WingPointsIndex是在expandedCylinderInliersIndex中的，而不是在currentClusterPoints中，切记

WingPointsIndex_incurrent = expandedCylinderInliersIndex(WingPointsIndex);
PureCylinderInliersIndex = expandedCylinderInliersIndex(~ismember(expandedCylinderInliersIndex, WingPointsIndex_incurrent));


expandedPureCylinderInliersIndex = PureCylinderInliersIndex; %是在currentClusterPoints中的索引

% 
% % %%%%%%%%%%%%%%%%%%%%%%%%
% % 
% temPoints = currentClusterPoints(PureCylinderInliersIndex,:);
% figureNum =15;
% figure(figureNum)
% pcshow(currentClusterPoints,[0.5,0.5,0.5],'MarkerSize',50); %显示点云数据
% hold on
% pcshow(temPoints,[1,0,0],'MarkerSize',50); %显示点云数据
% % %%%%%%%%%%%%%%%%%%%%%%%

end