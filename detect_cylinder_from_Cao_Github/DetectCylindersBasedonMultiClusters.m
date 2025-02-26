function [detectedCylinders,detectedCylinders_pointCloud,remainingPointCloud] = DetectCylindersBasedonMultiClusters(clusterLabels,detectedCylinders,detectedCylinders_pointCloud,numClusters, remainingPointCloud,minClustersDistance,minSizeValidCluster,magnification_sigma,Sampling_Order,neighboringSize)
%依次对每个点族进行圆柱拟合，输出检测到圆柱，以及剩余的点云remainingPointCloud

% minR = 3;                 %最小圆柱半径  
% maxR = 250;               %最大圆柱半径

% %当前已有的圆柱数量
% numCylinders = size(detectedCylinders,1);

%记录所有将被删除点的索引，都暂存在All_Indices_to_be_removed_inremain中，当所有的族中圆柱检测都完成后，统一删除
All_Indices_to_be_removed_inremain = [];

% 对每个簇进行圆柱检测
for i = 1:numClusters
    % 获取当前簇的点云
    %找到第i个聚类对应的点云索引,是在remainingPointCloud中的索引

    Index_currentClusterPoints_inremain = find(clusterLabels == i);
    currentClusterPoints = remainingPointCloud(Index_currentClusterPoints_inremain,:);

    % 对当前簇进行圆柱检测
    % [bestCylinderModel,predictCylinderModel,Index_detectedPoints_inremain] = DetectingSingleCylinder_By_KpointSampling(currentClusterPoints,Index_currentClusterPoints_inremain,remainingPointCloud);
    [bestCylinderModel,predictCylinderModel,sigma,SeedPointIndexs_incurrent,bestCylinderInliersIndex] = DetectingSingleCylinder_By_KpointSampling(currentClusterPoints,minClustersDistance,minSizeValidCluster,magnification_sigma,neighboringSize);

    if isempty(bestCylinderModel)
        % All_Indices_to_be_removed_inremain = [All_Indices_to_be_removed_inremain;Index_currentClusterPoints_inremain];
        continue;
    end

    % if predictCylinderModel==2
    %     ALLInValidFeatureVectors =  [ALLInValidFeatureVectors;FeatureVector_CylindricalError];
    % end


    %（5.5.8）如果检测到满足所有要求的圆柱   
    if ~isempty(bestCylinderModel)  %& predictCylinderModel==1

        % ALLValidFeatureVectors =  [ALLValidFeatureVectors;FeatureVector_CylindricalError];

        % （5.5.8.1）将找到的圆柱体模型存储起来
        % numCylinders = numCylinders + 1;
        
        detectedCylinders{end+1} =  bestCylinderModel;

  

        % A = bestCylinderModel.Parameters(1:3);
        % B = bestCylinderModel.Parameters(4:6);
        % 
        % AB_norm = norm(A-B);
        % if AB_norm == 0
        %     ee=0;
        % end


        % % 绘制备选的空间点
        % figure(1)
        % hold on
        % plot3(subpoints(:,1), subpoints(:,2), subpoints(:,3), 'bo','MarkerSize',10);  % 'bo'表示蓝色圆点
        % % 绘制法向量
        % for i = 1:size(subpoints, 1)
        %     % 计算法向量在每个点的位置
        %     figure(1)
        %     hold on
        %     quiver3(subpoints(i,1), subpoints(i,2), subpoints(i,3), ...
        %         subnormal(i,1)*50, subnormal(i,2)*50, subnormal(i,3)*50, ...
        %         'MaxHeadSize', 5,'Color', 'magenta', 'LineWidth', 5); %MaxHeadSize表示箭头大小， 'r'表示红色箭头，'LineWidth'设置线宽，
        % end

        % figure(1)
        % hold on
        % plot(bestCylinderModel);
        %
        % set(gca, 'view', [20, 60]);


        %（5.5.8.2）删除对后续圆柱检测有干扰的点，为了删除的比较彻底，将最小二乘检测到的圆柱进行膨胀，即把半径增加3倍sigma
        %膨胀圆柱体内部的点，只要距离表面较远，不会被删除，将参与后续圆柱体检测
       
        % expandedCylinderInliersIndex =  Delete_the_visited_pointcloud_by_expanded_cylinder_cluster(bestCylinderModel,currentClusterPoints,sigma,SeedPointIndexs_incurrent,minClustersDistance);
        
        expandedCylinderInliersIndex =  Delete_the_visited_pointcloud_by_expanded_cylinder_cluster(bestCylinderModel,currentClusterPoints,3,sigma,SeedPointIndexs_incurrent,minClustersDistance,0.25);


        %找到圆柱体被删除点在remainingPointCloud的索引
        expandedCylinderInliersIndex_inremain = Index_currentClusterPoints_inremain(expandedCylinderInliersIndex);


        %记录所有将被删除点的索引，都暂存在All_Indices_to_be_removed_inremain中，当所有的族中圆柱检测都完成后，统一删除
        All_Indices_to_be_removed_inremain = [All_Indices_to_be_removed_inremain;expandedCylinderInliersIndex_inremain];

        % figure(1)        
        % hold on
        % color = rand(1, 3);
        % pcshow(remainingPointCloud(expandedCylinderInliersIndex_inremain, :),color,'MarkerSize',50);
        % set(gca, 'view', [20, 60]);

        figure(Sampling_Order)        
        hold on
        color = rand(1, 3);
        bestCylinderInliersIndex_inremain = Index_currentClusterPoints_inremain(bestCylinderInliersIndex);
        pcshow(remainingPointCloud(bestCylinderInliersIndex_inremain, :),color,'MarkerSize',50);
        set(gca, 'view', [20, 60]);



        %检测到的圆柱体对应的点云,也是用于可视化的点云
        detectedCylinders_pointCloud{end+1} =  remainingPointCloud(bestCylinderInliersIndex_inremain, :);

         PPP = remainingPointCloud(bestCylinderInliersIndex_inremain, :);
         if norm(PPP(1,:) - PPP(2,:) ) <0.001 || size(PPP,2) == 0
             ee=0;
         end


        % figure(1)
        % hold on
        % plot(bestCylinderModel);
        % set(gca, 'view', [20, 60]);    
        %%%%%%%%%%%%%%%%%%%%%%%%%
for ii=1:size(detectedCylinders,2)

    P = detectedCylinders_pointCloud{ii};
    Cylinder = detectedCylinders{ii};

    A = Cylinder.Parameters(1:3);
    B = Cylinder.Parameters(4:6);

    AB = B - A;  
    AB_norm = norm(AB);  

    i

    if size(P,2) ==0
        ee=0;
    end

    if size(P,2) ~= size(A,2)
        ee=0;
    end
    
    % 如果 AB_norm 为零，返回 NaN 或者原始点云  
    if AB_norm == 0  
        error('A 和 B 不能是相同的点');  
    end  
end
%%%%%%%%%%%%%%%%%%%%%%%

    end


   
end

remainingPointCloud(All_Indices_to_be_removed_inremain, :) = [];
% remainingPointCloudNormal(All_Indices_to_be_removed_inremain, :) = [];



end