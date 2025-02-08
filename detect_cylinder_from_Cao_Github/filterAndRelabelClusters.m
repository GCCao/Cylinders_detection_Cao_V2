function [clusterLabels, numClusters, remainingPointCloud] = filterAndRelabelClusters(remainingPointCloud,minClustersDistance,minSizeValidCluster)
%该函数是把点云进行聚类，对于数量小于minSizeValidCluster的族，认为不可靠，不再参与后续的计算，当删除，同时更新remainingPointCloud
% remainingPointCloud：总的剩余点，是一个N*3的数组
% minSizeValidCluster：具有资格参与后续运算的最小族尺寸

%Convert the array to a point cloud class.
points = pointCloud(remainingPointCloud);

% 使用pcsegdist对点云进行聚类
[clusterLabels, numClusters] = pcsegdist(points, minClustersDistance);

% 计算每个族的点数
clusterCounts = histcounts(clusterLabels, 0.5:1:numClusters+0.5);

% 找到点数小于或等于minSizeValidCluster的族
InvalidClusters = find(clusterCounts <= minSizeValidCluster);


%将点数小于或等于minSizeValidCluster的族对应的点删除，更新remainingPointCloud
InvalidClusterID = [];
indexsInvalidClusterLabels =[];
if ~isempty(InvalidClusters)
    for i = 1:length(InvalidClusters)
        % InvalidClusterID = [InvalidClusterID;InvalidClusters(i)]; 
        InvalidClusterID = InvalidClusters(i);
        indexsInvalidClusterLabels = [indexsInvalidClusterLabels; find(clusterLabels == InvalidClusterID)];
    end

    remainingPointCloud(indexsInvalidClusterLabels,:) =[];

    %对更新后的remainingPointCloud重新聚类，此时得到的族都是满足点数要求的
    points = pointCloud(remainingPointCloud);

    % 使用pcsegdist对点云进行聚类
    [clusterLabels, numClusters] = pcsegdist(points, minClustersDistance);




       %%%%%%%-------------------------
    % 计算每个族的点数
    clusterCounts = histcounts(clusterLabels, 0.5:1:numClusters+0.5);

    % 找到点数小于或等于minSizeValidCluster的族
    InvalidClusters = find(clusterCounts <= minSizeValidCluster);

    if ~isempty(InvalidClusters)
        ee=0;
    end
    %%%%%%-------------
end
% %将点数小于或等于minSizeValidCluster的族对应的点删除，更新remainingPointCloud
% if ~isempty(InvalidClusters)
%     for i = 1:length(InvalidClusters)
%         InvalidClusterID = InvalidClusters(i);        
%         remainingPointCloud(find(clusterLabels == InvalidClusterID),:) =[];
%     end
% 
%     %对更新后的remainingPointCloud重新聚类，此时得到的族都是满足点数要求的
%     points = pointCloud(remainingPointCloud);
% 
%     % 使用pcsegdist对点云进行聚类
%     [clusterLabels, numClusters] = pcsegdist(points, minClustersDistance);
% 
% 
% 
% 
%     %%%%%%%-------------------------
%     % 计算每个族的点数
%     clusterCounts = histcounts(clusterLabels, 0.5:1:numClusters+0.5);
% 
%     % 找到点数小于或等于minSizeValidCluster的族
%     InvalidClusters = find(clusterCounts <= minSizeValidCluster);
% 
%     if ~isempty(InvalidClusters)
%         ee=0;
%     end
%     %%%%%%%-------------
% end




end