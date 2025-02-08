function Median_PointDistances = Compute_Median_PointDistances(this, K)

% 输入:
    % P: N x 3 的点云数据，N 是点的数量
    
    % 输出:
    % medianDist: 所有点与其 6 邻域之间距离平均值的中位数

    P = this.Location; 
    
    % 构建 KD 树
    kdtree = KDTreeSearcher(P);
    
    % 计算每个点的 6 邻域
    [neighborsIdx, neighborsDist] = knnsearch(kdtree, P, 'K', K + 1);
    
    % 去除每个点本身的距离（因为第一个邻居是点自己，距离为零）
    neighborsDist(:, 1) = [];
    
    % 计算每个点与其邻域之间的距离的平均值
    meanDistances = mean(neighborsDist, 2);
    
    % 返回所有平均值的中位数
    Median_PointDistances = median(meanDistances);

end