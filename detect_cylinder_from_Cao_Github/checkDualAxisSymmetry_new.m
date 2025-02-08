function [isSymmetric, symmetryScores, axes] = checkDualAxisSymmetry_new(Q, tolerance)  
% Q: N×2 投影点矩阵（已转换到平面坐标系）  
% tolerance: 对称性判断的容差  
% isSymmetric: 是否双轴对称的布尔值  
% axes: 2×2 矩阵，每行表示一个对称轴的方向向量  

% 1. 计算点集的质心  
centroid = mean(Q, 1);  

% 2. 将点集平移到质心（使用 bsxfun 更高效）  
Q_centered = bsxfun(@minus, Q, centroid);  

% 3. 计算协方差矩阵（优化矩阵运算）  
n = size(Q, 1);  
covariance = (Q_centered' * Q_centered) / n;  

% 4. 计算特征值和特征向量  
[eigVectors, eigValues] = eig(covariance);  
axes = eigVectors';  

% 5. 预分配内存  
symmetryScores = zeros(2, 1);  
n_points = size(Q_centered, 1);  

% 6. 向量化计算对称性得分  
for i = 1:2  
    axis = axes(i,:);  
    
    % 计算所有点关于当前轴的对称点（向量化）  
    proj = Q_centered * axis' * axis;  
    reflectedPoints = 2 * proj - Q_centered;  
    
    % 计算距离矩阵（使用 pdist2 函数）  
    distMatrix = pdist2(Q_centered, reflectedPoints);  
    
    % 计算每个点到最近对称点的距离  
    minDists = min(distMatrix, [], 2);  
    
    % 计算对称性得分  
    symmetryScores(i) = mean(minDists);  
end  

% 7. 判断是否双轴对称  
isSymmetric = all(symmetryScores < tolerance);  

% 8. 如果需要，将轴向量转换回原坐标系  
axes = axes + centroid;  
end