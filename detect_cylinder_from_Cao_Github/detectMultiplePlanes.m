function [planes, remainingPoints] = detectMultiplePlanes(P, remainingNormals, params)  
% detectMultiplePlanes 检测点云中的多个平面  
%  
% 输入参数：  
%   P: N×3的点云矩阵  
%   params: 参数结构体  
%       .distThreshold: 点到平面的距离阈值  
%       .minPoints: 平面最少包含的点数  
%       .maxPlanes: 最大检测平面数  
%       .confidence: RANSAC置信度  
%       .maxIterations: RANSAC最大迭代次数  
%  
% 输出参数：  
%   planes: 检测到的平面结构体数组  
%       .points: 平面上的点  
%       .normal: 平面法向量  
%       .d: 平面方程的d值（ax+by+cz+d=0）  
%   remainingPoints: 剩余未分类的点  

% 1. 参数初始化  
if nargin < 2  
    params.distThreshold = 0.5;    % 距离阈值  
    params.minPoints = 200;         % 最小点数  
    params.maxPlanes = 20;          % 最大平面数  
    params.confidence = 0.99;       % 置信度  
    params.maxIterations = 1000;    % 最大迭代次数  
end  

% 2. 初始化输出  
planes = struct('points',{},'normal',{},'d',{});  
remainingPoints = P;  
planeCount = 0;  

% 3. 主循环：持续检测平面直到满足终止条件  
while size(remainingPoints,1) >= params.minPoints && ...  
      planeCount < params.maxPlanes  
    
    % 3.1 使用RANSAC检测单个平面  
    % [planeModel, inlierIndices] = ransacFitPlane(remainingPoints, params);  

    [planeModel, inlierIndices] = ransacFitPlane(remainingPoints, remainingNormals, params);  
    
    % 3.2 如果找到满足条件的平面  
    if ~isempty(planeModel) && length(inlierIndices) >= params.minPoints  
        planeCount = planeCount + 1;  
        
        % 存储平面信息  
        planes(planeCount).points = remainingPoints(inlierIndices,:);  
        planes(planeCount).normal = planeModel(1:3);  
        planes(planeCount).d = planeModel(4);  
        
        % 更新剩余点  
        outlierIndices = setdiff(1:size(remainingPoints,1), inlierIndices);  
        remainingPoints = remainingPoints(outlierIndices,:);  
    else  
        break;  
    end  
end  
end  

% function [bestPlaneModel, bestInlierIndices] = ransacFitPlane(points, params)  
% % ransacFitPlane 使用RANSAC方法拟合平面  
% %  
% % 输入参数：  
% %   points: N×3的点云矩阵  
% %   params: 参数结构体  
% %  
% % 输出参数：  
% %   bestPlaneModel: [a,b,c,d]平面方程参数  
% %   bestInlierIndices: 内点索引  
% 
% bestPlaneModel = [];  
% bestInlierIndices = [];  
% bestInlierCount = 0;  
% N = size(points, 1);  
% 
% % 1. 计算需要的迭代次数  
% p = params.confidence;  
% e = 1 - params.minPoints/N;  
% s = 3; % 拟合平面需要的最小点数  
% maxIters = min(params.maxIterations, ceil(log(1-p)/log(1-(1-e)^s)));  
% 
% % 2. RANSAC主循环  
% for iter = 1:maxIters  
%     % 2.1 随机选择三个点  
%     idx = randperm(N, 3);  
%     pt1 = points(idx(1),:);  
%     pt2 = points(idx(2),:);  
%     pt3 = points(idx(3),:);  
% 
%     % 2.2 计算平面方程  
%     v1 = pt2 - pt1;  
%     v2 = pt3 - pt1;  
%     normal = cross(v1, v2);  
% 
%     % 跳过共线点  
%     if norm(normal) < 1e-6  
%         continue;  
%     end  
% 
%     normal = normal / norm(normal);  
%     d = -dot(normal, pt1);  
%     planeModel = [normal, d];  
% 
%     % 2.3 计算所有点到平面的距离  
%     distances = abs(points * normal' + d);  
% 
%     % 2.4 统计内点  
%     inlierIndices = find(distances <= params.distThreshold);  
%     inlierCount = length(inlierIndices);  
% 
%     % 2.5 更新最佳模型  
%     if inlierCount > bestInlierCount  
%         bestInlierCount = inlierCount;  
%         bestPlaneModel = planeModel;  
%         bestInlierIndices = inlierIndices;  
%     end  
% end  
% 
% % 3. 使用所有内点重新拟合平面（可选）  
% if ~isempty(bestInlierIndices)  
%     inlierPoints = points(bestInlierIndices,:);  
%     centroid = mean(inlierPoints);  
%     [~,~,V] = svd(inlierPoints - centroid,0);  
%     normal = V(:,3)';  
%     d = -dot(normal, centroid);  
%     bestPlaneModel = [normal, d];  
% end  
% end


function [bestPlaneModel, bestInlierIndices] = ransacFitPlane(points, pointNormals, params)  
% ransacFitPlane 使用 RANSAC 方法拟合平面  
%  
% 输入参数：  
%   points: N×3 的点云矩阵  
%   pointNormals: N×3 的点法向量矩阵  
%   params: 参数结构体  
%  
% 输出参数：  
%   bestPlaneModel: [a, b, c, d] 平面方程参数  
%   bestInlierIndices: 内点索引  

bestPlaneModel = [];  
bestInlierIndices = [];  
bestInlierCount = 0;  
N = size(points, 1);  

% 1. 计算需要的迭代次数  
p = params.confidence;  
e = 1 - params.minPoints/N;  
s = 3; % 拟合平面需要的最小点数  
maxIters = min(params.maxIterations, ceil(log(1 - p) / log(1 - (1 - e)^s)));  

% 2. RANSAC 主循环  
for iter = 1:maxIters  
    % 2.1 随机选择三个点  
    idx = randperm(N, 3);  
    pt1 = points(idx(1), :);  
    pt2 = points(idx(2), :);  
    pt3 = points(idx(3), :);  
    
    % 获取对应的法向量  
    n1 = pointNormals(idx(1), :);  
    n2 = pointNormals(idx(2), :);  
    n3 = pointNormals(idx(3), :);  
    
    % 计算法向量之间的夹角  
    angleThreshold = 5; % 角度阈值（度）  
    angle12 = acosd(dot(n1, n2) / (norm(n1) * norm(n2)));  
    angle13 = acosd(dot(n1, n3) / (norm(n1) * norm(n3)));  
    angle23 = acosd(dot(n2, n3) / (norm(n2) * norm(n3)));  
    
    % 检查法向量夹角是否小于阈值  
    if abs(angle12) > angleThreshold || abs(angle13) > angleThreshold || abs(angle23) > angleThreshold  
        continue; % 如果任意两个法向量的夹角大于阈值，跳过本次采样  
    end  
    
    % 2.2 计算平面方程  
    v1 = pt2 - pt1;  
    v2 = pt3 - pt1;  
    normal = cross(v1, v2);  
    
    % 跳过共线点  
    if norm(normal) < 1e-6  
        continue;  
    end  
    
    normal = normal / norm(normal);  
    d = -dot(normal, pt1);  
    planeModel = [normal, d];  
    
    % 2.3 计算所有点到平面的距离  
    distances = abs(points * normal' + d);  
    
    % 2.4 统计内点  
    inlierIndices = find(distances <= params.distThreshold);  
    inlierCount = length(inlierIndices);  
    
    % 2.5 更新最佳模型  
    if inlierCount > bestInlierCount  
        bestInlierCount = inlierCount;  
        bestPlaneModel = planeModel;  
        bestInlierIndices = inlierIndices;  
    end  
end  

% 3. 使用所有内点重新拟合平面（可选）  
if ~isempty(bestInlierIndices)  
    inlierPoints = points(bestInlierIndices, :);  
    centroid = mean(inlierPoints);  
    [~, ~, V] = svd(inlierPoints - centroid, 0);  
    normal = V(:, 3)';  
    d = -dot(normal, centroid);  
    bestPlaneModel = [normal, d];  
end  
end