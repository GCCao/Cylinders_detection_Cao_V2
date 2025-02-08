function [planes, remainingPoints] = detectMultiplePlanes_new(P, params)  
% detectMultiplePlanes 检测点云中的多个平面并验证其合法性  
%  
% 输入参数：  
%   P: N×3的点云矩阵  
%   params: 参数结构体  
%  
% 输出参数：  
%   planes: 检测到的平面结构体数组  
%   remainingPoints: 剩余未分类的点  

% 1. 参数初始化  
if nargin < 2  
    params.distThreshold = 0.2;    % 点到平面的距离阈值  
    params.minPoints = 100;         % 平面最少包含的点数  
    params.maxPlanes = 10;          % 最大平面数  
    params.confidence = 0.99;       % RANSAC置信度  
    params.maxIterations = 1000;    % RANSAC最大迭代次数  
    params.angleThreshold = 10;     % 法向量夹角阈值（度）  
    params.invalidRatioThreshold = 0.3; % 无效点比例阈值  
    params.kNeighbors = 30;         % 计算法向量的邻近点数  
end  

% 2. 初始化输出  
planes = struct('points',{},'normal',{},'d',{});  
remainingPoints = P;  
planeCount = 0;  
iterationCount = 0;  
maxIteration = 20;  % 设置最大迭代次数为10  

% 3. 主循环：持续检测平面直到满足终止条件  
while size(remainingPoints,1) >= params.minPoints && ...  
      planeCount < params.maxPlanes && ...  
      iterationCount < maxIteration  % 使用新的终止条件  
    
    iterationCount = iterationCount + 1;  
    
    % 3.1 使用RANSAC检测单个平面  
    [planeModel, inlierIndices] = ransacFitPlane(remainingPoints, params);  
    
    % 3.2 如果找到满足条件的平面  
    if ~isempty(planeModel) && length(inlierIndices) >= params.minPoints  
        % 获取平面点云  
        planePoints = remainingPoints(inlierIndices,:);  
        
        % 创建点云对象用于计算法向量  
        ptCloud = pointCloud(planePoints);  
        
        % 计算点云法向量  
        pointNormals = pcnormals(ptCloud, params.kNeighbors);  
        
        % 检查平面的合法性  
        isValidPlane = checkPlaneValidity(planePoints, pointNormals, ...  
                                        planeModel(1:3), params);  
        
        if isValidPlane  
            % 存储合法平面  
            planeCount = planeCount + 1;  
            planes(planeCount).points = planePoints;  
            planes(planeCount).normal = planeModel(1:3);  
            planes(planeCount).d = planeModel(4);  
            
            % 更新剩余点  
            outlierIndices = setdiff(1:size(remainingPoints,1), inlierIndices);  
            remainingPoints = remainingPoints(outlierIndices,:);  
        else  
            % 不合法平面的点保留在remainingPoints中  
            continue;  
        end  
    else  
        break;  
    end  
end  

% 输出统计信息  
fprintf('Detection completed:\n');  
fprintf('- Total iterations: %d\n', iterationCount);  
fprintf('- Detected planes: %d\n', planeCount);  
fprintf('- Remaining points: %d\n', size(remainingPoints,1));  
if iterationCount >= maxIteration  
    fprintf('Warning: Maximum iteration limit (10) reached\n');  
end  

end  

function [bestPlaneModel, bestInlierIndices] = ransacFitPlane(points, params)  
% ransacFitPlane 使用RANSAC方法拟合平面  

bestPlaneModel = [];  
bestInlierIndices = [];  
bestInlierCount = 0;  
N = size(points, 1);  

% 计算需要的迭代次数  
p = params.confidence;  
e = 1 - params.minPoints/N;  
s = 3;  
maxIters = min(params.maxIterations, ceil(log(1-p)/log(1-(1-e)^s)));  

% RANSAC主循环  
for iter = 1:maxIters  
    % 随机选择三个点  
    idx = randperm(N, 3);  
    pt1 = points(idx(1),:);  
    pt2 = points(idx(2),:);  
    pt3 = points(idx(3),:);  
    
    % 计算平面方程  
    v1 = pt2 - pt1;  
    v2 = pt3 - pt1;  
    normal = cross(v1, v2);  
    
    if norm(normal) < 1e-6  
        continue;  
    end  
    
    normal = normal / norm(normal);  
    d = -dot(normal, pt1);  
    planeModel = [normal, d];  
    
    % 计算所有点到平面的距离  
    distances = abs(points * normal' + d);  
    
    % 统计内点  
    inlierIndices = find(distances <= params.distThreshold);  
    inlierCount = length(inlierIndices);  
    
    if inlierCount > bestInlierCount  
        bestInlierCount = inlierCount;  
        bestPlaneModel = planeModel;  
        bestInlierIndices = inlierIndices;  
    end  
end  

if ~isempty(bestInlierIndices)  
    inlierPoints = points(bestInlierIndices,:);  
    centroid = mean(inlierPoints);  
    [~,~,V] = svd(inlierPoints - centroid,0);  
    normal = V(:,3)';  
    d = -dot(normal, centroid);  
    bestPlaneModel = [normal, d];  
end  
end  

function isValid = checkPlaneValidity(points, pointNormals, planeNormal, params)  
% checkPlaneValidity 检查平面的合法性  

numPoints = size(points, 1);  

% 确保法向量方向一致  
for i = 1:numPoints  
    if dot(pointNormals(i,:), planeNormal) < 0  
        pointNormals(i,:) = -pointNormals(i,:);  
    end  
end  

% 计算法向量夹角  
angles = zeros(numPoints, 1);  
for i = 1:numPoints  
    angles(i) = acosd(abs(dot(pointNormals(i,:), planeNormal)));  
end  

% 统计不符合条件的点  
invalidPoints = angles > params.angleThreshold;  
invalidRatio = sum(invalidPoints) / numPoints;  

% 判断是否合法  
isValid = invalidRatio <= params.invalidRatioThreshold;  
end