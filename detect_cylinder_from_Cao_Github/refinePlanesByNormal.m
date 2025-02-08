function [planes, remainingPoints] = refinePlanesByNormal(planes, remainingPoints)  
% refinePlanesByNormal 基于法向量一致性对平面进行优化  
%  
% 输入参数：  
%   planes: 检测到的平面结构体数组  
%       .points: 平面上的点  
%       .normal: 平面法向量  
%       .d: 平面方程的d值  
%   remainingPoints: 未分类的点云  
%  
% 输出参数：  
%   planes: 更新后的平面结构体数组  
%   remainingPoints: 更新后的未分类点云  

% 参数设置  
angleThreshold = 10;  % 法向量夹角阈值（度）  
invalidRatioThreshold = 0.4;  % 无效点比例阈值  
k = 6;  % 用于计算法向量的邻近点数量  

% 记录需要删除的平面索引  
planesToRemove = [];  

% 遍历所有平面  
for i = 1:length(planes)  
    % 获取当前平面的点云  
    planePoints = planes(i).points;  
    numPoints = size(planePoints, 1);  
    
    % 创建点云对象  
    ptCloud = pointCloud(planePoints);  
    
    % 计算每个点的法向量  
    normals = pcnormals(ptCloud, k);  
    
    % 确保法向量方向一致（与平面法向量指向相同的半球）  
    for j = 1:size(normals, 1)  
        if dot(normals(j,:), planes(i).normal) < 0  
            normals(j,:) = -normals(j,:);  
        end  
    end  
    
    % 计算每个点的法向量与平面法向量的夹角  
    angles = zeros(numPoints, 1);  
    for j = 1:numPoints  
        angles(j) = acosd(abs(dot(normals(j,:), planes(i).normal)));  
    end  
        

    % 找出不符合条件的点（夹角大于阈值）  
    invalidPoints = angles > angleThreshold;  
    invalidRatio = sum(invalidPoints) / numPoints;  
    
    % 如果无效点比例超过阈值，将该平面标记为待删除  
    if invalidRatio > invalidRatioThreshold  
        remainingPoints = [remainingPoints; planePoints];  
        planesToRemove = [planesToRemove, i];  
    end  
end  

% 删除不合法的平面（从后向前删除，避免索引变化）  
planes(planesToRemove) = [];  



end