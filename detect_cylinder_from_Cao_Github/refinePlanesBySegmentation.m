function [planes, remainingPoints] = refinePlanesBySegmentation(planes, remainingPoints)  
% refinePlanesBySegmentation 使用pcsegdist对平面点云进行聚类分析  
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

% 设置聚类距离阈值  
distanceThreshold = 5;  

% 记录需要删除的平面索引  
planesToRemove = [];  

% 遍历所有平面  
for i = 1:length(planes)  
    % 创建点云对象  
    planePC = pointCloud(planes(i).points);  
    
    % 使用pcsegdist进行聚类  
    [labels, numClusters] = pcsegdist(planePC, distanceThreshold);  
    
    % 如果聚类数量大于2  
    if numClusters > 2  
        % 将该平面的点加入到remainingPoints  
        remainingPoints = [remainingPoints; planes(i).points];  
        % 记录需要删除的平面索引  
        planesToRemove = [planesToRemove, i];  
    end  
end  

% 删除不满足条件的平面（从后向前删除，避免索引变化）  
planes(planesToRemove) = [];  

end