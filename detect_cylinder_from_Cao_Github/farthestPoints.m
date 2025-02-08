function [point1, point2, maxDistance] = farthestPoints(P_proj)  
    % P_proj: N×3 的点云数据  
    % point1: 距离最远的第一个点  
    % point2: 距离最远的第二个点  
    % maxDistance: 这两个点之间的最大距离  

    N = size(P_proj, 1); % 点的数量  
    maxDistance = 0;     % 初始化最大距离  
    point1 = [];         % 初始化第一个点  
    point2 = [];         % 初始化第二个点  

    % 遍历所有点对  
    for i = 1:N  
        for j = i+1:N  
            % 计算当前点对的距离  
            currentDistance = sqrt(sum((P_proj(i, :) - P_proj(j, :)).^2));  

            % 检查是否是最大距离  
            if currentDistance > maxDistance  
                maxDistance = currentDistance; % 更新最大距离  
                point1 = P_proj(i, :);         % 更新点1  
                point2 = P_proj(j, :);         % 更新点2  
            end  
        end  
    end 
end