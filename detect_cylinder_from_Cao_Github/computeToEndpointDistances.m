function [distanceToAandB] = computeToEndpointDistances(P, A, B)  
    % P: Nx3 点云数据（矩阵）  
    % A: 直线的起点（1x3 向量）  
    % B: 直线的终点（1x3 向量）  
    % distanceToA: 每个点到 A 的距离（Nx1 向量）  
    % distanceToB: 每个点到 B 的距离（Nx1 向量）  

    % 计算每个点到 A 的距离  
    distanceToA = sqrt(sum((P - A).^2, 2)); % Nx1 向量  

    % 计算每个点到 B 的距离  
    distanceToB = sqrt(sum((P - B).^2, 2)); % Nx1 向量  

    distanceToAandB = distanceToA + distanceToB;
end  