function [C, D, maxDistance] = findFarthestPoints_inP_proj(P_proj, A, B)  
    % P_proj: N×3 的点云数据  
    % A: 第一个端点 (1×3 向量)  
    % B: 第二个端点 (1×3 向量)  
    % C: 加权距离最大点 (1×3 向量)  
    % D: 与 C 距离最远的点 (1×3 向量)  
    % maxDistance: C 与 D 之间的最大距离 (标量)  

    % 计算点A到所有点的距离  
    dA = sqrt(sum((P_proj - A).^2, 2)); % N×1 列向量  
    % 计算点B到所有点的距离  
    dB = sqrt(sum((P_proj - B).^2, 2)); % N×1 列向量  

    % 计算加权距离  
    d = dA + dB; % N×1 列向量  

    dAB = norm(A-B); 

    % 找到加权距离最大的点C  
    [~, indexC] = max(d);  
    C = P_proj(indexC, :); % 加权距离最大的点  

    % 判断C是否在A和B之间
    distancesAToC = norm(A-C);
    distancesBToC = norm(C-B);
    if (distancesAToC+distancesBToC)< (dAB+0.001)
        C = A;
        D = B;
        maxDistance = dAB;
        return;
    end


    % 计算C到所有点的距离  
    distancesToC = sqrt(sum((P_proj - C).^2, 2)); % N×1 列向量  

    % 找到与C距离最远的点D  
    % 排除自身C  
    distancesToC(indexC) = -Inf; % 将 C 的距离设为负无穷以排除  
    [maxDistance, indexD] = max(distancesToC);  
    D = P_proj(indexD, :); % 最远的点  
end  