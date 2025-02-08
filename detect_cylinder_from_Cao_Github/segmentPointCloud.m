function [segments] = segmentPointCloud(P, A, B)  
    % P: Nx3 的点云数据  
    % A: 圆柱体轴线起点 (1x3 向量)  
    % B: 圆柱体轴线终点 (1x3 向量)  
    % segments: 100个段的点云数据，cell 数组，每个 cell 包含对应段的点云  

    % 计算轴线的方向和长度  
    direction = B - A;  
    lengthC = norm(direction);  
    unitDirection = direction / lengthC;  

    % 定义每段的长度  
    segmentLength = lengthC / 100;  

    %计算所有点在AB直线上的投影 
    P_proj = projectPointsToLine_by_MatrixOperation(P, A, B); 

    
    % 初始化 cell 数组来存储每个段的点云  
    segments = cell(1, 100);  
    
    % 遍历每个段  
    for i = 1:100  
        % 计算当前段的区间  
        startPoint = A + (i-1) * segmentLength * unitDirection;  
        endPoint = A + i * segmentLength * unitDirection;  

        % 分类点云到当前段  
        % 计算点到段的最近距离  
        % distance = pointToSegmentDistance(P, startPoint, endPoint);  
        [distanceToStartandEnd] = computeToEndpointDistances(P_proj,startPoint, endPoint);  
              

        % 找到在当前段范围内的点  
        isWithinSegment = distanceToStartandEnd < (segmentLength+0.0001);  

        idx = find(distanceToStartandEnd < (segmentLength+0.0001));
        % segments{i} = P(isWithinSegment, :);  
        segments{i} = P(idx, :);  
    end  
end  
