function [C, D, maxDistance] = findFarthestPoints_inP_proj_new(P_proj)  
    % P_proj: N×3 的点云数据    
    % C: 加权距离最大点 (1×3 向量)  
    % D: 与 C 距离最远的点 (1×3 向量)  
    % maxDistance: C 与 D 之间的最大距离 (标量)  

 
    % % 检查输入数据的维度  
    % if size(P_proj, 2) ~= 3  
    %     error('输入点云必须是 N×3 的矩阵');  
    % end  
    % 
    % % 计算质心  
    % centroid = mean(P_proj, 1); % 在第一维（每列）上取平均  
    % 
    % 
    % % 计算点centroid到所有点的距离  
    % dcentroid = sqrt(sum((P_proj - centroid).^2, 2)); % N×1 列向量  
    % 
    % 
    % % 找到加权距离最大的点C  
    % [~, indexC] = max(dcentroid);  
    % C = P_proj(indexC, :); % 加权距离最大的点  
    % 
    % 
    % % 计算C到所有点的距离  
    % distancesToC = sqrt(sum((P_proj - C).^2, 2)); % N×1 列向量  
    % 
    % % 找到与C距离最远的点D  
    % % 排除自身C  
    % distancesToC(indexC) = -Inf; % 将 C 的距离设为负无穷以排除  
    % [maxDistance, indexD] = max(distancesToC);  
    % D = P_proj(indexD, :); % 最远的点  

    if isempty(P_proj) || size(P_proj, 2) ~= 3  
        %error('输入的点集必须是N x 3的矩阵。');  
        C=[];
        D=[];
        maxDistance =[];
        return;
    end  

    % 找到X坐标最小的点  
    [minx, minIndex] = min(P_proj(:, 1));  % 1 列是X坐标

    % 找到X坐标最大的点
    [maxx, maxIndex] = max(P_proj(:, 1));  % 1 列是X坐标

    minPoint = P_proj(minIndex, :);
    maxPoint = P_proj(maxIndex, :);

    if  (maxx - minx)<0.01 %说明点云主要分布在垂直于X轴的方向

        % 找到Y坐标最小的点
        [miny, minIndex] = min(P_proj(:, 2));  % 1 列是X坐标

        % 找到Y坐标最大的点
        [maxy, maxIndex] = max(P_proj(:, 2));  % 1 列是X坐标

        minPoint = P_proj(minIndex, :);
        maxPoint = P_proj(maxIndex, :);

        if  (maxy - miny)<0.01 %说明点云主要分布在垂直于X轴的方向

            % 找到z坐标最小的点
            [minz, minIndex] = min(P_proj(:, 3));  % 1 列是X坐标

            % 找到z坐标最大的点
            [maxz, maxIndex] = max(P_proj(:, 3));  % 1 列是X坐标

            minPoint = P_proj(minIndex, :);
            maxPoint = P_proj(maxIndex, :);

        end

    end

    


    maxDistance = norm(minPoint - maxPoint);

    C = minPoint;
    D = maxPoint;



    
    
end  