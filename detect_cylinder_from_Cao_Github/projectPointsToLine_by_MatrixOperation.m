function P_proj = projectPointsToLine_by_MatrixOperation(P, A, B)  
    % P: Nx3 点云数据（矩阵）  
    % A: 直线的起点（1x3 向量）  
    % B: 直线的终点（1x3 向量）  
    % P_proj: 投影到直线上的点云（Nx3 矩阵）  

    % 计算直线方向向量  
    AB = B - A;  
    AB_norm = norm(AB);  
    
    % 如果 AB_norm 为零，返回 NaN 或者原始点云  
    if AB_norm == 0  
        error('A 和 B 不能是相同的点');  
    end  
    
    % 单位方向向量  
    AB_unit = AB / AB_norm;  

    if size(P,2) ~= size(A,2)
        ee=0;
    end

    % P 到 A 的向量  
    AP = P - A;   

    % 计算每个点在直线方向上的投影系数  
    % projection_coeff = dot(AP, AB_unit, 2); % 得到 Nx1 向量  
    % % 计算每个 AP 的投影系数  
    projectionCoeffs = dot(AP, repmat(AB_unit, size(AP, 1), 1), 2); % Nx1 向量  
    
    % 计算投影点  
    P_proj = A + projectionCoeffs * AB_unit; % Nx3 矩阵  
end