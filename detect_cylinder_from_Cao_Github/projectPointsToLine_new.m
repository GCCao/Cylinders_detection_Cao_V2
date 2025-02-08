function ProjPoints = projectPointsToLine_new(P, q, direction)
    % P: N*3 点云 (N rows, 3 columns)
    % q: 直线L上的点 (1x3 vector)
    % direction: 直线L的方向向量 (1x3 vector)
    
    % 计算 direction 的长度平方
    d2 = dot(direction, direction);
    
    % P 中的每个点减去直线L上的点 q，得到每个点到 q 的向量
    v = P - q;
    
    % 计算 v 在 d 上的投影比例 t
    t = (v * direction') / d2;
    
    % 计算所有投影点的位置
    ProjPoints = q + t .* direction;
end