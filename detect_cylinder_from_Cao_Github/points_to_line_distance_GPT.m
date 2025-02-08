function distances = points_to_line_distance_GPT(P, P1, P2)
    % 计算向量 P1P2
    P1P2 = P2 - P1;
    
    % 计算 P1P2 的模（长度）
    P1P2_length = norm(P1P2);
    
    % 计算单位向量 P1P2
    P1P2_unit = P1P2 / P1P2_length;
    
    % 计算每个点到 P1 的向量
    P1P = P - P1;
    
    % 计算每个点在 P1P2 方向上的投影长度
    projection_lengths = P1P * P1P2_unit';
    
    % 计算每个点在 P1P2 方向上的投影向量
    projection_vectors = projection_lengths * P1P2_unit;
    
    % 计算每个点到直线的垂线段的向量
    perpendicular_vectors = P1P - projection_vectors;
    
    % 计算每个点到直线的垂线段的长度，即点到直线的距离
    distances = sqrt(sum(perpendicular_vectors.^2, 2));
end

