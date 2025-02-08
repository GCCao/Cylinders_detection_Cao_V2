function indicesInPlane = findPointsInPlane(planeParams, remainingPointCloud)
    % planeParams: 包含平面方程参数的1x4向量 [a, b, c, d]
    % remainingPointCloud: N*3 数组，每行代表一个点的 (x, y, z) 坐标
    % indicesInPlane: 位于平面内的点的索引数组

    % 提取平面方程参数
    a = planeParams(1);
    b = planeParams(2);
    c = planeParams(3);
    d = planeParams(4);
    
    % 初始化索引数组
    indicesInPlane = [];
    
    % 遍历点云中的每个点
    for idx = 1:size(remainingPointCloud, 1)
        % 提取当前点的坐标
        x = remainingPointCloud(idx, 1);
        y = remainingPointCloud(idx, 2);
        z = remainingPointCloud(idx, 3);
        
        % 计算点到平面的距离
        distance = abs(a * x + b * y + c * z + d) / sqrt(a^2 + b^2 + c^2);
        
        % 检查距离是否小于0.5
        if distance < 0.5
            % 如果点在平面内，添加索引到索引数组
            indicesInPlane = [indicesInPlane, idx];
        end
    end
end