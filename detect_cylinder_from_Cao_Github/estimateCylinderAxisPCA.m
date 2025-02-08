function cylinder_axis = estimateCylinderAxisPCA(normals_valid)
    % 检查输入矩阵的维度
    if size(normals_valid, 2) ~= 3
        error('Input must be a N x 3 matrix.');
    end
    
    % 将法向量矩阵转换为列向量
    normals_valid = normals_valid';

    % 计算法向量的协方差矩阵
    covariance_matrix = normals_valid * normals_valid';

    % 计算协方差矩阵的特征值和特征向量
    [V, D] = eig(covariance_matrix);

    % 找到特征值最小的索引，即方差最小的方向
    min_eigvalue_index = find(diag(D) == min(diag(D)));

    % 获取对应的特征向量，即圆柱体的轴线
    cylinder_axis = V(:, min_eigvalue_index);

    % 将特征向量转换为行向量
    cylinder_axis = cylinder_axis';
end