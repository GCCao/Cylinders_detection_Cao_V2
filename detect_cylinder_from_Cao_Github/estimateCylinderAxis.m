function [eigvectors, eigvalues] = estimateCylinderAxis(normals_valid)
    % 检查输入是否为Nx3矩阵
    if size(normals_valid, 2) ~= 3
        error('Input must be a N x 3 matrix.');
    end
    
    % 将法向量转换为列向量，每列代表一个法向量
    normals_valid = normals_valid';
    
    % 计算法向量的协方差矩阵
    covariance_matrix = normals_valid * normals_valid';
    
    % 计算协方差矩阵的特征值和特征向量
    [ eigvectors, eigvalues ] = eig(covariance_matrix);
    
    % 将特征向量和特征值转换回原始矩阵的维度
    eigvectors = eigvectors';
    eigvalues = diag(eigvalues);
    
    % 根据特征值排序特征向量，方差最小的方向在最后
    % 因为MATLAB的eig函数返回的是升序排列的特征值和对应的特征向量
    [~, sort_index] = sort(eigvalues);
    eigvectors = eigvectors(:, sort_index);
    eigvalues = eigvalues(sort_index);
end