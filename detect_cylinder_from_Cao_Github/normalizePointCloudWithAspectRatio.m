function P_normalized = normalizePointCloudWithAspectRatio(P)
    % P 是一个N*3的矩阵，每行代表一个点的 (x, y, z) 坐标

    % 计算点云的边界框
    minCoords = min(P);
    maxCoords = max(P);

    % 计算边界框的尺寸
    dimensions = maxCoords - minCoords;

    % 确定最长的边界框尺寸
    maxDimension = max(dimensions);

    % 计算缩放因子，使得最长边长为1000
    scaleFactor = 1000 / maxDimension;

    % 计算归一化后的点云
    P_normalized = (P - minCoords) * scaleFactor;

    % 计算归一化后点云的边界框尺寸
    newDimensions = dimensions * scaleFactor;

    % 计算额外的缩放因子，用于保持等比缩放
    aspectRatioScaleFactor = min(newDimensions) / 500;

    % 应用额外的缩放因子以保持等比缩放
    P_normalized = P_normalized * aspectRatioScaleFactor;
end