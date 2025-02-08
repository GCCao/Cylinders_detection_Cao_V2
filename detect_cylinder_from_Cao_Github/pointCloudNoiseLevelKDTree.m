function medianVariance = pointCloudNoiseLevelKDTree(P, k)
%计算点云的噪声等级
%假设P是一个N*3的点云，请计算每个点的k邻域，并采用PCA方法将k邻域拟合成平面，计算每个邻域点到该平面的距离，求解k个距离值的方差，最后输出所有方差的中值。

    % 输入: 
    % P - N*3 点云数据矩阵
    % k - 邻域中点的数量

    

    % 获取点云中点的数量
    N = size(P, 1);

   

    % 使用KDTree构建点云索引
    MdlKDT = KDTreeSearcher(P);

    %对第N/2进行局部采样，它的邻域尺寸为N/100。这样做是为了加速。
    idx = knnsearch(MdlKDT, P(round(N/2), :), 'K', round(N/100));
    Localneighbors = P(idx, :);



    %更新P
    P = Localneighbors;
    N = size(P, 1);
     % 初始化方差数组
    variances = zeros(N, 1);
    % 使用KDTree构建点云索引
    MdlKDT = KDTreeSearcher(P);

    % 对于每个点，使用KDTree查找k邻域
    for i = 1:N
        % 查找k邻域点
        idx = knnsearch(MdlKDT, P(i, :), 'K', k);
        neighbors = P(idx, :);

        % 使用PCA对邻域点进行拟合
        [coeff, ~, ~] = pca(neighbors);

        % 取主成分的法向量（最后一列）
        normalVector = coeff(:, 3);

        % 计算每个邻域点到拟合平面的距离
        distancesToPlane = abs((neighbors - P(i, :)) * normalVector);

        % 计算这些距离的方差
        variances(i) = var(distancesToPlane);
    end

    % 输出所有方差的中值
    medianVariance = median(variances);
end