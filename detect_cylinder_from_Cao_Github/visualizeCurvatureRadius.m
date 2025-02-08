function visualizeCurvatureRadius(P, normals)  
    % 输入：  
    % P: N*3的点云数组  
    % normals: N*3的法向量数组  
    
    % 参数设置  
    k = 10;  % 近邻点个数  
    
    % 初始化曲率半径数组  
    N = size(P, 1);  
    curvatureRadius = zeros(N, 1);  
    
    % 构建KD树用于快速近邻搜索  
    kdtree = KDTreeSearcher(P);  
    
    % 对每个点计算曲率半径  
    for i = 1:N  
        % 找到当前点的k个近邻点  
        [idx, dist] = knnsearch(kdtree, P(i,:), 'K', k+1);  
        idx = idx(2:end); % 去掉自身点  
        
        % 获取近邻点和对应法向量  
        neighborPoints = P(idx, :);  
        neighborNormals = normals(idx, :);  
        currentNormal = normals(i, :);  
        
        % 计算曲率  
        % 使用法向量变化率来估计曲率  
        normalDiff = sum((neighborNormals - repmat(currentNormal, k, 1)).^2, 2);  
        meanNormalDiff = mean(normalDiff);  
        
        % 计算平均距离  
        meanDist = mean(dist(2:end));  
        
        % 计算曲率半径 (曲率的倒数)  
        if meanNormalDiff > 1e-10  % 避免除零  
            curvatureRadius(i) = meanDist / meanNormalDiff;  
        else  
            curvatureRadius(i) = inf;  
        end  
    end  
    
    % 移除无限大的值和异常值  
    curvatureRadius = curvatureRadius(isfinite(curvatureRadius));  
    
    % 使用IQR方法去除异常值  
    Q1 = prctile(curvatureRadius, 25);  
    Q3 = prctile(curvatureRadius, 75);  
    IQR = Q3 - Q1;  
    validIdx = curvatureRadius >= (Q1 - 1.5*IQR) & curvatureRadius <= (Q3 + 1.5*IQR);  
    curvatureRadius = curvatureRadius(validIdx);  
    
    % 绘制直方图  
    figure;  
    histogram(curvatureRadius, 'Normalization', 'probability');  
    title('点云曲率半径分布');  
    xlabel('曲率半径');  
    ylabel('频率');  
    grid on;  
    
    % 输出统计信息  
    fprintf('曲率半径统计信息：\n');  
    fprintf('最小值：%.4f\n', min(curvatureRadius));  
    fprintf('最大值：%.4f\n', max(curvatureRadius));  
    fprintf('平均值：%.4f\n', mean(curvatureRadius));  
    fprintf('中位数：%.4f\n', median(curvatureRadius));  
    fprintf('标准差：%.4f\n', std(curvatureRadius));  
end