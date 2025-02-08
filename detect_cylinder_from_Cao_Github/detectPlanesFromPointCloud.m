function [planes, inliers,inliers_pionts] = detectPlanesFromPointCloud(P, numPlanes, distanceThreshold)
    % detectPlanesFromPointCloud 从点云数据中检测多个平面
    % 输入：
    %   P - N×3点云数据
    %   numPlanes - 要检测的平面数量
    %   distanceThreshold - 点到平面的距离阈值，用于RANSAC
    % 输出：
    %   planes - 检测到的平面参数，每个平面用一个4维向量表示 [a; b; c; d]
    %   inliers - 每个平面的内点索引

    % 初始化
    planes = [];
    inliers = cell(numPlanes, 1);
    remainingPoints = P;

    for i = 1:numPlanes
        % 使用RANSAC算法检测一个平面
        [plane, inlierIdx] = ransacPlane(remainingPoints, distanceThreshold);

        if isempty(plane)
            break;
        end

        % 保存检测到的平面和内点
        planes = [planes; plane'];
        inliers{i} = inlierIdx;
        inliers_pionts{i}  = remainingPoints(inlierIdx, :);

        % 从点云数据中移除检测到的平面上的点
        remainingPoints(inlierIdx, :) = [];
    end
end

function [plane, inlierIdx] = ransacPlane(P, distanceThreshold)
    % ransacPlane 使用RANSAC算法检测一个平面
    % 输入：
    %   P - N×3点云数据
    %   distanceThreshold - 点到平面的距离阈值
    % 输出：
    %   plane - 检测到的平面参数 [a; b; c; d]
    %   inlierIdx - 平面内点的索引

    maxInliers = 0;
    plane = [];
    inlierIdx = [];

    numPoints = size(P, 1);
    numIterations = 1000; % 迭代次数
    minPoints = 3; % 平面拟合需要的最小点数

    for i = 1:numIterations
        % 随机选择3个点
        indices = randperm(numPoints, minPoints);
        points = P(indices, :);

        % 计算平面参数
        [planeParams] = fitPlane(points);

        % 计算点到平面的距离
        distances = pointToPlaneDistance(P, planeParams);

        % 找到内点
        currentInliers = find(abs(distances) < distanceThreshold);
        numInliers = length(currentInliers);

        % 如果内点数目大于当前最大值，则更新平面
        if numInliers > maxInliers
            maxInliers = numInliers;
            plane = planeParams;
            inlierIdx = currentInliers;
        end
    end
end


function [plane] = fitPlane(points)
    % fitPlane 拟合一个平面到给定的点集
    % 输入： % points - 3×3点集
    % 输出： % plane - 平面参数 [a; b; c; d]
    % D - 平面到点的距离
    % 构造矩阵 A
    A = [points, ones(size(points, 1), 1)];
    [~, ~, V] = svd(A);
    plane = V(:, end);
end

function distances = pointToPlaneDistance(P, planeParams)
    % 计算点到平面的距离
    % 输入:
    %   planeParams - 平面参数 [a; b; c; d]
    %   P - 点云数据 (N×3)
    % 输出:
    %   distances - 每个点到平面的距离

    % 提取平面参数
    a = planeParams(1);
    b = planeParams(2);
    c = planeParams(3);
    d = planeParams(4);

    % 计算点到平面的距离
    distances = abs(a * P(:,1) + b * P(:,2) + c * P(:,3) + d) / sqrt(a^2 + b^2 + c^2);
end