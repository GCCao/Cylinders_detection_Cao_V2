function [minHausdorff] = calculateHausdorffDistance(A, B)
%计算两个点云之间的最近距离

    % 检查输入参数是否为N1*3和N2*3的矩阵
    if size(A, 1) < 1 || size(A, 2) ~= 3 || size(B, 1) < 1 || size(B, 2) ~= 3
        error('输入的点云A和B必须是N*3的矩阵');
    end
    
    % 初始化最小距离为无穷大
    minAB = inf;
    minBA = inf;
    
    % 计算A中每个点到B中所有点的最小距离
    for i = 1:size(A, 1)
        distances = sqrt(sum((A(i,:) - B).^2, 2)); % 计算点A(i)到点B的所有点的距离
        minAB = min(minAB, min(distances)); % 更新minAB
    end
    
    % 计算B中每个点到A中所有点的最小距离
    for i = 1:size(B, 1)
        distances = sqrt(sum((B(i,:) - A).^2, 2)); % 计算点B(i)到点A的所有点的距离
        minBA = min(minBA, min(distances)); % 更新minBA
    end
    
    % 计算Hausdorff距离
    minHausdorff = min(minAB, minBA);
end