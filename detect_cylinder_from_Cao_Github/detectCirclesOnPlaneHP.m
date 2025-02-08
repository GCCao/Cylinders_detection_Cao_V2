function circles = detectCirclesOnPlaneHP(P_proj, a, b, c, radiusRange)
    % detectCirclesOnPlane 在点云P中使用Hough变换检测位于平面F上的多个圆
    % 输入：
    %   P_proj - 投影后的点云数据 (N x 3 的矩阵)
    %   a, b, c - 平面F的法向量
    %   radiusRange - 圆半径的范围 [minRadius, maxRadius]
    % 输出：
    %   circles - 检测到的圆的信息，返回一个Mx3的矩阵，其中M为检测到的圆的数量，每行表示一个圆的 [圆心x, 圆心y, 半径]

       
    % 2. 创建Hough变换的累加器数组
    % 在这里，我们假设圆心 (x, y) 是投影后的二维坐标，半径 r 是圆的半径
    [rows, cols] = size(P_proj(:,1:2));
    houghAccumulator = zeros(rows, cols, radiusRange(2) - radiusRange(1) + 1);

    % 3. 对每一个点进行Hough变换
    for i = 1:size(P_proj, 1)
        x = P_proj(i, 1);
        y = P_proj(i, 2);
        for r = radiusRange(1):radiusRange(2)
            for theta = 0:359
                a = round(x - r * cosd(theta));
                b = round(y - r * sind(theta));
                if a > 0 && a <= rows && b > 0 && b <= cols
                    houghAccumulator(a, b, r - radiusRange(1) + 1) = ...
                        houghAccumulator(a, b, r - radiusRange(1) + 1) + 1;
                end
            end
        end
    end

    % 4. 从累加器中找到峰值，确定圆的位置和半径
    threshold = max(houghAccumulator(:)) * 0.5;  % 可以根据需求调整阈值
    circles = [];
    for r = radiusRange(1):radiusRange(2)
        [rowIdx, colIdx] = find(houghAccumulator(:,:,r - radiusRange(1) + 1) > threshold);
        if ~isempty(rowIdx)
            detectedCircles = [rowIdx, colIdx, repmat(r, length(rowIdx), 1)];
            circles = [circles; detectedCircles];
        end
    end
end
