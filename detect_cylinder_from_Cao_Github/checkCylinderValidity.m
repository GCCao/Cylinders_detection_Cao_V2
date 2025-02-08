function [isRealCylinder,angle_std] = checkCylinderValidity(axisOrientation, points, Thstd)
% 检查圆柱的合法性
% 通过计算每个点法向与圆柱轴线的夹角，统计夹角的方差

n = size(points,1);
angles = zeros(n, 1); 
Ptemp = pointCloud(points);
normals = surfaceNormalImpl(Ptemp, 30);

for i = 1:n
    % ang = acos(abs(dot(normals(i,:), axisOrientation)));

    n1 = axisOrientation;
    n2 = normals(i,:);

    % Calculate the dot product of n1 and n2
    dot_product = dot(n1, n2);

    % Calculate the magnitudes of n1 and n2
    magnitude_n1 = norm(n1);
    magnitude_n2 = norm(n2);

    % Calculate the cosine of the angle between n1 and n2
    cos_theta = dot_product / (magnitude_n1 * magnitude_n2);

    % Ensure the cosine value is within the valid range [-1, 1]
    cos_theta = max(min(cos_theta, 1), -1);

    % Calculate the angle in radians
    theta_rad = acos(cos_theta);

    % Convert the angle to degrees
    theta_deg = rad2deg(theta_rad);

    angles(i) = abs(theta_deg - 90);
end

% 2. 分析角度分布
angle_std = std(angles);

% 3. 判断标准
if angle_std < Thstd  % 阈值需要根据实际情况调整
    isRealCylinder = true;
else
    isRealCylinder = false;
end


end