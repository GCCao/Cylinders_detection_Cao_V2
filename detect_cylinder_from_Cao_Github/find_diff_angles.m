function diff_angles = find_diff_angles(P, A, B)  
% findLargeAngleNormals - 找到点云中法向量与指定轴线方向夹角与 90 度之差大于 10 度的点的索引  
%  
% 用法：  
% indices = findLargeAngleNormals(P, normals, A, B)  
%  
% 输入参数：  
% P       - Nx3 的点云数据矩阵，每行表示一个点的坐标 [x, y, z]  
% normals - Nx3 的法向量矩阵，对应于 P 中的每个点  
% A       - 轴线的第一个点坐标，1x3 向量  
% B       - 轴线的第二个点坐标，1x3 向量  
%  
% 输出参数：
% indices - 满足条件的点的索引向量

%计算点云的法向量
PC = pointCloud(P);
if isempty(PC.Normal)
    % Use 6 neighboring points to estimate a normal vector
    normals = surfaceNormalImpl(PC, 6);
end


% 计算轴线的方向向量并归一化
D = B - A;                     % 轴线的方向向量
D_unit = D / norm(D);          % 归一化为单位向量

% 归一化法向量
normals_magnitude = sqrt(sum(normals.^2, 2));  % 计算每个法向量的模长
normals_unit = normals ./ normals_magnitude;   % 归一化为单位向量

% 计算每个法向量与轴线方向之间的夹角余弦值
cos_theta = normals_unit * D_unit';            % 点乘，得到余弦值

% 修正可能由于数值误差导致的超出 [-1, 1] 范围的值
cos_theta = max(min(cos_theta, 1), -1);

% 计算夹角（以度为单位）
theta = acosd(cos_theta);                      % 反余弦函数得到夹角

% 计算夹角与 90 度之差的绝对值
diff_angles = abs(theta - 90);

   
end