function [P_proj,P_F,valid_indices] = project_point_cloud_into_plane(P, normals, a, b, c, angleTH)
    % 输入：
    % P - 点云数据 (N x 3 的矩阵, 每行代表一个点的坐标)
    % normals - 点云的法向量 (N x 3 的矩阵, 每行代表一个点的法向量)
    % a, b, c - 平面F的法向矢量的分量
    
    % 输出：
    % P_proj - 投影后的点云数据 (M x 3 的矩阵, 只有满足条件的点被投影)
    % valid_indices 夹角与90度之差小于10度的点的索引
    % P_F 投影矩阵，P到F的投影矩阵
    
    % 平面法向量
    plane_normal = [a, b, c];
    plane_normal = plane_normal / norm(plane_normal); % 归一化
    
    % 计算每个点的法向量与平面法向量的夹角
    dot_products = sum(normals .* plane_normal, 2);
    angles = acosd(dot_products); % 转为角度
    
    % 找到夹角与90度之差小于10度的点
    valid_indices = abs(angles - 90) < angleTH;
    
    % 只对满足条件的点进行投影
    P_valid = P(valid_indices, :);
    normals_valid = normals(valid_indices, :);
    
    % Step 1: 将P投影到平面F上
       
    % 计算投影矩阵
    I = eye(3); % 单位矩阵
    P_F = I - plane_normal' * plane_normal; % 投影矩阵

    % 将点云投影到平面 F 上
    P_proj = (P_F * P_valid')'; % N x 3 的矩阵

    % % 投影到平面上
    % 
    % % 点云中的每个点与法向量的点积 
    % dot_products = P_valid * plane_normal'; 
    % % 点云投影点的坐标计算 
    % P_proj = P_valid - dot_products * plane_normal;
end