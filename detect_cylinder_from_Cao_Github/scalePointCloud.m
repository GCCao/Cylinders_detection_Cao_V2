function P_out = scalePointCloud(P)  
% P: 输入点云，N×3 矩阵，每行表示一个点的坐标 [x,y,z]  
% P_out: 输出点云，如果需要会被放大1000倍  

% 1. 计算最小包围盒  
min_coords = min(P, [], 1);  % 获取x,y,z的最小值  
max_coords = max(P, [], 1);  % 获取x,y,z的最大值  

% 2. 计算包围盒的边长  
box_dimensions = max_coords - min_coords;  

% 3. 找出最长边  
longest_edge = max(box_dimensions);  

Zoomfactor = 500/longest_edge;

% 4. 根据条件决定是否放大  
if longest_edge < 200  
    P_out = P * Zoomfactor;  % 整体放大
    fprintf('点云已放大到1000mm（原最长边：%.2f）\n', longest_edge);  
else  
    P_out = P;  % 保持原始大小  
    fprintf('点云保持原始大小（最长边：%.2f）\n', longest_edge);  
end  

% 5. 显示包围盒信息  
fprintf('包围盒尺寸: \n');  
fprintf('X: %.2f\n', box_dimensions(1));  
fprintf('Y: %.2f\n', box_dimensions(2));  
fprintf('Z: %.2f\n', box_dimensions(3));  

end