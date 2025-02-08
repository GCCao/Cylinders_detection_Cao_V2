function [surface_area] = compute_area(Points)
% 假设PointCloud是一个Nx3的矩阵，包含点云的XYZ坐标

Points = double(Points);

[t,tnorm]=MyRobustCrust(Points);

figure;
set(gcf,'position',[0,0,1280,800]);
axis equal
title('Points Cloud','fontsize',14)
plot3(Points(:,1),Points(:,2),Points(:,3),'g.')
view(3);
axis equal



%% plot of the output triangulation
figure
title('Output Triangulation','fontsize',14)
axis equal
trisurf(t,Points(:,1),Points(:,2),Points(:,3),'facecolor','c','edgecolor','b')%plot della superficie trattata
view(3);
axis equal


% % 使用Delaunay三角化创建三角形网格
% [triangles] = delaunay(Points(:,1), Points(:,2), Points(:,3));
% 
% % 使用trisurf显示三角网格
% figure
% trisurf(triangles, Points(:,1), Points(:,2), Points(:,3));

% 初始化表面积
surface_area = 0;

% 遍历每个三角形，计算面积并累加
for i = 1:size(triangles, 1)
    % 获取三角形的顶点索引
    idx = triangles(i, :);
    % 提取三角形的顶点坐标
    A = Points(idx(1), :);
    B = Points(idx(2), :);
    C = Points(idx(3), :);
    
    % 计算边长
    a = norm(B - C);
    b = norm(A - C);
    c = norm(A - B);
    
    % 使用海伦公式计算三角形面积
    s = (a + b + c) / 2;
    area = sqrt(s * (s - a) * (s - b) * (s - c));
    
    % 累加面积
    surface_area = surface_area + area;
end

end