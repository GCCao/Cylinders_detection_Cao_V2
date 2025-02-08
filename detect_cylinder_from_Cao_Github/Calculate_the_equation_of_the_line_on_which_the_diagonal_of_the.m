function [Line_direction_vector,pointmin,pointmax] = Calculate_the_equation_of_the_line_on_which_the_diagonal_of_the_min_bounding_rect_lies(points)
%UNTITLED3 此处显示有关此函数的摘要
%   此处显示详细说明


% 计算包围盒的最小和最大坐标
x_min = min(points(:,1));
y_min = min(points(:,2));
z_min = min(points(:,3));
x_max = max(points(:,1));
y_max = max(points(:,2));
z_max = max(points(:,3));

% 确定对角线的两个端点
pointmin = [x_min, y_min, z_min];
pointmax = [x_max, y_max, z_max];

% 确定直线的方向向量
Line_direction_vector = pointmax - pointmin;

end