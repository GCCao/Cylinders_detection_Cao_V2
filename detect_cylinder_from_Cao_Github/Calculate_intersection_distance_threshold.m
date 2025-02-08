function [Thre_intersection] = Calculate_intersection_distance_threshold(R)
% 根据圆柱体的半径，确定随机采样发矢量投影的交点的距离阈值
% Given data
y = [1 1.5 3 5 7];  % y-values      半径对应的阈值
x = [5 10 50 100 200];  % x-values  对应半径

% Degree of the polynomial (you can change this as needed)
degree = 3;

% Perform polynomial fitting
p = polyfit(x, y, degree);

% Generate x values for plotting the fitted curve

y_fit = polyval(p, R);  % Calculate the corresponding y values

Thre_intersection = y_fit;
end

