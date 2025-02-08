function [startp,endp] = params2endpoints_MY(inlier,vector,point)
%将点云投影到一条空间直线上，并计算起始点

%（1）----------------将点云投影到轴线上，并且提取出起始端点
% 将所有的合法聚类点投影到轴线上
for i=1: size(inlier,1)
    p = inlier(i,:);    
    Points_projed(i,:) = ProjectPointOnLine(point,vector,p');
end

% %可视化投影点云
% hold on
% pcshow(Points_projed,[0,1,0],'MarkerSize',50); %显示点云数据

%轴线法向量的最大分量
[maxComponent,maxComponentIndex] = max( abs(vector') );

% 找到投影点中 maxComponentIndex
% 分量的最大值和最小值,主要是为了确保选错，比如当轴向平行于Z轴时，轴线上所有的点X坐标都一样，靠X坐标没法区分两个端点
max_CompCoord = max(Points_projed(:, maxComponentIndex));
min_CompCoord = min(Points_projed(:, maxComponentIndex));

% 找到对应最大和最小 CompCoord 值的投影点
indices_max = find(Points_projed(:, maxComponentIndex) == max_CompCoord, 1, 'first');
indices_min = find(Points_projed(:, maxComponentIndex) == min_CompCoord, 1, 'first');

% 输出两个端点
startp = Points_projed(indices_min, :);
endp = Points_projed(indices_max, :);


end